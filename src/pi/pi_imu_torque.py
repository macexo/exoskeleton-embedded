"""
pi_imu_torque.py

Combined IMU buffering and torque commanding for the McMaster Exoskeleton Pi.

A background listener thread fans out CAN frames by message type:
  - IMU_ACCEL / IMU_GYRO: paired per joint, downsampled 417 Hz -> 187 Hz with
    a Bresenham accumulator, and pushed into a per-joint deque
    (matches pi_can_buffer.py).
  - Direct extended motor feedback: stored independently per joint.
  - ESTOP: logged to the console.
The foreground loop is an interactive prompt that sends torque commands and
queries the buffered IMU / motor state. It mirrors torque_cmd.py for the
sending side and pi_can_buffer.py for the receiving side.

Use as ML template:
    Replace the prompt loop with an inference loop that:
      1. pulls the latest IMU window via get_imu_data(node_id),
      2. runs the model to produce a desired torque per joint,
      3. sends torque with send_torque(node_id, torque_nm).
    The CAN listener thread keeps IMU and motor-status state fresh in the
    background; the model only needs to read snapshots and emit commands.

Setup (run once on the Pi):
    sudo ip link set can1 up type can bitrate 1000000

Usage:
    python3 pi_imu_torque.py [can_interface]   # default can_interface = can1
"""

import argparse
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path
from time import sleep

import can

try:
    import can_common
    import can_v3
    import can_motor
    import can_system
except ImportError:
    REPO_ROOT = Path(__file__).resolve().parents[2]
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from apis.can.python import can_common, can_v3, can_motor, can_system

# V3 streams sequence-paired raw accel+gyro at 417 Hz. The ML model wants 187 Hz, so we
# downsample with a Bresenham-style accumulator below: every paired sample adds
# `target` to an accumulator and emits one entry whenever it crosses `hz`. This
# spreads the kept samples uniformly without timer jitter.
hz = can_v3.SAMPLE_RATE_HZ
target = 187
imu_receiver = can_v3.ImuReceiver()
motor_map = dict(can_v3.DEFAULT_MOTOR_MAP)
feedback_source = "forwarded"

# AK70-9 KV60 effective torque constant (Kt * gear_ratio). Used only to display
# the implied current to the operator; the STM32 does the real conversion.
KT_EFFECTIVE = 1.431  # Nm/A

# Test-safe display ceiling — matches the STM32's 5 A current clamp.
TORQUE_LIMIT = 5.0 * KT_EFFECTIVE  # ~7.155 Nm

# Joint mapping. Node ID -> short label. The numeric suffix used in the prompt
# (q1, l1, m1, t 1 ...) matches the node ID directly.
NODES = {
    can_common.NODE_LEFT_HIP:   "left_hip",
    can_common.NODE_RIGHT_HIP:  "right_hip",
    can_common.NODE_LEFT_KNEE:  "left_knee",
    can_common.NODE_RIGHT_KNEE: "right_knee",
}


def make_joint_state():
    return {
        # IMU pairing + downsampling state
        "accumulator": 0,
        "queue": deque(maxlen=target),
        # Latest full motor status snapshot (or None until a frame arrives)
        "motor": None,
    }


joints = {node_id: make_joint_state() for node_id in NODES}
state_lock = threading.Lock()

# ── Real-time fault tracking ──
# Edge-triggered: log on state change, not on every frame. We only consider a
# joint "stale" once we've heard from it at least once, so empty buses on
# startup don't produce spurious warnings.
STALE_THRESHOLD_S    = 1.0   # silent for >1s -> stale
WATCHDOG_INTERVAL_S  = 0.5

last_seen           = {nid: None for nid in NODES}
prev_motor_error_pi = {nid: 0 for nid in NODES}
stale_state         = {nid: False for nid in NODES}
status_lock         = threading.Lock()

def stale_watchdog():
    """Edge-log when a known joint goes silent for >STALE_THRESHOLD_S, and
    again when traffic resumes. Runs in its own thread."""
    while True:
        sleep(WATCHDOG_INTERVAL_S)
        now = time.monotonic()
        with status_lock:
            for nid, last in last_seen.items():
                if last is None:
                    continue  # never heard from this joint -> not "stale", just absent
                age = now - last
                is_stale = age > STALE_THRESHOLD_S
                if is_stale and not stale_state[nid]:
                    print(f"\n[STALE] {NODES[nid]} silent for {age:.1f}s")
                    stale_state[nid] = True
                elif not is_stale and stale_state[nid]:
                    print(f"\n[ALIVE] {NODES[nid]} traffic resumed")
                    stale_state[nid] = False


def process_msg(msg: can.Message):
    """Receive V3 sequence-paired IMUs and independent motor feedback."""
    feedback = can_v3.decode_motor_feedback(msg, motor_map, feedback_source)
    if feedback is not None:
        node, motor = feedback
        with state_lock:
            joints[node]["motor"] = motor
        with status_lock:
            if motor["error"] != prev_motor_error_pi[node]:
                print(f"\n[MOTOR] {NODES[node]} fault "
                      f"{prev_motor_error_pi[node]} -> {motor['error']}")
                prev_motor_error_pi[node] = motor["error"]
        return
    if (not msg.is_extended_id and msg.arbitration_id == 0 and msg.dlc == 1
            and len(msg.data) == 1 and not msg.is_remote_frame
            and not msg.is_error_frame and not msg.is_fd):
        print(f"\n[ESTOP] reason {msg.data[0]}")
        return
    pair = imu_receiver.process(msg)
    if pair is None:
        return
    node = pair.node
    with state_lock:
        joint = joints[node]
        if pair.discontinuity:
            joint["queue"].clear()
            joint["accumulator"] = 0
        joint["accumulator"] += target
        if joint["accumulator"] >= hz:
            motor = joint["motor"]
            position = motor["position"] if can_v3.fresh_motor(motor) else None
            # Public tuple shape retained; acceleration m/s², gyro degrees/s.
            joint["queue"].append((pair.timestamp * 1000.0, pair.gyro_dps,
                                   pair.accel_mps2, position))
            joint["accumulator"] -= hz
    with status_lock:
        first = last_seen[node] is None
        last_seen[node] = time.monotonic()
        stale_state[node] = False
    if first:
        print(f"\n[NODE] first complete V3 IMU pair from {NODES[node]}")


def can_listener(bus: can.Bus):
    """Long-running RX loop. Self-recovers from transient bus errors so a
    short cable wiggle doesn't kill the listener thread."""
    while True:
        try:
            msg = bus.recv(timeout=1.0)
        except can.CanError as e:
            print(f"\n[CAN] bus recv error: {e} (retry in 0.5 s)")
            sleep(0.5)
            continue
        if msg is None:
            continue
        try:
            process_msg(msg)
        except Exception as e:
            print(f"\n[CAN] error processing frame 0x{msg.arbitration_id:X}: {e}")


def get_imu_data(node_id: int):
    """Public accessor: return a snapshot of the buffered IMU samples."""
    if node_id not in joints:
        return []
    with state_lock:
        return list(joints[node_id]["queue"])


def get_motor_status(node_id: int):
    """Public accessor: return the latest motor status dict, or None."""
    if node_id not in joints:
        return None
    with state_lock:
        snap = joints[node_id]["motor"]
        if snap is None:
            return None
        result = dict(snap)
    result["age_s"] = max(0.0, time.monotonic() - result["received_at"],
                          (time.time() * 1000.0 - result["timestamp"]) / 1000.0)
    result["stale"] = not can_v3.fresh_motor(result)
    return result


def send_torque(bus: can.Bus, node_id: int, torque_nm: float):
    """Forward a validated request; return whether the local send succeeded."""
    if (type(node_id) is not int or node_id not in NODES
            or not math.isfinite(torque_nm) or abs(torque_nm) > TORQUE_LIMIT):
        raise ValueError(f"joint must be 1..4 and torque finite within +/-{TORQUE_LIMIT:.3f} Nm")
    try:
        can_motor.send_torque_cmd(bus, node_id, torque_nm)
    except can.CanError as e:
        print(f"[CAN] send_torque to {NODES.get(node_id, node_id)} failed: {e}")
        return False
    return True


def _fmt_pos(pos):
    return f"{pos:7.2f}" if pos is not None else "    N/A"


def print_full_queue(node_id: int):
    data = get_imu_data(node_id)
    label = NODES[node_id]
    if not data:
        print(f"[{label}] empty queue, wait a second")
        return

    print(f"\n[{label}] queue snapshot ({len(data)} samples)")
    for i, (timestamp, gyro, accel, motor_pos) in enumerate(data, start=1):
        print(
            f"[{i:3}] Time: {timestamp:.4f} | "
            f"Gyro deg/s: {gyro[0]:6.2f}, {gyro[1]:6.2f}, {gyro[2]:6.2f} | "
            f"Accel m/s²: {accel[0]:6.2f}, {accel[1]:6.2f}, {accel[2]:6.2f} | "
            f"Pos: {_fmt_pos(motor_pos)} deg"
        )
    print()


def print_latest_imu(node_id: int):
    data = get_imu_data(node_id)
    label = NODES[node_id]
    if not data:
        print(f"[{label}] empty queue, wait a second")
        return

    timestamp, gyro, accel, motor_pos = data[-1]
    print(f"\n[{label}] latest IMU sample (queue depth {len(data)})")
    print(f"  time (ms):     {timestamp:.4f}")
    print(f"  gyro deg/s:    {gyro}")
    print(f"  accel m/s²:    {accel}")
    print(f"  motor pos deg: {_fmt_pos(motor_pos)}")
    print()


def print_motor_status(node_id: int):
    snap = get_motor_status(node_id)
    label = NODES[node_id]
    if snap is None:
        print(f"[{label}] no motor status received yet")
        return

    print(f"\n[{label}] last motor feedback "
          f"({'STALE' if snap['stale'] else 'fresh'}, age {snap['age_s']:.3f} s)")
    print(f"  time (ms):   {snap['timestamp']:.4f}")
    print(f"  position:    {snap['position']:.2f} deg")
    print(f"  speed:       {snap['speed']:.0f} ERPM")
    print(f"  phase current: {snap['phase_current']:.2f} A")
    print(f"  temperature: {snap['temperature']} C")
    print(f"  error:       {snap['error']}")
    print()


def print_help():
    print("\ncommands:")
    for nid, label in NODES.items():
        print(f"  q{nid}              IMU queue snapshot for {label}")
    for nid, label in NODES.items():
        print(f"  l{nid}              latest IMU sample for {label}")
    for nid, label in NODES.items():
        print(f"  m{nid}              latest motor status for {label}")
    print("  t <node> <Nm>   send torque to a joint (node = 1..4)")
    print("  t_all <Nm>      request the same torque on all joints (alias: tall)")
    print("  tstop           zero torque on all joints")
    print("  estop           broadcast ESTOP (latches each STM32)")
    print("  h               show this help")
    print("  quit            exit (broadcasts ESTOP first as a safety)")
    print(f"\ntorque range: +/- {TORQUE_LIMIT:.2f} Nm (STM32 clamps to its current limit)")
    print()


def parse_query_command(token: str):
    """Return (kind, node_id) for q<n>/l<n>/m<n> tokens, or (None, None)."""
    if len(token) != 2 or token[0] not in {"q", "l", "m"} or not token[1].isdigit():
        return (None, None)
    node_id = int(token[1])
    if node_id not in NODES:
        return (None, None)
    return (token[0], node_id)


def handle_command(bus: can.Bus, raw: str) -> bool:
    """Dispatch a single line of user input. Returns False to exit."""
    parts = raw.strip().lower().split()
    if not parts:
        return True

    cmd = parts[0]

    if cmd in ("quit", "exit"):
        return False

    if cmd == "h":
        print_help()
        return True

    if cmd == "estop":
        can_system.send_estop(bus, can_common.NODE_PI)
        print("ESTOP broadcast sent")
        return True

    if cmd == "tstop":
        for nid, label in NODES.items():
            if send_torque(bus, nid, 0.0):
                print(f"  {label}: sent zero torque request")
        print("zero torque requests attempted on all joints")
        return True

    if cmd in ("t_all", "tall"):
        if len(parts) != 2:
            print("usage: tall <torque_nm>")
            return True
        try:
            torque = float(parts[1])
        except ValueError:
            print("torque must be a number")
            return True
        if not math.isfinite(torque) or abs(torque) > TORQUE_LIMIT:
            print(f"torque must be finite within +/-{TORQUE_LIMIT:.3f} Nm")
            return True
        for nid, label in NODES.items():
            if send_torque(bus, nid, torque):
                print(f"  {label}: sent request {torque:.3f} Nm (execution unconfirmed)")
        return True

    if cmd == "t":
        if len(parts) != 3:
            print("usage: t <node_id> <torque_nm>")
            return True
        try:
            node_id = int(parts[1])
            torque = float(parts[2])
        except ValueError:
            print("node_id must be int, torque must be float")
            return True
        if node_id not in NODES:
            print(f"unknown node {node_id}. valid: {list(NODES.keys())}")
            return True
        if not math.isfinite(torque) or abs(torque) > TORQUE_LIMIT:
            print(f"torque must be finite within +/-{TORQUE_LIMIT:.3f} Nm")
            return True
        if send_torque(bus, node_id, torque):
            iq = torque / KT_EFFECTIVE
            print(f"sent request {torque:.3f} Nm to {NODES[node_id]} "
                  f"(nominal {iq:.3f} A; execution unconfirmed)")
        return True

    kind, node_id = parse_query_command(cmd)
    if kind == "q":
        print_full_queue(node_id)
        return True
    if kind == "l":
        print_latest_imu(node_id)
        return True
    if kind == "m":
        print_motor_status(node_id)
        return True

    print("unknown command, type 'h' for help")
    return True


def main():
    global motor_map, feedback_source
    parser = argparse.ArgumentParser(description="V3 IMU and torque commissioning console")
    parser.add_argument("channel", nargs="?", default="can1")
    parser.add_argument("--feedback-source", choices=can_v3.FEEDBACK_SOURCES,
                        default="forwarded", help="forwarded PR4 status (default) or raw direct0x29")
    parser.add_argument("--motor-map", type=can_v3.parse_motor_map,
                        default=dict(can_v3.DEFAULT_MOTOR_MAP))
    args = parser.parse_args()
    channel = args.channel
    motor_map = args.motor_map
    feedback_source = args.feedback_source

    print("Exoskeleton V3 SPI IMU + Torque Console")
    print(f"Motor feedback source: {feedback_source}; configured driver map: {motor_map}")
    print("PR4 defaults: only node4->107; other nodes need JOINT_MOTOR_ID assignment")
    print("=" * 50)
    print(f"CAN interface: {channel}")
    print("bitrate:       1 Mbit/s (must be configured at the OS level)")
    print("=" * 50)

    try:
        bus = can_common.create_bus(channel)
    except (OSError, can.CanError) as e:
        print(f"[FATAL] cannot open CAN interface '{channel}': {e}")
        print(f"  hint: sudo ip link set {channel} up type can bitrate 1000000")
        sys.exit(1)

    listener = threading.Thread(target=can_listener, args=(bus,), daemon=True)
    listener.start()

    watchdog = threading.Thread(target=stale_watchdog, daemon=True)
    watchdog.start()

    print("filling buffers...")
    sleep(1)
    print_help()

    try:
        while True:
            try:
                raw = input("> ")
            except EOFError:
                break
            if not handle_command(bus, raw):
                break
    except KeyboardInterrupt:
        print("\nCtrl+C: sending ESTOP...")
        try:
            can_system.send_estop(bus, can_common.NODE_PI)
        except can.CanError as e:
            print(f"failed to send ESTOP: {e}")
    finally:
        try:
            can_system.send_estop(bus, can_common.NODE_PI)
        except can.CanError:
            pass
        bus.shutdown()
        print("CAN bus closed")


if __name__ == "__main__":
    main()
