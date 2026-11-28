"""
pi_inference_loop.py

Closed-loop ML torque controller for the McMaster Exoskeleton Pi, with V3 input.

V3 IMUs are raw sequence-paired SPI streams at 417 Hz. Encoder positions come
from independent forwarded motor status, never from the gyro payload.

Reuses the IMU/motor buffering and CAN listener pattern from pi_imu_torque.py,
but replaces the human prompt with an automated control loop:

    collect IMU window  ->  POST /predict_msgpack  ->  send torque per joint

The REPL only exposes operator-level commands:
    health   - probe the inference server (/health) and verify model is loaded
    start    - launch the closed-loop control loop in a background thread
    estop    - broadcast ESTOP and stop the loop
    status   - print last prediction, cycle stats, and joint liveness
    quit     - exit (broadcasts ESTOP first)

Window assembly follows ml_control_interface.md v1.0.0:
    indices 0-5    left thigh IMU  (accel xyz m/s^2, gyro xyz rad/s)
    indices 6-11   left shank IMU
    indices 12-17  right thigh IMU
    indices 18-23  right shank IMU
    indices 24-27  encoder angles [hip_l, knee_l, hip_r, knee_r] (rad)

Hardware mapping assumption: each joint node carries one IMU. The hip joints
host the thigh IMUs; the knee joints host the shank IMUs. If the wiring is
different, only JOINT_TO_IMU_SLOT below needs to change.

Model output is treated as Nm directly (no body-mass denormalization), per the
operator decision for current bring-up. The STM32 still enforces its 5 A
current clamp.

Setup (run once on the Pi):
    sudo ip link set can1 up type can bitrate 1000000
    # Inference server must be reachable at http://127.0.0.1:8000
    # See exoskeleton-ai/docs/inference.md (Mode A) to start it.

Usage:
    python3 pi_inference_loop.py [can_interface] [assist_scale]
    # can_interface defaults to can1
    # assist_scale is a fraction in [0.0, 1.0] applied to every torque output
    # (e.g. 0.2 = 20% assistance). Defaults to 1.0 (full model output).
"""

import argparse
import csv
import datetime
import math
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from time import sleep

import can
import msgpack

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

# ── Inference server ──
SERVER_BASE   = "http://127.0.0.1:8000"
PREDICT_URL   = f"{SERVER_BASE}/predict_msgpack"
HEALTH_URL    = f"{SERVER_BASE}/health"
WINDOW_SIZE   = 187          # alias of WINDOW_LEN, kept for code that
                             # references the flat tensor size
FEATURE_COUNT = 28
HTTP_TIMEOUT  = 0.050        # 50 ms ceiling per request

# ── Control loop ──
CONTROL_HZ          = 100
CONTROL_PERIOD_S    = 1.0 / CONTROL_HZ
INFERENCE_BUDGET_MS = 10.0   # docs target; overruns trigger hold-last
MAX_CONSECUTIVE_FAILS = 10   # auto-ESTOP after this many bad cycles in a row

# Fraction of model output applied as torque, in [0.0, 1.0]. Overridden by the
# 2nd CLI argument. 1.0 = full assistance, 0.2 = 20%, 0.0 = no torque.
ASSIST_SCALE = 1.0

# Hard safety clamp on the torque actually sent over CAN. Applied AFTER
# ASSIST_SCALE, so the motor never sees |tau| > TORQUE_CLAMP_NM regardless of
# what the model predicts or how the scale is set.
TORQUE_CLAMP_NM = 7.0

# When True, the loop runs inference and updates stats/logs as normal but
# does NOT transmit any TORQUE_CMD frames. Useful for bench validation
# without actuating the motors. Toggle at runtime with the `dryrun` REPL cmd.
DRY_RUN = False

# EMA smoothing of the final torque (post scale, post clamp). Set to None to
# disable. y_t = alpha*x_t + (1-alpha)*y_{t-1}; lower alpha = smoother/laggier.
# Updated every inference cycle (~100 Hz), independent of OUTPUT_HZ.
EMA_ALPHA: float | None = 0.2

# How often torque frames are actually sent on CAN. The loop still runs
# inference + EMA at CONTROL_HZ; only the send is downsampled. Best with a
# divisor of CONTROL_HZ for an even cadence. Default 30 Hz at CONTROL_HZ=100
# rounds to send-every-3 -> ~33.3 Hz effective.
OUTPUT_HZ = 30

# How often the background logger writes a sample to the session CSV.
LOG_INTERVAL_S = 0.200

# Directory where per-session CSVs land. Created on first write.
LOG_DIR = Path(__file__).resolve().parent / "logs"

# ── IMU resampling onto the model's 10 ms grid ──
# The model expects 187 samples spaced exactly 10 ms apart (1.87 s window).
# The STM32 streams paired accel+gyro frames at some rate that does not match
# this grid (V3 nominal rate: 417 Hz). Rather
# than relying on index-based downsampling, we buffer raw timestamped samples
# and linearly interpolate onto the target grid every time we assemble a
# window. This is robust to changes in the input rate and to host jitter.
WINDOW_LEN    = 187                          # samples per model window
WINDOW_DT_S   = 0.010                        # 10 ms per tick (100 Hz grid)
WINDOW_SPAN_S = WINDOW_LEN * WINDOW_DT_S     # 1.87 s total span
# Maximum raw samples kept per joint. Sized to safely cover WINDOW_SPAN_S even
# if the STM32 streams at 1 kHz, with margin for stalls and jitter.
RAW_BUFFER_LEN = 4096

# ── Joint topology ──
NODES = {
    can_common.NODE_LEFT_HIP:   "left_hip",
    can_common.NODE_RIGHT_HIP:  "right_hip",
    can_common.NODE_LEFT_KNEE:  "left_knee",
    can_common.NODE_RIGHT_KNEE: "right_knee",
}

# Maps each CAN node to its slot in the 24-element IMU portion of the feature
# vector. Hip joints host thigh IMUs, knee joints host shank IMUs. If the
# wiring differs, change only this table.
JOINT_TO_IMU_SLOT = {
    can_common.NODE_LEFT_HIP:    0,   # left thigh   -> features 0..5
    can_common.NODE_LEFT_KNEE:   6,   # left shank   -> features 6..11
    can_common.NODE_RIGHT_HIP:  12,   # right thigh  -> features 12..17
    can_common.NODE_RIGHT_KNEE: 18,   # right shank  -> features 18..23
}

# Order in which the 4 model outputs map back to CAN nodes.
# Model order (per ml_control_interface.md): [hip_l, knee_l, hip_r, knee_r]
OUTPUT_NODE_ORDER = [
    can_common.NODE_LEFT_HIP,
    can_common.NODE_LEFT_KNEE,
    can_common.NODE_RIGHT_HIP,
    can_common.NODE_RIGHT_KNEE,
]

# Per-joint sign applied to the final torque before send. Hips are inverted
# because the model's positive-torque convention opposed the pilot's motion
# during bring-up; knees stay as-is.
TORQUE_SIGN = {
    can_common.NODE_LEFT_HIP:   -1.0,
    can_common.NODE_LEFT_KNEE:  +1.0,
    can_common.NODE_RIGHT_HIP:  -1.0,
    can_common.NODE_RIGHT_KNEE: +1.0,
}

# Hip motion gate. Each cycle we look at the last WINDOW_SPAN_S of encoder
# samples per hip; if the peak-to-peak range stays within +/- HIP_MOTION_GATE_DEG
# (i.e. total range <= 2 * HIP_MOTION_GATE_DEG), the hip is treated as "still"
# and its torque is forced to 0. Prevents the assist from pushing the pilot
# into hip flexion when they're standing still. Per-hip; knees are not gated.
HIP_MOTION_GATE_DEG = 5.0
HIP_GATED_NODES = {can_common.NODE_LEFT_HIP, can_common.NODE_RIGHT_HIP}

# Cooldown after a hip gets gated. Once gated, the hip stays gated for AT
# LEAST this long even if encoder range subsequently exceeds the threshold.
# Prevents flicker when the pilot's hip drifts right at the gate boundary
# (range oscillating in the 9-11 deg band would otherwise toggle every cycle
# and cause stop/start jitter on the motor).
HIP_GATE_COOLDOWN_S = 2

# Encoder slot per joint within features 24..27.
# Order: [hip_l (24), knee_l (25), hip_r (26), knee_r (27)]
JOINT_TO_ENCODER_SLOT = {
    can_common.NODE_LEFT_HIP:   24,
    can_common.NODE_LEFT_KNEE:  25,
    can_common.NODE_RIGHT_HIP:  26,
    can_common.NODE_RIGHT_KNEE: 27,
}

DEG_TO_RAD = math.pi / 180.0

# V3 gyro frames contain sequence + raw XYZ, never motor position. Live
# operation requires fresh selected motor feedback for every joint. Synthetic
# encoders are permitted only for explicit dry-run diagnostics.
ALLOW_ENCODER_FALLBACK_ZERO = False
imu_receiver = can_v3.ImuReceiver()
motor_map = dict(can_v3.DEFAULT_MOTOR_MAP)
feedback_source = "forwarded"

# Explicit dry-run diagnostic: force selected encoder columns to zero.
# Live mode ignores these overrides and always requires actual feedback.
FORCE_ZERO_ENCODERS: set[int] = set()

# Dry-run diagnostic: mirror fresh hip feedback onto knee columns.
# A missing hip stays missing unless an explicit zero override is enabled.
MIRROR_KNEE_TO_HIP = False

# Pairing used by MIRROR_KNEE_TO_HIP: knee node -> hip node to copy from.
KNEE_TO_HIP_MIRROR = {
    can_common.NODE_LEFT_KNEE:  can_common.NODE_LEFT_HIP,
    can_common.NODE_RIGHT_KNEE: can_common.NODE_RIGHT_HIP,
}


def make_joint_state():
    return {
        # Raw timestamped samples: deque of (t_s, accel, gyro_rad, enc_rad).
        # t_s is the later host receive timestamp of the two V3 halves.
        # assemble_window() linearly interpolates onto a
        # fixed 10 ms grid from this buffer.
        "samples":         deque(maxlen=RAW_BUFFER_LEN),
        "motor":           None,
        "current_setpoint": None,  # (A, monotonic s) of the latest STM32 setpoint
        # Diagnostics: track recent inter-arrival times so we can verify the
        # actual incoming rate at runtime. ~1 s of history is plenty.
        "intervals_ms":    deque(maxlen=256),
        "last_pair_t_s":   None,
    }


joints      = {nid: make_joint_state() for nid in NODES}
state_lock  = threading.Lock()

# ── Liveness tracking ──
STALE_THRESHOLD_S   = 1.0
WATCHDOG_INTERVAL_S = 0.5
last_seen           = {nid: None for nid in NODES}
last_imu_timestamp  = {nid: None for nid in NODES}
stale_state         = {nid: False for nid in NODES}
status_lock         = threading.Lock()

# ── Loop state ──
loop_thread:    threading.Thread | None = None
loop_stop_evt   = threading.Event()
estop_latched   = threading.Event()
# Serialize reset cache invalidation with the single CAN listener.
input_lock = threading.Lock()
input_after_s = 0.0
loop_state_lock = threading.Lock()
loop_stats = {
    "cycles":          0,
    "infer_fails":     0,
    "overruns":        0,
    "last_pred_nm":    None,  # tuple of 4 Nm, post scale/clamp/sign/gate (sent)
    "last_raw_nm":     None,  # tuple of 4 Nm, raw model output before everything
    "last_cycle_ms":   None,
    "last_infer_ms":   None,
    "running":         False,
    "last_fault":      None,
    "hip_l_range_deg": None,  # peak-to-peak left-hip enc range over last 1.87 s
    "hip_r_range_deg": None,
    "hip_l_gated":     False, # True when the hip motion gate zeroed this hip
    "hip_r_gated":     False,
    "hip_l_cooldown_s": None, # seconds remaining before this hip can un-gate
    "hip_r_cooldown_s": None,
}


# ─────────────────────────── CAN listener ───────────────────────────

def _encoder_for_pair(node, now):
    """Resolve a fresh encoder; caller holds state_lock. No live zero fallback."""
    if DRY_RUN and node in FORCE_ZERO_ENCODERS:
        return 0.0
    source = KNEE_TO_HIP_MIRROR.get(node, node) if DRY_RUN and MIRROR_KNEE_TO_HIP else node
    motor = joints[source]["motor"]
    if can_v3.fresh_motor(motor, now) and motor["error"] == 0:
        return motor["position"] * DEG_TO_RAD
    if DRY_RUN and ALLOW_ENCODER_FALLBACK_ZERO:
        return 0.0
    return None


def process_msg(msg: can.Message):
    with input_lock:
        if can_v3.receive_timestamp(msg) <= input_after_s:
            return
        _process_msg(msg)


def _process_msg(msg: can.Message):
    """Use V3 sequence pairing and motor feedback, converting to model SI units."""
    feedback = can_v3.decode_motor_feedback(msg, motor_map, feedback_source)
    if feedback is not None:
        node, motor = feedback
        with state_lock:
            joints[node]["motor"] = motor
        return
    setpoint = can_v3.decode_current_setpoint(msg)
    if setpoint is not None:
        # Requested current only: never motor feedback or IMU liveness.
        node, current_ma = setpoint
        with state_lock:
            joints[node]["current_setpoint"] = (current_ma / 1000.0, time.monotonic())
        return
    if (not msg.is_extended_id and msg.arbitration_id == 0 and msg.dlc == 1
            and len(msg.data) == 1 and not msg.is_remote_frame
            and not msg.is_error_frame and not msg.is_fd):
        # A software ESTOP from another Pi must stop this sender too.
        estop_latched.set()
        loop_stop_evt.set()
        print(f"\n[ESTOP] reason {msg.data[0]}; control loop stopped")
        return
    pair = imu_receiver.process(msg)
    if pair is None:
        return
    node = pair.node
    now = time.monotonic()
    with state_lock:
        joint = joints[node]
        if pair.discontinuity:
            joint["samples"].clear()
            joint["intervals_ms"].clear()
            joint["last_pair_t_s"] = None
        encoder = _encoder_for_pair(node, now)
        if encoder is None:
            # Refill a complete window after an encoder dropout; never bridge
            # that gap with interpolation or pretend missing feedback is zero.
            joint["samples"].clear()
            joint["intervals_ms"].clear()
            joint["last_pair_t_s"] = None
        else:
            joint["samples"].append(
                (pair.timestamp, pair.accel_mps2, pair.gyro_rads, encoder))
            if joint["last_pair_t_s"] is not None:
                joint["intervals_ms"].append(
                    (pair.timestamp - joint["last_pair_t_s"]) * 1000.0)
            joint["last_pair_t_s"] = pair.timestamp
    # Only complete validated IMU pairs count as IMU liveness. Motor traffic,
    # wrong destinations, malformed frames and lone halves cannot mask loss.
    with status_lock:
        last_seen[node] = now
        last_imu_timestamp[node] = pair.timestamp
        stale_state[node] = False


def can_listener(bus: can.Bus):
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


def stale_watchdog():
    while True:
        sleep(WATCHDOG_INTERVAL_S)
        now = time.monotonic()
        with status_lock:
            for nid, last in last_seen.items():
                if last is None:
                    continue
                age = now - last
                is_stale = age > STALE_THRESHOLD_S
                if is_stale and not stale_state[nid]:
                    print(f"\n[STALE] {NODES[nid]} silent for {age:.1f}s")
                    stale_state[nid] = True
                elif not is_stale and stale_state[nid]:
                    print(f"\n[ALIVE] {NODES[nid]} traffic resumed")
                    stale_state[nid] = False


# ─────────────────────────── Inference HTTP ───────────────────────────

def server_health() -> tuple[bool, str]:
    """Return (ok, message). ok=True only if /health returns 200 {status:ok}."""
    try:
        with urllib.request.urlopen(HEALTH_URL, timeout=2.0) as resp:
            if resp.status != 200:
                return False, f"HTTP {resp.status}"
            body = resp.read()
            try:
                data = msgpack.unpackb(body, raw=False)
            except Exception:
                import json
                data = json.loads(body.decode("utf-8"))
            return (data.get("status") == "ok"), str(data)
    except urllib.error.URLError as e:
        return False, f"unreachable: {e.reason}"
    except Exception as e:
        return False, f"error: {e}"


def predict_msgpack(flat_window: list[float]) -> tuple[list[float], float]:
    """POST /predict_msgpack. Returns (4 Nm/kg outputs in model order, inference_ms).

    Raises on transport, decode, or shape error."""
    body = msgpack.packb(flat_window, use_single_float=True)
    req = urllib.request.Request(
        PREDICT_URL,
        data=body,
        headers={"Content-Type": "application/x-msgpack"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT) as resp:
        if resp.status != 200:
            raise RuntimeError(f"predict HTTP {resp.status}")
        result = msgpack.unpackb(resp.read(), raw=False)
    out = [
        float(result["hip_left"]),
        float(result["knee_left"]),
        float(result["hip_right"]),
        float(result["knee_right"]),
    ]
    infer_ms = float(result.get("inference_ms", 0.0))
    return out, infer_ms


# ─────────────────────────── Window assembly ───────────────────────────

def joint_is_online(nid: int) -> bool:
    """Check paired-IMU freshness directly, independent of watchdog print cadence."""
    with status_lock:
        last = last_seen[nid]
        stamp = last_imu_timestamp[nid]
        return (last is not None and stamp is not None
                and 0 <= time.monotonic() - last < STALE_THRESHOLD_S
                and 0 <= time.time() - stamp < STALE_THRESHOLD_S)


def encoder_is_ready(nid: int) -> bool:
    with state_lock:
        return _encoder_for_pair(nid, time.monotonic()) is not None


def buffers_ready(min_start_t_s: float = 0.0) -> tuple[bool, str]:
    """Live control requires four fresh IMUs, four encoders and a full new window.

    Only explicit dry-run diagnostics may omit joints or use encoder overrides.
    """
    online = [nid for nid in NODES if joint_is_online(nid)]
    if not online:
        return False, "no complete V3 IMU pairs received recently"
    required = online if DRY_RUN else list(NODES)
    for nid in required:
        if nid not in online:
            return False, f"{NODES[nid]} missing/stale V3 IMU pairs"
        if not encoder_is_ready(nid):
            return False, f"{NODES[nid]} missing/stale/faulted motor encoder"
        with state_lock:
            fresh = [sample for sample in joints[nid]["samples"]
                     if sample[0] >= min_start_t_s]
        if len(fresh) < 2:
            return False, f"{NODES[nid]} IMU buffer empty"
        span_s = fresh[-1][0] - fresh[0][0]
        if span_s < WINDOW_SPAN_S:
            return False, f"{NODES[nid]} IMU span {span_s:.2f}s/{WINDOW_SPAN_S:.2f}s"
    return True, "ready"


def _resample_to_grid(samples: list[tuple]) -> list[tuple] | None:
    """Linear-interpolate raw timestamped samples onto the model's grid.

    Input:  list of (t_s, (ax, ay, az), (gx, gy, gz), enc_rad), monotonic in t.
    Output: list of WINDOW_LEN entries, each ((ax, ay, az), (gx, gy, gz),
            enc_rad), at times t_end - (WINDOW_LEN-1-i)*WINDOW_DT_S for i in
            0..WINDOW_LEN-1. The newest grid tick is the most recent sample's
            timestamp; the oldest is WINDOW_SPAN_S - WINDOW_DT_S before that.
    Returns None if samples don't cover the full span."""
    if len(samples) < 2:
        return None
    t_end = samples[-1][0]
    t_start = t_end - (WINDOW_LEN - 1) * WINDOW_DT_S
    if samples[0][0] > t_start:
        return None    # not enough history

    out: list[tuple] = []
    j = 0   # index into samples; advance monotonically since grid is sorted
    n = len(samples)
    for i in range(WINDOW_LEN):
        t = t_start + i * WINDOW_DT_S
        # Advance j so that samples[j].t <= t < samples[j+1].t (or j == n-1).
        while j + 1 < n and samples[j + 1][0] <= t:
            j += 1
        if j + 1 >= n:
            # t is at or past the last sample; just use it.
            _, accel, gyro, enc = samples[-1]
            out.append((accel, gyro, enc))
            continue
        t0, a0, g0, e0 = samples[j]
        t1, a1, g1, e1 = samples[j + 1]
        dt = t1 - t0
        if dt <= 0:
            # Duplicate or out-of-order timestamps -- use the older sample.
            out.append((a0, g0, e0))
            continue
        u = (t - t0) / dt
        accel = (a0[0] + u * (a1[0] - a0[0]),
                 a0[1] + u * (a1[1] - a0[1]),
                 a0[2] + u * (a1[2] - a0[2]))
        gyro  = (g0[0] + u * (g1[0] - g0[0]),
                 g0[1] + u * (g1[1] - g0[1]),
                 g0[2] + u * (g1[2] - g0[2]))
        enc   = e0 + u * (e1 - e0)
        out.append((accel, gyro, enc))
    return out


def hip_encoder_ranges_deg(min_start_t_s: float = 0.0) -> dict[int, float | None]:
    """For each hip joint, return the peak-to-peak encoder range (degrees)
    over its last WINDOW_SPAN_S of post-`start` samples.

    Returns None for a hip if it has no usable samples (offline, or not yet
    enough history). Caller decides what None means."""
    out: dict[int, float | None] = {}
    horizon_s = WINDOW_SPAN_S
    with state_lock:
        for nid in HIP_GATED_NODES:
            samples = joints[nid]["samples"]
            if len(samples) < 2:
                out[nid] = None
                continue
            t_end = samples[-1][0]
            t_cut = max(min_start_t_s, t_end - horizon_s)
            encs_rad = [s[3] for s in samples if s[0] >= t_cut]
            if len(encs_rad) < 2:
                out[nid] = None
                continue
            rng_rad = max(encs_rad) - min(encs_rad)
            out[nid] = rng_rad / DEG_TO_RAD
    return out


def assemble_window(min_start_t_s: float = 0.0) -> list[float] | None:
    """Build a row-major flat list of WINDOW_LEN * FEATURE_COUNT = 5236 float32s.

    Each online joint's raw timestamped samples are linearly interpolated onto
    the model's 10 ms grid (187 ticks spanning 1.87 s, ending at the joint's
    most recent sample). Samples older than `min_start_t_s` are discarded so
    the first inference can never use pre-`start` history. Offline joints get
    zero IMU and zero encoder only during explicit dry-run diagnostics.
    Live mode requires all four joints, encoders and complete windows."""
    if not DRY_RUN and any(not joint_is_online(nid) or not encoder_is_ready(nid)
                           for nid in NODES):
        return None
    flat = [0.0] * (WINDOW_SIZE * FEATURE_COUNT)
    any_filled = False

    with state_lock:
        snapshots: dict[int, list[tuple] | None] = {}
        for nid in NODES:
            if not joint_is_online(nid):
                snapshots[nid] = None
                continue
            snapshots[nid] = [s for s in joints[nid]["samples"]
                              if s[0] >= min_start_t_s]

    for nid, raw in snapshots.items():
        if raw is None:
            continue
        grid = _resample_to_grid(raw)
        if grid is None:
            if not DRY_RUN:
                return None
            continue   # Explicit dry-run diagnostic only.
        any_filled = True
        imu_slot = JOINT_TO_IMU_SLOT[nid]
        enc_slot = JOINT_TO_ENCODER_SLOT[nid]
        for t, (accel, gyro_rad, enc_rad) in enumerate(grid):
            base = t * FEATURE_COUNT + imu_slot
            flat[base + 0] = accel[0]
            flat[base + 1] = accel[1]
            flat[base + 2] = accel[2]
            flat[base + 3] = gyro_rad[0]
            flat[base + 4] = gyro_rad[1]
            flat[base + 5] = gyro_rad[2]
            flat[t * FEATURE_COUNT + enc_slot] = enc_rad

    return flat if any_filled else None


# ─────────────────────────── Control loop ───────────────────────────

def control_loop(bus: can.Bus):
    period = CONTROL_PERIOD_S
    next_tick = time.monotonic()
    last_torques = [0.0, 0.0, 0.0, 0.0]
    ema_state: list[float] | None = None    # None until first prediction lands
    consecutive_fails = 0
    cycle_counter = 0
    # Per-hip monotonic timestamp of when the gate was last engaged. None when
    # the hip is currently un-gated. Used to enforce HIP_GATE_COOLDOWN_S.
    hip_gated_since: dict[int, float | None] = {
        nid: None for nid in HIP_GATED_NODES
    }
    # >=1; how many inference cycles per CAN send. Snapshotted per cycle so
    # runtime knob changes apply on the next tick.
    def _send_every() -> int:
        hz = max(1, int(OUTPUT_HZ))
        return max(1, CONTROL_HZ // hz)

    # Discard any samples buffered before this moment. Without this, sitting
    # at the prompt for a few seconds would let the wait below complete
    # instantly using pre-`start` data, causing inference to fire on a
    # window the operator never expected.
    start_t_s = time.time()
    print(f"[loop] waiting for {WINDOW_SPAN_S:.2f} s of fresh data...")
    last_why = None
    last_print = 0.0
    while not loop_stop_evt.is_set():
        ok, why = buffers_ready(min_start_t_s=start_t_s)
        if ok:
            break
        now = time.monotonic()
        if why != last_why or (now - last_print) > 5.0:
            print(f"[loop] {why}")
            last_why = why
            last_print = now
        if loop_stop_evt.wait(0.5):
            return
    if loop_stop_evt.is_set():
        return

    # Live startup requires all four joints. A partial topology is allowed
    # only in dry-run, which emits no torque requests.
    expected_joints = {nid for nid in NODES if joint_is_online(nid)}
    offline = sorted(NODES[nid] for nid in NODES if nid not in expected_joints)
    if offline:
        print(f"[loop] WARNING: degraded mode -- offline joints (zero-filled): "
              f"{', '.join(offline)}")
        print("[loop] dry-run only: no torque requests are transmitted")
    print("[loop] buffers ready; entering control loop at "
          f"{CONTROL_HZ} Hz")

    with loop_state_lock:
        loop_stats["running"] = True
        loop_stats["last_fault"] = None

    try:
        while not loop_stop_evt.is_set():
            cycle_start = time.monotonic()

            # Recheck both IMU pairing and independent motor feedback.
            required = expected_joints if DRY_RUN else set(NODES)
            stale_now = {NODES[nid] for nid in required
                         if not joint_is_online(nid) or not encoder_is_ready(nid)}
            if stale_now:
                trip_estop(bus, f"joint IMU/encoder missing, stale or faulted: {sorted(stale_now)}")
                return

            window = assemble_window(min_start_t_s=start_t_s)
            if window is None:
                # Should not happen after buffers_ready(); treat as fault.
                trip_estop(bus, "buffer disappeared mid-loop")
                return

            infer_ms: float | None = None
            raw_preds: list[float] | None = None
            try:
                preds, infer_ms = predict_msgpack(window)
                consecutive_fails = 0
                if any((not math.isfinite(v)) for v in preds):
                    trip_estop(bus, f"non-finite prediction {preds}")
                    return
                if infer_ms > INFERENCE_BUDGET_MS:
                    with loop_state_lock:
                        loop_stats["overruns"] += 1
                scaled_clamped = [
                    max(-TORQUE_CLAMP_NM, min(TORQUE_CLAMP_NM, v * ASSIST_SCALE))
                    for v in preds
                ]
                # EMA smoothing (post scale, post clamp). Disabled when
                # EMA_ALPHA is None -- pass values through unchanged.
                alpha = EMA_ALPHA
                if alpha is None:
                    ema_state = None
                    torques = scaled_clamped
                else:
                    if ema_state is None:
                        ema_state = list(scaled_clamped)
                    else:
                        ema_state = [
                            alpha * x + (1.0 - alpha) * y
                            for x, y in zip(scaled_clamped, ema_state)
                        ]
                    torques = list(ema_state)
                # Hold after EMA, before sign/gating; reapply each exactly once per cycle.
                last_torques = torques
                raw_preds = preds
            except Exception as e:
                consecutive_fails += 1
                with loop_state_lock:
                    loop_stats["infer_fails"] += 1
                print(f"\n[loop] inference failed ({consecutive_fails}/"
                      f"{MAX_CONSECUTIVE_FAILS}): {e}")
                if consecutive_fails >= MAX_CONSECUTIVE_FAILS:
                    trip_estop(bus, "inference failed repeatedly")
                    return
                torques = last_torques   # hold-last
                raw_preds = None

            # Apply per-joint sign convention. Flipped values are what the
            # motor actually sees and what we log as `*_nm`.
            torques = [TORQUE_SIGN[nid] * t
                       for nid, t in zip(OUTPUT_NODE_ORDER, torques)]

            # Hip motion gate. Zero out hip torques when the hip encoder has
            # been still (peak-to-peak <= 2 * HIP_MOTION_GATE_DEG over the
            # last 1.87 s of post-`start` data). Once gated, a hip stays
            # gated for at least HIP_GATE_COOLDOWN_S to suppress flicker
            # near the threshold. Knees pass through unchanged.
            hip_ranges = hip_encoder_ranges_deg(min_start_t_s=start_t_s)
            hip_gate_threshold = 2.0 * HIP_MOTION_GATE_DEG
            now_mono = time.monotonic()
            gated_hips: set[int] = set()
            for i, nid in enumerate(OUTPUT_NODE_ORDER):
                if nid not in HIP_GATED_NODES:
                    continue
                rng = hip_ranges.get(nid)
                below_threshold = (rng is not None and rng <= hip_gate_threshold)
                in_cooldown = (
                    hip_gated_since[nid] is not None
                    and (now_mono - hip_gated_since[nid]) < HIP_GATE_COOLDOWN_S
                )
                gate_active = below_threshold or in_cooldown
                if gate_active:
                    torques[i] = 0.0
                    gated_hips.add(nid)
                    if hip_gated_since[nid] is None:
                        hip_gated_since[nid] = now_mono   # first cycle gated
                else:
                    hip_gated_since[nid] = None           # release timer

            send_every = _send_every()
            do_send = (cycle_counter % send_every == 0)
            if do_send and not DRY_RUN:
                if loop_stop_evt.is_set():
                    return
                if any(not joint_is_online(nid) or not encoder_is_ready(nid) for nid in NODES):
                    trip_estop(bus, "V3 IMU/encoder became unavailable during inference")
                    return
                for nid, torque in zip(OUTPUT_NODE_ORDER, torques):
                    if loop_stop_evt.is_set() or estop_latched.is_set():
                        return
                    try:
                        can_motor.send_torque_cmd(bus, nid, float(torque))
                    except can.CanError as e:
                        trip_estop(bus, f"torque send to {NODES[nid]} failed: {e}")
                        return
            cycle_counter += 1

            cycle_ms = (time.monotonic() - cycle_start) * 1000.0
            with loop_state_lock:
                loop_stats["cycles"] += 1
                loop_stats["last_pred_nm"]  = tuple(torques)
                loop_stats["last_raw_nm"]   = (tuple(raw_preds)
                                               if raw_preds is not None else None)
                loop_stats["last_cycle_ms"] = cycle_ms
                loop_stats["last_infer_ms"] = infer_ms
                loop_stats["hip_l_range_deg"] = hip_ranges.get(
                    can_common.NODE_LEFT_HIP)
                loop_stats["hip_r_range_deg"] = hip_ranges.get(
                    can_common.NODE_RIGHT_HIP)
                loop_stats["hip_l_gated"] = (can_common.NODE_LEFT_HIP
                                             in gated_hips)
                loop_stats["hip_r_gated"] = (can_common.NODE_RIGHT_HIP
                                             in gated_hips)
                for nid, key in ((can_common.NODE_LEFT_HIP,  "hip_l_cooldown_s"),
                                 (can_common.NODE_RIGHT_HIP, "hip_r_cooldown_s")):
                    since = hip_gated_since[nid]
                    if since is None:
                        loop_stats[key] = None
                    else:
                        remaining = HIP_GATE_COOLDOWN_S - (now_mono - since)
                        loop_stats[key] = max(0.0, remaining)

            # Pace to fixed cadence. If we already missed the deadline, fire
            # immediately and re-anchor next_tick to now to avoid catch-up
            # storms.
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                if loop_stop_evt.wait(sleep_for):
                    break
            else:
                with loop_state_lock:
                    loop_stats["overruns"] += 1
                next_tick = time.monotonic()
    finally:
        # Always zero torques on exit (skip in dry-run -- we never sent any).
        if not DRY_RUN:
            for nid in OUTPUT_NODE_ORDER:
                try:
                    can_motor.send_torque_cmd(bus, nid, 0.0)
                except can.CanError:
                    pass
        with loop_state_lock:
            loop_stats["running"] = False
        print("[loop] stopped"
              + ("" if DRY_RUN else "; zero torque requests attempted"))


# Joint order used for all per-joint log columns. Stable so column indices
# don't shift if NODES dict iteration order ever changes.
_LOG_JOINT_ORDER = [
    can_common.NODE_LEFT_HIP,
    can_common.NODE_LEFT_KNEE,
    can_common.NODE_RIGHT_HIP,
    can_common.NODE_RIGHT_KNEE,
]


def _per_joint_cols(suffixes: list[str]) -> list[str]:
    cols = []
    for nid in _LOG_JOINT_ORDER:
        label = NODES[nid]
        for s in suffixes:
            cols.append(f"{label}_{s}")
    return cols


_LOG_COLUMNS = (
    [
        "monotonic_s", "wall_iso",
        "running", "dry_run",
        "cycles", "infer_fails", "overruns",
        "last_cycle_ms", "last_infer_ms",
        "assist_scale", "clamp_nm",
        "ema_alpha", "output_hz",
        "force_zero", "mirror_knee",
        "hip_l_raw", "knee_l_raw", "hip_r_raw", "knee_r_raw",
        "hip_l_nm",  "knee_l_nm",  "hip_r_nm",  "knee_r_nm",
        "hip_l_range_deg", "hip_r_range_deg",
        "hip_l_gated", "hip_r_gated",
        "hip_l_cooldown_s", "hip_r_cooldown_s",
    ]
    + _per_joint_cols(["status", "age_s", "samples", "span_s", "input_hz"])
    + _per_joint_cols(["ax", "ay", "az", "gx", "gy", "gz", "enc_rad"])
    + _per_joint_cols(["motor_pos_deg", "motor_speed_erpm",
                       "motor_phase_current_a", "motor_temp_c", "motor_error",
                       "motor_status", "motor_age_s"])
    + ["last_fault"]
    # Current comparisons, appended so existing column positions are unchanged.
    # current_setpoint_a matches the joint-controller README's CSV field name.
    + ["current_calc_kt_nm_per_a"]
    + _per_joint_cols(["raw_pred_current_calc_a"])
    + _per_joint_cols(["current_setpoint_a", "current_setpoint_age_s"])
)


def _log_row() -> list:
    with loop_state_lock:
        stats = dict(loop_stats)
    raw  = stats["last_raw_nm"]  or (None, None, None, None)
    sent = stats["last_pred_nm"] or (None, None, None, None)

    # Snapshot per-joint live state under state_lock; everything we read here
    # is mutated in process_msg.
    per_joint: dict[int, dict] = {}
    with state_lock:
        for nid in _LOG_JOINT_ORDER:
            j = joints[nid]
            samples = j["samples"]
            if len(samples) > 0:
                t_s, accel, gyro, enc = samples[-1]
                latest_imu = (accel, gyro)
                latest_enc = enc
                span_s = (samples[-1][0] - samples[0][0]
                          if len(samples) > 1 else 0.0)
            else:
                latest_imu = None
                latest_enc = None
                span_s = 0.0
            intervals = list(j["intervals_ms"])
            mean_dt_ms = (sum(intervals) / len(intervals)) if intervals else None
            input_hz = (1000.0 / mean_dt_ms) if mean_dt_ms else None
            per_joint[nid] = {
                "samples":  len(samples),
                "span_s":   span_s,
                "input_hz": input_hz,
                "imu":      latest_imu,
                "enc_rad":  latest_enc,
                "motor":    dict(j["motor"]) if j["motor"] is not None else None,
                "current_setpoint": j["current_setpoint"],
            }

    now = time.monotonic()
    with status_lock:
        joint_status: dict[int, str] = {}
        joint_age: dict[int, float | None] = {}
        for nid in _LOG_JOINT_ORDER:
            last = last_seen[nid]
            joint_status[nid] = (
                "stale" if stale_state[nid]
                else ("ok" if last is not None else "absent")
            )
            joint_age[nid] = (now - last) if last is not None else None

    force_zero_str = "|".join(
        sorted(NODES[nid] for nid in FORCE_ZERO_ENCODERS)
    )
    wall = datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="milliseconds")

    row: list = [
        f"{now:.3f}", wall,
        stats["running"], DRY_RUN,
        stats["cycles"], stats["infer_fails"], stats["overruns"],
        stats["last_cycle_ms"], stats["last_infer_ms"],
        ASSIST_SCALE, TORQUE_CLAMP_NM,
        EMA_ALPHA, OUTPUT_HZ,
        force_zero_str, MIRROR_KNEE_TO_HIP,
        raw[0], raw[1], raw[2], raw[3],
        sent[0], sent[1], sent[2], sent[3],
        stats["hip_l_range_deg"], stats["hip_r_range_deg"],
        stats["hip_l_gated"], stats["hip_r_gated"],
        stats["hip_l_cooldown_s"], stats["hip_r_cooldown_s"],
    ]

    # status / age / raw buffer counts + measured input rate, joint-major
    for nid in _LOG_JOINT_ORDER:
        pj  = per_joint[nid]
        age = joint_age[nid]
        row += [
            joint_status[nid],
            f"{age:.3f}" if age is not None else None,
            pj["samples"],
            f"{pj['span_s']:.3f}",
            f"{pj['input_hz']:.1f}" if pj["input_hz"] is not None else None,
        ]

    # latest IMU sample + encoder per joint
    for nid in _LOG_JOINT_ORDER:
        pj = per_joint[nid]
        if pj["imu"] is not None:
            accel, gyro = pj["imu"]
            row += [accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]]
        else:
            row += [None, None, None, None, None, None]
        row += [pj["enc_rad"]]

    # motor status per joint
    for nid in _LOG_JOINT_ORDER:
        m = per_joint[nid]["motor"]
        if m is not None:
            row += [m["position"], m["speed"], m["phase_current"],
                    m["temperature"], m["error"],
                    "fresh" if can_v3.fresh_motor(m, now) else "stale",
                    max(0.0, now - m["received_at"],
                        (time.time() * 1000.0 - m["timestamp"]) / 1000.0)]
        else:
            row += [None, None, None, None, None, "missing", None]

    row.append(stats["last_fault"])

    # Calculated from the raw prediction, requested by the STM32 and reported by
    # the driver (motor_phase_current_a): separate quantities, never merged.
    # Missing values stay blank; the last setpoint is kept with its age.
    row.append(can_v3.NOMINAL_KT_NM_PER_A)
    raw_by_node = dict(zip(OUTPUT_NODE_ORDER, raw))

    for nid in _LOG_JOINT_ORDER:
        predicted = raw_by_node[nid]
        row.append(predicted / can_v3.NOMINAL_KT_NM_PER_A if predicted is not None else None)

    for nid in _LOG_JOINT_ORDER:
        current = per_joint[nid]["current_setpoint"]

        if current is None:
            row += [None, None]
        else:
            row += [current[0], f"{max(0.0, now - current[1]):.3f}"]

    return row


def session_logger(log_path: Path, stop_evt: threading.Event):
    """Append one CSV row every LOG_INTERVAL_S until stop_evt is set."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    with open(log_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(_LOG_COLUMNS)
        fh.flush()
        next_tick = time.monotonic()
        while not stop_evt.is_set():
            try:
                writer.writerow(_log_row())
                fh.flush()
            except Exception as e:
                print(f"\n[log] write failed: {e}")
            next_tick += LOG_INTERVAL_S
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                if stop_evt.wait(sleep_for):
                    break
            else:
                next_tick = time.monotonic()


def trip_estop(bus: can.Bus, reason: str):
    estop_latched.set()
    loop_stop_evt.set()
    print(f"\n[FAULT] {reason} -- broadcasting ESTOP")
    with loop_state_lock:
        loop_stats["last_fault"] = reason
    try:
        can_system.send_estop(bus, can_common.NODE_PI,
                              can_system.ESTOP_SOFTWARE)
    except can.CanError as e:
        print(f"[FAULT] ESTOP send failed: {e}")
    loop_stop_evt.set()


# ─────────────────────────── REPL ───────────────────────────

def cmd_health() -> bool:
    ok, msg = server_health()
    print(f"  /health: {'OK' if ok else 'FAIL'} - {msg}")
    if not ok:
        return False
    print("  probing /predict_msgpack with zero window...")
    try:
        zeros = [0.0] * (WINDOW_SIZE * FEATURE_COUNT)
        preds, infer_ms = predict_msgpack(zeros)
        print(f"  predict OK: outputs={['%.4f' % v for v in preds]}  "
              f"inference_ms={infer_ms:.2f}")
        return True
    except Exception as e:
        print(f"  predict FAIL: {e}")
        return False


def cmd_start(bus: can.Bus):
    global loop_thread
    if loop_thread is not None and loop_thread.is_alive():
        print("loop already running")
        return
    with input_lock:
        if estop_latched.is_set():
            print("ESTOP is latched locally; send reset, then collect fresh input before restart")
            return
        loop_stop_evt.clear()
    with loop_state_lock:
        loop_stats["cycles"] = 0
        loop_stats["infer_fails"] = 0
        loop_stats["overruns"] = 0
    loop_thread = threading.Thread(
        target=control_loop, args=(bus,), daemon=True
    )
    loop_thread.start()
    print("loop started (use 'estop' to stop, 'status' to inspect)")


def cmd_estop(bus: can.Bus):
    estop_latched.set()
    loop_stop_evt.set()
    try:
        can_system.send_estop(bus, can_common.NODE_PI)
        print("ESTOP broadcast sent")
    except can.CanError as e:
        print(f"ESTOP send failed: {e}")
    if loop_thread is not None and loop_thread.is_alive():
        loop_thread.join(timeout=1.0)


# PR4 retains the last-year reset request, handled by motor-enabled boards.
GLOBAL_RESET_CAN_ID = 0x67


def _clear_telemetry():
    """Discard pre-reset input; caller holds input_lock to exclude the listener."""
    global imu_receiver, input_after_s
    input_after_s = time.time()
    imu_receiver = can_v3.ImuReceiver()
    with state_lock:
        for joint in joints.values():
            joint.update(make_joint_state())
    with status_lock:
        for node in NODES:
            last_seen[node] = None
            last_imu_timestamp[node] = None
            stale_state[node] = False
    with loop_state_lock:
        loop_stats["last_pred_nm"] = None
        loop_stats["last_raw_nm"] = None


def cmd_reset(bus: can.Bus):
    """Cancel the sender before requesting reset; sending is not reset confirmation."""
    estop_latched.set()
    loop_stop_evt.set()
    if loop_thread is not None and loop_thread.is_alive():
        loop_thread.join(timeout=1.0)
        if loop_thread.is_alive():
            print("Reset request withheld: controller has not stopped; ESTOP remains latched")
            return False
    # Reuse the previous controller's standard, empty 0x67 frame. Invalidate
    # both cached data and in-flight old receive timestamps around the send.
    with input_lock:
        _clear_telemetry()
        msg = can.Message(arbitration_id=GLOBAL_RESET_CAN_ID, is_extended_id=False, data=[])
        try:
            bus.send(msg)
        except (OSError, can.CanError) as error:
            print(f"Global reset send failed: {error}; ESTOP remains latched locally")
            return False
        _clear_telemetry()
        estop_latched.clear()
    print("Global reset request sent (ID 0x067); board resets are unconfirmed")
    print("  PR4 handles reset only on motor-enabled nodes; start requires fresh input")
    return True


JOINT_NAME_TO_NODE = {name: nid for nid, name in NODES.items()}


def _format_forced_joints() -> str:
    if not FORCE_ZERO_ENCODERS:
        return "{} (no joints forced)"
    names = sorted(NODES[nid] for nid in FORCE_ZERO_ENCODERS)
    return "{" + ", ".join(names) + "}"


def cmd_zeroenc(parts: list[str]):
    """Show or set FORCE_ZERO_ENCODERS (per-joint) at runtime."""
    global FORCE_ZERO_ENCODERS
    if len(parts) == 1:
        print(f"FORCE_ZERO_ENCODERS = {_format_forced_joints()}")
        return
    if not DRY_RUN:
        print("zeroenc overrides require dry-run; live V3 control requires actual feedback")
        return
    args = parts[1:]
    if len(args) == 1 and args[0] in ("none", "clear", "off"):
        FORCE_ZERO_ENCODERS = set()
    elif len(args) == 1 and args[0] in ("all", "on"):
        FORCE_ZERO_ENCODERS = set(NODES.keys())
    else:
        unknown = [a for a in args if a not in JOINT_NAME_TO_NODE]
        if unknown:
            print(f"zeroenc: unknown joint(s): {', '.join(unknown)}")
            print(f"  valid joints: {', '.join(sorted(JOINT_NAME_TO_NODE))}")
            print("  or: all, none")
            return
        FORCE_ZERO_ENCODERS = {JOINT_NAME_TO_NODE[a] for a in args}
    print(f"FORCE_ZERO_ENCODERS = {_format_forced_joints()}")
    if FORCE_ZERO_ENCODERS:
        print("  encoder columns for those joints will be zero on subsequent ticks")
    else:
        print("  encoder columns will reflect real motor position again")


def cmd_mirrorknee(parts: list[str]):
    """Toggle or set MIRROR_KNEE_TO_HIP at runtime."""
    global MIRROR_KNEE_TO_HIP
    if not DRY_RUN:
        print("mirrorknee requires dry-run; live V3 control requires each joint's encoder")
        return
    if len(parts) == 1:
        MIRROR_KNEE_TO_HIP = not MIRROR_KNEE_TO_HIP
    elif len(parts) == 2 and parts[1] in ("on", "off"):
        MIRROR_KNEE_TO_HIP = (parts[1] == "on")
    else:
        print("usage: mirrorknee            (toggle)")
        print("       mirrorknee on | off   (set explicitly)")
        return
    state = "ON" if MIRROR_KNEE_TO_HIP else "OFF"
    print(f"MIRROR_KNEE_TO_HIP = {state}")
    if MIRROR_KNEE_TO_HIP:
        print("  left_knee encoder copies left_hip; right_knee copies right_hip")
        if FORCE_ZERO_ENCODERS:
            print(f"  (FORCE_ZERO_ENCODERS takes precedence for: "
                  f"{_format_forced_joints()})")
    else:
        print("  knees use their own encoder source again")


def cmd_assist(parts: list[str]):
    """Show or set ASSIST_SCALE at runtime."""
    global ASSIST_SCALE
    if len(parts) == 1:
        print(f"ASSIST_SCALE = {ASSIST_SCALE:.2f} "
              f"({ASSIST_SCALE * 100:.0f}% of model output)")
        return
    if len(parts) != 2:
        print("usage: assist             (show current value)")
        print("       assist <0.0..1.0>  (set scale, e.g. 0.2 for 20%)")
        return
    try:
        scale = float(parts[1])
    except ValueError:
        print(f"assist: not a number: {parts[1]!r}")
        return
    if not (0.0 <= scale <= 1.0):
        print(f"assist: must be in [0.0, 1.0], got {scale}")
        return
    ASSIST_SCALE = scale
    print(f"ASSIST_SCALE = {ASSIST_SCALE:.2f} "
          f"({ASSIST_SCALE * 100:.0f}% of model output)")
    print("  takes effect on the next control cycle")


def cmd_ema(parts: list[str]):
    """Show or set EMA_ALPHA at runtime. `ema off` disables smoothing."""
    global EMA_ALPHA
    if len(parts) == 1:
        if EMA_ALPHA is None:
            print("EMA disabled")
        else:
            print(f"EMA_ALPHA = {EMA_ALPHA:.2f}")
        return
    if len(parts) != 2:
        print("usage: ema             (show current value)")
        print("       ema <0.0..1.0>  (set alpha; smaller = smoother)")
        print("       ema off         (disable smoothing)")
        return
    if parts[1] == "off":
        EMA_ALPHA = None
        print("EMA disabled")
        return
    try:
        alpha = float(parts[1])
    except ValueError:
        print(f"ema: not a number: {parts[1]!r}")
        return
    if not (0.0 < alpha <= 1.0):
        print(f"ema: alpha must be in (0.0, 1.0], got {alpha}")
        return
    EMA_ALPHA = alpha
    print(f"EMA_ALPHA = {EMA_ALPHA:.2f}")
    print("  applied on next cycle; state resets if previously disabled")


def cmd_outrate(parts: list[str]):
    """Show or set OUTPUT_HZ at runtime."""
    global OUTPUT_HZ
    if len(parts) == 1:
        send_every = max(1, CONTROL_HZ // max(1, int(OUTPUT_HZ)))
        print(f"OUTPUT_HZ = {OUTPUT_HZ} "
              f"(send every {send_every} cycles of {CONTROL_HZ} Hz loop)")
        return
    if len(parts) != 2:
        print("usage: outrate            (show current value)")
        print(f"       outrate <hz>       (1..{CONTROL_HZ}; divisor of "
              f"{CONTROL_HZ} for even cadence)")
        return
    try:
        hz = int(parts[1])
    except ValueError:
        print(f"outrate: not an integer: {parts[1]!r}")
        return
    if not (1 <= hz <= CONTROL_HZ):
        print(f"outrate: must be in [1, {CONTROL_HZ}], got {hz}")
        return
    OUTPUT_HZ = hz
    send_every = max(1, CONTROL_HZ // OUTPUT_HZ)
    actual_hz = CONTROL_HZ / send_every
    print(f"OUTPUT_HZ = {OUTPUT_HZ} "
          f"(send every {send_every} cycles -> ~{actual_hz:.1f} Hz effective)")
    if actual_hz != hz:
        print(f"  note: {CONTROL_HZ} Hz / {hz} = {CONTROL_HZ / hz:.2f}, "
              "rounded to nearest integer divisor")


def cmd_dryrun(parts: list[str]):
    """Toggle or set DRY_RUN at runtime."""
    global DRY_RUN
    if loop_thread is not None and loop_thread.is_alive():
        print("stop the loop before changing dry-run mode; then collect a fresh input window")
        return
    if len(parts) == 1:
        DRY_RUN = not DRY_RUN
    elif len(parts) == 2 and parts[1] in ("on", "off"):
        DRY_RUN = (parts[1] == "on")
    else:
        print("usage: dryrun            (toggle)")
        print("       dryrun on | off   (set explicitly)")
        return
    state = "ON" if DRY_RUN else "OFF"
    print(f"DRY_RUN = {state}")
    if DRY_RUN:
        print("  inference still runs; no TORQUE_CMD frames will be sent")
    else:
        FORCE_ZERO_ENCODERS.clear()
        with state_lock:
            for joint in joints.values():
                joint["samples"].clear()
                joint["intervals_ms"].clear()
                joint["last_pair_t_s"] = None
        print("  live mode selected; start to collect a full fresh input window")


def cmd_status():
    with loop_state_lock:
        stats = dict(loop_stats)
    print(f"  running:               {stats['running']}")
    print(f"  cycles:                {stats['cycles']}")
    print(f"  infer_fails:           {stats['infer_fails']}")
    print(f"  overruns:              {stats['overruns']}")
    print(f"  FORCE_ZERO_ENCODERS:   {_format_forced_joints()}")
    print(f"  MIRROR_KNEE_TO_HIP:    {'ON' if MIRROR_KNEE_TO_HIP else 'OFF'}")
    print(f"  ASSIST_SCALE:          {ASSIST_SCALE:.2f}")
    print(f"  TORQUE_CLAMP_NM:       +/- {TORQUE_CLAMP_NM:.2f}")
    print(f"  DRY_RUN:               {'ON' if DRY_RUN else 'OFF'}")
    if EMA_ALPHA is None:
        print(f"  EMA_ALPHA:             OFF")
    else:
        print(f"  EMA_ALPHA:             {EMA_ALPHA:.2f}")
    send_every = max(1, CONTROL_HZ // max(1, int(OUTPUT_HZ)))
    print(f"  OUTPUT_HZ:             {OUTPUT_HZ} "
          f"(send every {send_every} cycles -> {CONTROL_HZ / send_every:.1f} Hz)")
    if stats["last_cycle_ms"] is not None:
        print(f"  last_cycle_ms:         {stats['last_cycle_ms']:.2f}")
    if stats["last_infer_ms"] is not None:
        print(f"  last_infer_ms:         {stats['last_infer_ms']:.2f}")
    if stats["last_pred_nm"] is not None:
        labels = ["hip_l", "knee_l", "hip_r", "knee_r"]
        for label, v in zip(labels, stats["last_pred_nm"]):
            print(f"  {label:>8}:              {v:+.4f} Nm")
    gate_thr = 2.0 * HIP_MOTION_GATE_DEG
    for side, rng_key, gated_key, cd_key in (
        ("hip_l", "hip_l_range_deg", "hip_l_gated", "hip_l_cooldown_s"),
        ("hip_r", "hip_r_range_deg", "hip_r_gated", "hip_r_cooldown_s"),
    ):
        rng = stats[rng_key]
        cd = stats[cd_key]
        state_str = ("GATED" if stats[gated_key] else "active")
        if cd is not None and cd > 0:
            state_str += f", cooldown {cd:.2f}s"
        if rng is None:
            print(f"  {side} range:           -- ({state_str})")
        else:
            print(f"  {side} range:           {rng:5.2f} deg "
                  f"(threshold {gate_thr:.1f}, {state_str})")
    if stats["last_fault"]:
        print(f"  last_fault:            {stats['last_fault']}")
    with status_lock:
        joint_marks = {
            nid: ("stale" if stale_state[nid]
                  else ("ok" if last_seen[nid] is not None else "absent"))
            for nid in NODES
        }
    # Per-joint input diagnostics (mean inter-pair interval -> Hz, raw buffer
    # span). The model needs span >= 1.87 s to assemble a window.
    with state_lock:
        diag = {}
        for nid in NODES:
            samples = joints[nid]["samples"]
            intervals = list(joints[nid]["intervals_ms"])
            mean_dt = (sum(intervals) / len(intervals)) if intervals else None
            span_s = (samples[-1][0] - samples[0][0]
                      if len(samples) > 1 else 0.0)
            diag[nid] = (len(samples), span_s, mean_dt)
    for nid, label in NODES.items():
        n_samp, span_s, mean_dt = diag[nid]
        hz_s = f"{1000.0 / mean_dt:5.1f} Hz" if mean_dt else "  -- Hz"
        dt_s = f"{mean_dt:5.2f} ms" if mean_dt else "  -- ms"
        print(f"  {label:>10}:           {joint_marks[nid]:>6}  "
              f"samples={n_samp:4d}  span={span_s:4.2f}s  "
              f"dt={dt_s}  rate={hz_s}")


def print_help():
    print("\ncommands:")
    print("  health             probe /health and /predict_msgpack with zeros")
    print("  start              begin closed-loop control (collect -> infer -> torque)")
    print("  estop              broadcast ESTOP and stop the loop")
    print("  reset              stop loop, request PR4 reset (0x67), discard input history")
    print("  status             show cycle stats, last prediction, joint liveness")
    print("  zeroenc [<joint>...|all|none]  show/set which joints force encoder=0")
    print("  mirrorknee [on|off] toggle (or set) MIRROR_KNEE_TO_HIP at runtime")
    print("  assist [<0.0..1.0>] show or set assist scale (fraction of model output)")
    print("  dryrun [on|off]    toggle (or set) dry-run mode (suppress torque sends)")
    print("  ema [<alpha>|off]  show/set EMA smoothing on the output torque")
    print("  outrate [<hz>]     show/set torque send rate (Hz; default 10)")
    print("  h                  show this help")
    print("  quit               exit (broadcasts ESTOP first)")
    print()


def handle_command(bus: can.Bus, raw: str) -> bool:
    parts = raw.strip().lower().split()
    if not parts:
        return True
    cmd = parts[0]
    if cmd in ("quit", "exit"):
        return False
    if cmd == "h" or cmd == "help":
        print_help()
        return True
    if cmd == "health":
        cmd_health()
        return True
    if cmd == "start":
        cmd_start(bus)
        return True
    if cmd == "estop":
        cmd_estop(bus)
        return True
    if cmd == "reset":
        cmd_reset(bus)
        return True
    if cmd == "status":
        cmd_status()
        return True
    if cmd == "zeroenc":
        cmd_zeroenc(parts)
        return True
    if cmd == "mirrorknee":
        cmd_mirrorknee(parts)
        return True
    if cmd == "assist":
        cmd_assist(parts)
        return True
    if cmd == "dryrun":
        cmd_dryrun(parts)
        return True
    if cmd == "ema":
        cmd_ema(parts)
        return True
    if cmd == "outrate":
        cmd_outrate(parts)
        return True
    print("unknown command, type 'h' for help")
    return True


# ─────────────────────────── main ───────────────────────────

def main():
    global ASSIST_SCALE, motor_map, feedback_source, DRY_RUN
    parser = argparse.ArgumentParser(description="V3 SPI IMU torque-controller Pi application")
    parser.add_argument("channel", nargs="?", default="can1")
    parser.add_argument("assist_scale", nargs="?", type=float, default=1.0)
    parser.add_argument("--feedback-source", choices=can_v3.FEEDBACK_SOURCES,
                        default="forwarded", help="forwarded PR4 status (default) or raw direct0x29")
    parser.add_argument("--motor-map", type=can_v3.parse_motor_map,
                        default=dict(can_v3.DEFAULT_MOTOR_MAP))
    parser.add_argument("--dry-run", action="store_true", help="run diagnostics without torque output")
    args = parser.parse_args()
    if not math.isfinite(args.assist_scale) or not 0.0 <= args.assist_scale <= 1.0:
        parser.error("assist_scale must be finite and within [0, 1]")
    channel = args.channel
    motor_map = args.motor_map
    feedback_source = args.feedback_source
    ASSIST_SCALE = args.assist_scale
    DRY_RUN = args.dry_run

    print("Exoskeleton V3 SPI IMU Inference Loop")
    print(f"Motor feedback source: {feedback_source}; configured driver map: {motor_map}")
    print("Driver labels follow firmware JOINT_MOTOR_ID defaults; they do not configure motors")
    print("=" * 50)
    print(f"CAN interface:   {channel}")
    print(f"Inference URL:   {PREDICT_URL}")
    print(f"Control rate:    {CONTROL_HZ} Hz")
    print(f"Window:          {WINDOW_LEN} x {FEATURE_COUNT} float32  "
          f"({WINDOW_DT_S * 1000:.0f} ms grid, {WINDOW_SPAN_S:.2f} s span)")
    print(f"Assist scale:    {ASSIST_SCALE:.2f}  ({ASSIST_SCALE * 100:.0f}% of model output)")
    print(f"Torque clamp:    +/- {TORQUE_CLAMP_NM:.2f} Nm")
    print(f"Hip motion gate: +/- {HIP_MOTION_GATE_DEG:.1f} deg "
          f"(hips zeroed when last {WINDOW_SPAN_S:.2f}s range "
          f"<= {2 * HIP_MOTION_GATE_DEG:.1f} deg; "
          f"cooldown {HIP_GATE_COOLDOWN_S:.1f}s)")
    print(f"Dry run:         {'ON (no torques sent)' if DRY_RUN else 'OFF'}")
    print(f"EMA alpha:       {'OFF' if EMA_ALPHA is None else f'{EMA_ALPHA:.2f}'}")
    _send_every_init = max(1, CONTROL_HZ // max(1, int(OUTPUT_HZ)))
    print(f"Output rate:     {OUTPUT_HZ} Hz "
          f"(send every {_send_every_init} cycles)")
    if FORCE_ZERO_ENCODERS:
        print(f"WARNING: FORCE_ZERO_ENCODERS = {_format_forced_joints()} -- "
              "those joints' encoder columns are zero regardless of motor "
              "position")
    if MIRROR_KNEE_TO_HIP:
        print("WARNING: MIRROR_KNEE_TO_HIP=True -- knee encoders copy "
              "their corresponding hip encoder")
    print("=" * 50)

    try:
        bus = can_common.create_bus(channel)
    except (OSError, can.CanError) as e:
        print(f"[FATAL] cannot open CAN interface '{channel}': {e}")
        print(f"  hint: sudo ip link set {channel} up type can bitrate 1000000")
        sys.exit(1)

    threading.Thread(target=can_listener, args=(bus,), daemon=True).start()
    threading.Thread(target=stale_watchdog, daemon=True).start()

    session_ts = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y%m%dT%H%M%SZ")
    log_path = LOG_DIR / f"inference_{session_ts}.csv"
    log_stop_evt = threading.Event()
    threading.Thread(
        target=session_logger, args=(log_path, log_stop_evt), daemon=True
    ).start()
    print(f"Session log:     {log_path}")

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
    finally:
        loop_stop_evt.set()
        log_stop_evt.set()
        try:
            can_system.send_estop(bus, can_common.NODE_PI)
        except can.CanError:
            pass
        if loop_thread is not None and loop_thread.is_alive():
            loop_thread.join(timeout=1.0)
        bus.shutdown()
        print("CAN bus closed")


if __name__ == "__main__":
    main()
