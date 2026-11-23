"""Pi decoders for pcb-imu-canbus V3 telemetry; no firmware or bus initialization.

V3 IMU frames are sequence + raw XYZ, NOT the legacy can_imu physical-unit
format. CAN receive timestamps are host timestamps, not MCU sample timestamps.
Motor feedback interpretation and the default physical map need bench validation.
"""

from dataclasses import dataclass, field
import math
import struct
import time

from . import can_motor

SAMPLE_RATE_HZ = 417
ACCEL_G_PER_COUNT = 0.000488
STANDARD_GRAVITY = 9.80665
GYRO_DPS_PER_COUNT = 0.070
PAIR_EXPIRY_S = 0.5
MAX_PENDING_PAIRS = 32
MOTOR_STALE_S = 0.5
# PR4 only assigns right knee by default. Other nodes are IMU-only until the
# firmware build explicitly sets JOINT_MOTOR_ID; never invent physical routing.
DEFAULT_MOTOR_MAP = {4: 107}
MOTOR_STATUS_IDS = {0x100 | (node << 4): node for node in range(1, 5)}
FEEDBACK_SOURCES = ("forwarded", "direct")
IMU_IDS = {((kind << 7) | (node << 4)): (node, kind)
           for node in range(1, 5) for kind in (3, 4)}


def _data_frame(msg, extended, length):
    return (bool(msg.is_extended_id) == extended
            and not getattr(msg, "is_remote_frame", False)
            and not getattr(msg, "is_error_frame", False)
            and not getattr(msg, "is_fd", False)
            and msg.dlc == length and len(msg.data) == length)


def receive_timestamp(msg):
    """Return a usable host epoch timestamp, falling back for unset timestamps."""
    stamp = float(msg.timestamp)
    return stamp if math.isfinite(stamp) and stamp > 0 else time.time()


def parse_motor_map(text):
    """Parse explicitly verified joint:driver entries (e.g. 4:107)."""
    result = {}
    for entry in text.split(","):
        joint, driver = (int(value.strip()) for value in entry.split(":"))
        if joint in result:
            raise ValueError("duplicate joint in motor map")
        result[joint] = driver
    validate_motor_map(result)
    return result


def validate_motor_map(mapping):
    """Validate attribution only; the STM32 config controls actual motor routing."""
    if (not mapping or not set(mapping).issubset({1, 2, 3, 4})
            or any(type(joint) is not int for joint in mapping)
            or any(type(driver) is not int or not 1 <= driver <= 255
                   for driver in mapping.values())
            or len(set(mapping.values())) != len(mapping)):
        raise ValueError("motor map requires joint IDs 1..4 and unique driver IDs 1..255")


@dataclass(frozen=True)
class ImuPair:
    node: int
    sequence: int
    timestamp: float
    raw_accel: tuple
    raw_gyro: tuple
    discontinuity: bool = False

    @property
    def accel_mps2(self):
        return tuple(value * ACCEL_G_PER_COUNT * STANDARD_GRAVITY for value in self.raw_accel)

    @property
    def gyro_dps(self):
        return tuple(value * GYRO_DPS_PER_COUNT for value in self.raw_gyro)

    @property
    def gyro_rads(self):
        return tuple(math.radians(value) for value in self.gyro_dps)


@dataclass
class NodeState:
    pending: dict = field(default_factory=dict)
    last_sequence: object = None
    last_pair_at: object = None
    last_timestamp: object = None
    pairs: int = 0
    missing: int = 0
    discontinuities: int = 0
    duplicates: int = 0
    late: int = 0
    incomplete: int = 0


class ImuReceiver:
    """Bounded sequence pairing, called by a single CAN listener.

    Pairing is per node and sequence, accepts either half first, and expires
    incomplete pairs using a monotonic clock. Late/duplicate completed samples
    cannot enter a consumer's time-ordered buffer. A reset/large jump marks a
    discontinuity so consumers clear their history instead of joining epochs.
    """

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.nodes = {node: NodeState() for node in range(1, 5)}

    def process(self, msg):
        """Return a complete ImuPair or None; malformed/other frames are ignored."""
        if not _data_frame(msg, False, 8) or msg.arbitration_id not in IMU_IDS:
            return None
        node, kind = IMU_IDS[msg.arbitration_id]
        sequence, x, y, z = struct.unpack("<Hhhh", msg.data)
        state = self.nodes[node]
        now = self.clock()
        for seq, (started, _) in list(state.pending.items()):
            if now - started >= PAIR_EXPIRY_S:
                del state.pending[seq]
                state.incomplete += 1
        if sequence not in state.pending:
            if len(state.pending) >= MAX_PENDING_PAIRS:
                del state.pending[next(iter(state.pending))]
                state.incomplete += 1
            state.pending[sequence] = (now, {})
        halves = state.pending[sequence][1]
        if kind in halves:
            state.duplicates += 1
            return None
        halves[kind] = ((x, y, z), receive_timestamp(msg))
        if len(halves) != 2:
            return None
        del state.pending[sequence]
        timestamp = max(halves[3][1], halves[4][1])
        discontinuity = False
        if state.last_sequence is not None:
            gap = (sequence - state.last_sequence) & 0xFFFF
            elapsed = max(0.0, now - state.last_pair_at)
            if gap == 0:
                state.duplicates += 1
                return None
            if gap >= 0x8000 and sequence != 0 and elapsed < PAIR_EXPIRY_S:
                state.late += 1
                return None
            plausible_gap = int(elapsed * SAMPLE_RATE_HZ * 1.5) + 10
            discontinuity = gap >= 0x8000 or gap > plausible_gap or elapsed >= PAIR_EXPIRY_S
            if discontinuity:
                state.discontinuities += 1
                state.pending.clear()
            else:
                state.missing += gap - 1
        # A wall-clock rollback/backlog must not produce nonmonotonic buffers.
        if state.last_timestamp is not None and timestamp <= state.last_timestamp:
            state.late += 1
            return None
        state.last_sequence = sequence
        state.last_pair_at = now
        state.last_timestamp = timestamp
        state.pairs += 1
        return ImuPair(node, sequence, timestamp, halves[3][0], halves[4][0], discontinuity)


def decode_motor_feedback(msg, motor_map, source="forwarded"):
    """Decode only the selected PR4 source; never let raw traffic override it.

    Default forwarded MOTOR_STATUS reuses the legacy decoder after exact frame
    validation. Direct extended function 0x29 is opt-in and requires a configured
    physical driver map. Phase current is received feedback, not requested Iq.
    """
    if source == "forwarded":
        if not _data_frame(msg, False, 8) or msg.arbitration_id not in MOTOR_STATUS_IDS:
            return None
        node = MOTOR_STATUS_IDS[msg.arbitration_id]
        position, speed, current, temperature, fault = can_motor.parse_motor_status(msg)
        driver = motor_map.get(node)  # Unknown physical routing stays unknown.
    elif source == "direct":
        if (not _data_frame(msg, True, 8) or not 0 <= msg.arbitration_id <= 0x1FFFFFFF
                or msg.arbitration_id >> 8 != 0x29):
            return None
        driver = msg.arbitration_id & 0xFF
        node = next((node for node, value in motor_map.items() if value == driver), None)
        if node is None:
            return None
        position, speed, current, temperature, fault = struct.unpack(">hhhbB", msg.data)
        position /= 10.0
        speed *= 10
        current /= 100.0
    else:
        raise ValueError("feedback source must be forwarded or direct")
    return node, {
        "timestamp": receive_timestamp(msg) * 1000.0,
        "received_at": time.monotonic(),
        "source": source,
        "driver_id": driver,
        "position": position,
        "speed": speed,
        "phase_current": current,
        "temperature": temperature,
        "error": fault,
    }


def fresh_motor(motor, now=None):
    """True only for real, recent selected motor feedback; never synthesize missing zeros."""
    now = time.monotonic() if now is None else now
    return (motor is not None and 0 <= now - motor["received_at"] < MOTOR_STALE_S
            and 0 <= time.time() * 1000.0 - motor["timestamp"] < MOTOR_STALE_S * 1000.0)
