"""
console_telemetry.py

V3 telemetry helpers for the pi_imu_torque.py manual console.

Brings over only the parts of sensing-actuation's tested bench console that the
console needs, so no sibling repository is required at runtime:
  - IMU sequence pairing, scaling and loss accounting from the unchanged
    four-board checker (joint-controller-spi/Scripts/imu_canbus.py).
  - The bounded per-joint IMU queue, exact IMU ID matching and motor-status
    decoding from joint-controller-spi/Scripts/console_telemetry.py.
Motor-status scaling mirrors apis/can/python/can_motor.py:parse_motor_status.

Python standard library only.
"""

from collections import deque
from dataclasses import dataclass
import struct

# V3 rate and scales; must match the joint-controller firmware configuration.
SAMPLE_RATE_HZ = 417
ACCEL_G_PER_COUNT = 0.000488  # +/-16 g range.
GYRO_DPS_PER_COUNT = 0.070  # +/-2000 dps range.

# CAN ID bits [10:7] message type, [6:4] source node, [3:0] destination (0 = Pi).
CAN_MSG_IMU_ACCEL = 0x3
CAN_MSG_IMU_GYRO = 0x4
CAN_NODE_PI = 0x0
NODE_NAMES = {1: "left hip", 2: "right hip", 3: "left knee", 4: "right knee"}

# Incomplete pairs older than PAIR_EXPIRY_S are discarded. A sequence gap above
# elapsed * rate * GAP_SLACK + GAP_MARGIN is a jump (reset, duplicate NODE_ID).
PAIR_EXPIRY_S = 0.5
GAP_SLACK = 1.5
GAP_MARGIN = 10

# Sequence + raw XYZ, little endian.
PAYLOAD_FORMAT = "<Hhhh"


def build_id(msg_type, src, dest):
    """Build an 11-bit CAN ID: type in bits [10:7], source [6:4], destination [3:0]."""

    return (msg_type << 7) | (src << 4) | dest


def imu_ids(node):
    """Return the (accel, gyro) CAN IDs of a board built with NODE_ID = node."""

    return (build_id(CAN_MSG_IMU_ACCEL, node, CAN_NODE_PI), build_id(CAN_MSG_IMU_GYRO, node, CAN_NODE_PI))


def scale(accel, gyro):
    """Convert raw counts to (ax, ay, az) in g followed by (gx, gy, gz) in dps."""

    return (tuple(a * ACCEL_G_PER_COUNT for a in accel) + tuple(g * GYRO_DPS_PER_COUNT for g in gyro))


class Node:
    """Pairs one board's accel and gyro frames and counts its lost samples."""

    def __init__(self, node_id):
        self.node_id = node_id
        self.name = NODE_NAMES[node_id]
        self.pending = {}  # sequence -> (first receive time, {msg_type: axes})
        self.last_sequence = None
        self.first_pair_time = None
        self.last_pair_time = None
        self.last_rx = None  # Receive time of the newest frame, paired or not.
        self.pairs = 0
        self.missing = 0
        self.jumps = 0
        self.latest = None  # Scaled values of the newest pair.

    def add_frame(self, msg_type, sequence, axes, rx_time):
        """Store one frame. Returns (accel, gyro) raw counts once its pair completes, else None."""

        self.last_rx = rx_time

        if sequence not in self.pending:
            # Expire incomplete pairs so resets, loss and wrap cannot mix old data.
            self.pending = {seq: entry for seq, entry in self.pending.items() if rx_time - entry[0] < PAIR_EXPIRY_S}
            self.pending[sequence] = (rx_time, {})

        halves = self.pending[sequence][1]
        halves[msg_type] = axes

        if len(halves) < 2:
            return None

        del self.pending[sequence]

        self._count_pair(sequence, rx_time)
        accel = halves[CAN_MSG_IMU_ACCEL]
        gyro = halves[CAN_MSG_IMU_GYRO]
        self.latest = scale(accel, gyro)

        return accel, gyro

    def _count_pair(self, sequence, rx_time):
        """Update the loss counters for a completed pair."""

        if self.last_sequence is not None:
            gap = (sequence - self.last_sequence) & 0xFFFF

            # The board advances its sequence for every sample, sent or not, so a
            # gap of n means n - 1 lost pairs, as long as enough time passed for
            # them. A bigger gap, a repeat, or a step backwards is a jump.
            elapsed = max(rx_time - self.last_pair_time, 0.0)
            plausible = int(elapsed * SAMPLE_RATE_HZ * GAP_SLACK) + GAP_MARGIN

            if 0 < gap <= plausible:
                self.missing += gap - 1
            else:
                self.jumps += 1
        else:
            self.first_pair_time = rx_time

        self.last_sequence = sequence
        self.last_pair_time = rx_time
        self.pairs += 1

    def mean_rate(self):
        """Average received pair rate over the span this node was seen, in Hz."""

        if (self.pairs < 2 or self.last_pair_time <= self.first_pair_time):
            return 0.0

        return (self.pairs - 1) / (self.last_pair_time - self.first_pair_time)


@dataclass(frozen=True)
class IMUSample:
    sequence: int
    received_at: float
    raw_accel: tuple
    raw_gyro: tuple
    scaled: tuple

    def fields(self):
        fields = {"imu_sequence": self.sequence}
        fields.update(zip(("ax_raw", "ay_raw", "az_raw"), self.raw_accel))
        fields.update(zip(("gx_raw", "gy_raw", "gz_raw"), self.raw_gyro))
        fields.update(zip(("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"), self.scaled))

        return fields


class IMUState:
    """Bounded per-joint V3 queue; no downsampling or old gyro-position layout."""

    def __init__(self, joint, queue_size=187):
        self.node = Node(joint)
        self.queue = deque(maxlen=queue_size)

    def add_frame(self, kind, sequence, axes, received_at):
        # Expire even if this sequence already has one half; Node only prunes
        # when inserting a new sequence. Never join old/new halves.
        self.node.pending = {seq: entry for seq, entry in self.node.pending.items() if received_at - entry[0] < PAIR_EXPIRY_S}
        pair = self.node.add_frame(kind, sequence, axes, received_at)

        if pair is None:
            return None

        accel, gyro = pair
        sample = IMUSample(sequence, received_at, accel, gyro, scale(accel, gyro))
        self.queue.append(sample)

        return sample


def decode_imu_id(raw_id):
    """Match exact standard Pi-directed V3 IDs, rejecting other destinations/sources."""

    for joint in NODE_NAMES:
        accel, gyro = imu_ids(joint)

        if raw_id == accel:
            return joint, CAN_MSG_IMU_ACCEL
        if raw_id == gyro:
            return joint, CAN_MSG_IMU_GYRO

    return None


def parse_motor_status(payload):
    """can_motor.parse_motor_status semantics, with explicit eight-byte validation."""

    if len(payload) != 8:
        raise ValueError("motor status payload must contain exactly eight bytes")

    position, speed, phase_current, temperature, fault = struct.unpack("<hhhbB", payload)

    return position / 10.0, speed * 10.0, phase_current / 100.0, temperature, fault
