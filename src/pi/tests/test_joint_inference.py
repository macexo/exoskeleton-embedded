"""Replay SPI and legacy wire payloads through the Pi model input and output path."""
import math
import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import can

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pi_inference_loop as loop


class JointInferenceTest(unittest.TestCase):
    def setUp(self):
        loop.loop_stop_evt.clear()
        self.addCleanup(loop.loop_stop_evt.clear)
        for node in loop.NODES:
            loop.joints[node] = loop.make_joint_state()
            loop.last_seen[node] = None

    def receive(self, kind, node, data, timestamp=1.0):
        loop.process_msg(can.Message(arbitration_id=(kind << 7) | (node << 4),
                                     is_extended_id=False, data=data, timestamp=timestamp))

    def test_legacy_and_lost_spi_half(self):
        self.receive(3, 1, struct.pack("<hhh", 100, -200, 300))
        self.receive(4, 1, struct.pack("<hhhh", 0, 100, 0, 900))
        _, accel, gyro, angle = loop.joints[1]["samples"][-1]
        self.assertEqual(accel, (1, -2, 3))
        self.assertAlmostEqual(gyro[1], math.radians(1))
        self.assertAlmostEqual(angle, math.pi / 2)
        self.receive(3, 2, struct.pack("<Hhhh", 65535, 1, 2, 3))
        self.receive(4, 2, struct.pack("<Hhhh", 0, 4, 5, 6))
        self.assertFalse(loop.joints[2]["samples"])
        self.assertIsNone(loop.last_seen[2])

    def test_spi_waits_for_first_motor_status(self):
        for kind in (3, 4):
            self.receive(kind, 4, struct.pack("<Hhhh", 17, 1, 2, 3))
        self.assertFalse(loop.joints[4]["samples"])

    def test_stale_motor_status_drops_spi_sample(self):
        with patch.object(loop.time, "monotonic", return_value=1.0):
            self.receive(2, 4, struct.pack("<hhh bB", 300, 2, 0, 40, 0))
        with patch.object(loop.time, "monotonic", return_value=1.25):
            for kind in (3, 4):
                self.receive(kind, 4, struct.pack("<Hhhh", 17, 1, 2, 3))
        self.assertFalse(loop.joints[4]["samples"])
        self.assertIsNone(loop.last_seen[4])

    def test_spi_window_to_torque(self):
        # The loop requires timestamps after start. Use a fixed replay clock.
        start = 100.0
        for node in loop.NODES:
            self.receive(2, node, struct.pack("<hhh bB", 300, 2, -50, 40, 0))
            for i in range(loop.WINDOW_SIZE + 3):
                stamp = start + i * 0.01
                seq = (65530 + i) & 0xffff
                self.receive(3, node, struct.pack("<Hhhh", seq, 2048, 0, -2048), stamp)
                self.receive(4, node, struct.pack("<Hhhh", seq, 0, -100, 0), stamp)

        captured = []
        def predict(window):
            captured.append(window)
            loop.loop_stop_evt.set()
            return [1.0] * 4, 1.0

        with can.Bus(interface="virtual", receive_own_messages=True) as bus:
            with patch.object(loop.time, "time", return_value=start), patch.object(
                    loop, "predict_msgpack", side_effect=predict):
                loop.control_loop(bus)
            frames = [bus.recv(0.1) for _ in range(4)]
        self.assertEqual(len(captured), 1)
        window = captured[0]
        self.assertEqual(len(window), 187 * 28)
        for node, frame in zip(loop.OUTPUT_NODE_ORDER, frames):
            offset = loop.JOINT_TO_IMU_SLOT[node]
            self.assertAlmostEqual(window[offset], 2048 * 0.000488 * 9.80665)
            self.assertAlmostEqual(window[offset + 4], math.radians(-7))
            self.assertAlmostEqual(window[loop.JOINT_TO_ENCODER_SLOT[node]], math.radians(30))
            self.assertEqual(frame.arbitration_id, 0x80 + node)
            self.assertEqual(frame.dlc, 2)
            expected = 0 if node in loop.HIP_GATED_NODES else loop.TORQUE_SIGN[node] * loop.ASSIST_SCALE
            self.assertEqual(struct.unpack("<h", frame.data)[0], int(expected * 1000))


if __name__ == "__main__":
    unittest.main()
