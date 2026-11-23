# V3 Pi applications

These scripts adapt the previous-year Pi applications to the current V3 CAN
payloads. Firmware is maintained separately; this work changes no STM32 code.

```bash
python3 -m pip install -r src/pi/requirements.txt
python3 src/pi/pi_can_buffer.py can1
python3 src/pi/pi_imu_torque.py can1
# Existing automatic controller, when its model/server and hardware are ready:
python3 src/pi/pi_inference_loop.py can1 0.2 --dry-run
```

The manual motor + IMU bench V1 belongs in sensing-actuation. It does not need
inference or a model. Only run one torque-producing Pi script at a time.

## Current CAN contract

IMU IDs also match the folder’s `can.png` and the working
`pcb-imu-canbus` SPI baseline. Matches sensing-actuation PR #4 at `05b60dbe30a4c72d468401b182a04210aab36d83`:

- CAN is 1 Mbit/s, configured on the Pi before starting the script.
- IMU accel IDs `190/1A0/1B0/1C0`, gyro `210/220/230/240`: standard DLC 8,
  little-endian `<Hhhh` sequence + raw XYZ. Pair by node and sequence.
- Raw accel ×0.000488 gives g; gyro ×0.070 gives degrees/s. The existing model
  input uses m/s² and rad/s. Nominal IMU rate is 417 Hz.
- Default motor status IDs `110/120/130/140`: standard DLC 8, `<hhhbB`,
  position /10 degrees, speed ×10 ERPM, phase current /100 A, temperature, fault.
  The existing `can_motor.parse_motor_status` decoder is reused.
- Optional direct feedback is extended `(0x29 << 8) | driver`, big-endian fields.
  The earlier PR revision's `0x09` has been replaced by `0x29` in the latest PR.
- Torque IDs `081`–`084`: signed int16 little-endian Nm ×1000. ESTOP is `000`.
  PR command expiry is 200 ms; its local motor refresh is 50 ms.
- Only node 4 → motor 107 is assigned by default. Other motors require explicit
  firmware assignment and physical verification; Pi attribution does not enable them.

V3 gyro frames contain no encoder position. Position comes from motor status;
missing/stale feedback must not be treated as a real zero. Model shape, ordering,
resampling, signs, smoothing and controls are retained from the previous script.

No hardware testing is claimed. Confirm motor mapping, direction, raw feedback,
stop/expiry and IMU continuity on the unloaded bench before using automatic control.
