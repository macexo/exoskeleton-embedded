# V3 Pi applications

These scripts adapt the previous-year Pi applications to the current V3 CAN
payloads. Firmware is maintained separately; this work changes no STM32 code.

Python 3.10+ (Raspberry Pi OS Bookworm ships 3.11). Install the pinned
dependencies once; all three scripts import the shared decoders in
`apis/can/python`, which need `python-can`. Only the inference loop needs
`msgpack`. No sibling repository is required at runtime.

```bash
python3 -m pip install -r src/pi/requirements.txt
python3 src/pi/pi_can_buffer.py can1
python3 src/pi/pi_imu_torque.py can1
# Existing automatic controller, when its model/server and hardware are ready:
python3 src/pi/pi_inference_loop.py can1 0.2 --dry-run
```

Only run one torque-producing Pi script at a time.

## Manual console

`pi_imu_torque.py` is the manual motor + V3 IMU bench console, brought over
from sensing-actuation's tested `joint-controller-spi/Scripts/motor_console.py`.
It does not need inference or a model. Keep `console_telemetry.py` beside it.

```bash
python3 src/pi/pi_imu_torque.py can1                        # CSV: src/pi/logs/motor_log_<UTC time>.csv
python3 src/pi/pi_imu_torque.py can1 --out bench_001.csv    # relative to the current directory; never overwrites
python3 src/pi/pi_imu_torque.py can1 --motor-map 4:107      # label only the motor on the bench
```

| Command | Purpose |
| --- | --- |
| `t 4 0.1` | Request a small torque on joint 4 |
| `t_all 0.1` / `tall 0.1` | Request torque on all four nodes |
| `tstop` | Clear active requests and attempt zero on all joints |
| `estop` | Broadcast latched software ESTOP and clear active requests |
| `m1`–`m4` | Latest motor status and age |
| `l1`–`l4`, `q1`–`q4` | Latest IMU pair or recent paired samples |
| `status`, `help`, `quit` / `exit` | Summary, command reference, stop and exit |

Active requests refresh every 50 ms while the prompt waits for input. Firmware
expires them after 200 ms without a valid Pi command. Requested torque is limited
to ±7.155 Nm (nominal 1.431 Nm/A, firmware clamp ±5 A); begin far below that limit.
A sent request does not confirm motor execution. Quit, EOF and Ctrl+C clear
requests and attempt zero on all joints; unlike last year's prompt they do not
broadcast ESTOP, so use `estop` when the boards must latch.

The CSV records one row per event: command, refresh, stop and send failures,
raw IDs/payloads, IMU sequences/axes, motor position/speed/phase-current/
temperature/fault, and missing/stale ages once per second. Missing feedback is
not manufactured as zero. Requests are logged even when a joint has no motor
feedback or label.

The console opens its own SocketCAN socket with receive filters for the IMU,
ESTOP and selected motor-status IDs; live use requires Linux.

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

Each board's firmware selects its motor: `JOINT_MOTOR_ID` defaults by `NODE_ID`
in the joint-controller (`BOARD_JOINT_MOTOR_ID` in sensing's joint-controller-spi),
and a board built with 0 is IMU-only.

| Joint | Node | Firmware default driver |
| --- | --- | --- |
| Left hip | 1 | 104 |
| Right hip | 2 | 105 |
| Left knee | 3 | 106 |
| Right knee | 4 | 107 |

The Pi's `--motor-map` (default `1:104,2:105,3:106,4:107`, from
`can_v3.DEFAULT_MOTOR_MAP`) only labels feedback and, in direct mode, selects
which driver IDs are accepted. It does not configure or enable motors. Verify
physical assignments before using all-joint commands.

V3 gyro frames contain no encoder position. Position comes from motor status;
missing/stale feedback must not be treated as a real zero. Model shape, ordering,
resampling, signs, smoothing and controls are retained from the previous script.

No hardware testing is claimed. Confirm motor mapping, direction, raw feedback,
stop/expiry and IMU continuity on the unloaded bench before using automatic control.

## Bench check

Use an unloaded, secured motor with a second person at the physical power cutoff.
Start with zero torque, confirm IMU and motor status, then exercise small positive
and negative requests on one joint. Check targeting, `tstop`, latched ESTOP and
command expiry. Add motors one at a time, then test all-joint requests.

Capture the bus alongside the console (`candump -L can1 > bench_raw.log`) and
confirm IMU continuity/rate/loss while motors are commanded, using `status` or
`pi_can_buffer.py`, and agreement between raw frames and the CSV.

## Receive checks

If commands send but telemetry is missing, first check `candump -n 20 can1`
on the same interface. `pi_can_buffer.py` and `pi_inference_loop.py` use
`python-can`'s default receive-all filter. The manual console uses explicit
SocketCAN filters and keeps the fix from
[sensing-actuation PR #6](https://github.com/macexo/exoskeleton-sensing-actuation/pull/6):
keep `CAN_ERR_FLAG` out of data-frame masks, because Linux interprets that bit
as an error-only subscription.

With fresh IMUs but missing motors, check for forwarded status IDs `110`–`140`.
An absent driver-ID label does not block forwarded status; `--motor-map` assigns
labels and selects drivers in optional direct mode. Verify the actual firmware
assignment before changing it. A sent torque/current frame does not confirm a
motor reply or execution.
