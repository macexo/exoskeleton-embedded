# Joint controller

Build `joint-controller` in STM32CubeIDE for the STM32F446 Sensing PCB.
Set `NODE_ID` for the physical joint in the build configuration. The default
is right knee (node 4). The motor ID follows the confirmed Sensing map.

| Joint | Node ID | Default motor ID |
| --- | --- | --- |
| Left hip | 1 | 104 |
| Right hip | 2 | 105 |
| Left knee | 3 | 106 |
| Right knee | 4 | 107 |

Override `JOINT_MOTOR_ID` for a different motor or set it to 0 for IMU-only.
PC13 flashes the node number at startup and toggles on successful
IMU streaming. A solid LED indicates a read, send, or stale-sample condition.

## CAN and SPI

The board uses CAN1 at 1 Mbit/s and reads the LSM6DSO32 on SPI3 at 417 Hz.
SPI3 uses PC10/PB4/PB5 for clock/MISO/MOSI, PB6 for chip select, and PB1
for data ready. Configure the Pi SocketCAN interface used by the inference
loop for 1 Mbit/s and check its physical CANH/CANL transceiver connection.

| CAN traffic | Encoding |
| --- | --- |
| IMU to Pi | Standard IDs `0x180 + (node << 4)` and `0x200 + (node << 4)`. Both carry a little-endian uint16 sequence and three signed int16 raw axes. |
| Motor status to Pi | Standard ID `0x100 + (node << 4)`. Position (0.1°), speed (10 ERPM), current (0.01 A), temperature (°C), and fault code. V3 servo feedback uses extended ID `(0x29 << 8) | motor_id` (motor 107: `0x296B`). Fresh feedback is forwarded at up to 100 Hz, with fault changes eligible immediately. |
| Current setpoint to Pi | Standard ID `0x300 + (node << 4)` (`0x310`, `0x320`, `0x330`, `0x340`). Four bytes: signed little-endian int32 milliamps after torque conversion and the ±5 A clamp. Target 100 Hz, including zero while idle. |
| Pi torque to joint | Standard ID `0x080 + node`, two little-endian bytes of signed millinewton-metres. The STM32 clamps current to ±5 A and requests zero after 200 ms without a fresh command. |

The new setpoint frame reports `clamp(torque_nm / (0.159 * 9), -5, 5)`
quantized to milliamps using the same truncation as the motor command. It is
requested current, not measured phase current, DC supply current, or confirmation
that the driver received/applied the command. Zero commands, command timeout,
and e-stop make zero telemetry eligible immediately. Global reset prioritizes
the motor stop and suppresses diagnostic sends so telemetry cannot delay reset.

Motor status forwards fresh feedback only: 100 Hz requires the driver to supply
feedback at least that fast and available CAN capacity. Fault changes can bypass
the 10 ms interval. Setpoint telemetry retries when a transmit mailbox is full.
The existing motor-command refresh rate and all control limits remain unchanged.

The Pi decoder stores the current setpoint separately from reported motor current.
Its CSV adds `<joint>_current_setpoint_a` (e.g. `left_hip_current_setpoint_a`),
using the existing joint-name prefixes. CSV rows remain 5 Hz snapshots of the
latest received values, not a full 100 Hz CAN recording; missing initial values
are blank, and the last received value is retained across a telemetry dropout.
Capture CAN frames separately when every sample is needed. Raw model predictions
and Pi torque outputs remain local CSV fields; the new frame carries current only.

## Model-driven torque

Run `src/pi/pi_inference_loop.py` with the existing inference service.
The Pi recognizes SPI boards from their eight-byte accelerometer frames,
pairs matching sequence numbers per joint, converts acceleration to m/s²
and angular velocity to rad/s, and gets encoder position from motor status.
The legacy six-byte accelerometer layout is also supported. Normal SPI
encoder input requires motor status within 200 ms. The existing explicit
encoder overrides and degraded-mode handling for offline joints still apply.

The model uses the existing 187 × 28 window and the Pi sends its per-joint
torque outputs through the existing scaling, limits, sign, and hip motion
gate. Set the output rate so commands arrive comfortably within the STM32's
200 ms timeout. The default of about 33 Hz meets that timing requirement.

Firmware and board behavior derive from Sensing & Actuation's
`testing/stm32/joint-controller-spi` at `641931d`. Deploy this firmware
with the updated inference receiver. Other consumers of the old scaled CAN
layout need a compatible SPI decoder.

## Validation

Run `python src/stm32/joint-controller/Scripts/host_tests/test_motor_telemetry.py` with GCC
on PATH for host checks of telemetry timing, byte encoding, conversion/clamping,
timeout/e-stop zero, mailbox retries, and reset suppression. This uses mocked
HAL/CAN calls and does not validate physical bus delivery.

Run `python src/pi/tests/test_joint_inference.py` with the Pi dependencies
(`python-can`, `msgpack`) installed. The replay uses virtual CAN and a
stubbed model prediction. On the unloaded bench, confirm joint and motor
addresses, physical axis directions and scale, encoder zero and sign, torque
direction, timeout, and e-stop. Capture multi-node IMU timing, drop rate,
and motor fault/status frames before the suit test.
