# McMaster Exoskeleton - Embedded Code

Embedded firmware and Raspberry Pi software for the McMaster Exoskeleton project. This repository contains STM32 applications, reusable hardware APIs, and standalone sensor and motor experiments, including projects for Nucleo-F446RE development boards.

## Getting Started

For the November SPI workshop, follow [NOVEMBER_WORKSHOP.md](NOVEMBER_WORKSHOP.md).

Use `src/` for application integration, `apis/` for reusable drivers and interfaces, `testing/` for standalone hardware experiments, and `workshops/` for learning exercises and member submissions.

Follow each project's README or local testing instructions for its tools, board, wiring, and validation steps. Hardware experiments from previous development remain available as references. Project folders may include:

- `docs/` -- API references and hardware testing instructions
- `scripts/` -- host-side helpers, such as Python tools for UART testing

## Project Structure

```text
exoskeleton-embedded/
├── apis/                        # Reusable sensor, motor, and CAN interfaces
├── src/
│   ├── stm32/                   # STM32 application projects
│   └── pi/                      # Raspberry Pi application code
├── testing/
│   ├── stm32/
│   │   ├── nucleo-f446re/        # NUCLEO-F446RE bench projects
│   │   │   ├── can-api/
│   │   │   ├── can-blink/
│   │   │   ├── imu-uart/
│   │   │   ├── motor-api-tests/
│   │   │   ├── nucleo-pi-can/
│   │   │   ├── sensor-testing/
│   │   │   ├── torque-controller/
│   │   │   └── uart-serial-com/
│   │   ├── discovery-f429/       # STM32F429I-DISC1 bench projects
│   │   │   ├── imu-test/
│   │   │   └── torque-control-disc/
│   │   └── joint-pcb-v2/         # V2 custom STM32F446RE joint PCB
│   │       └── can-test/
│   └── pi/                      # Raspberry Pi experiments
├── workshops/                   # Learning exercises and member submissions
│   └── spi-imu/
├── CONTRIBUTING.md
├── NOVEMBER_WORKSHOP.md
└── README.md
```

Group standalone tests by hardware target, then by test purpose. Use a PCB revision in the path when it identifies distinct hardware; keep software versions and competition-season snapshots in Git history/tags and documentation. Import the individual folder containing `.project` and `.ioc`, not an entire board group. Project names are retained, so projects sharing a CubeIDE name (such as `can-api` and `can-blink`, both named `exo-can`) need separate workspaces.

The [final 2025–26 / ACE2026 snapshot](https://github.com/macexo/exoskeleton-embedded/commit/7ee332ff0386be1071b6d9ce570493b73baf9912) records the previous season's code. That provenance does not prevent its motor API, torque console, and joint-controller implementation from serving as references this season.

The confirmed 2026–27 **V3 IMU/CAN baseline** lives in [sensing-actuation's `testing/stm32/pcb-imu-canbus`](https://github.com/macexo/exoskeleton-sensing-actuation/tree/main/testing/stm32/pcb-imu-canbus). Motor-control integration is being developed separately. The SPI workshop remains learning material under `workshops/`.

## Hardware Test Projects

### sensor-testing -- IMU Testing

Tests the **LSM6DS3TR-C** 6-axis IMU (accelerometer + gyroscope) over I2C. The Nucleo board reads sensor data continuously and exposes it over UART so you can query values from your computer using the Python script.

**Quick start:**

1. Follow the project instructions to import `testing/stm32/nucleo-f446re/sensor-testing` into STM32CubeIDE, build, and flash the firmware.
2. Run `python scripts/imu_uart_test.py` to send commands and read sensor data.
3. See [`docs/testing-instructions.md`](testing/stm32/nucleo-f446re/sensor-testing/docs/testing-instructions.md) for full setup and wiring details.
4. See [`docs/lsm6ds3tr-api.md`](testing/stm32/nucleo-f446re/sensor-testing/docs/lsm6ds3tr-api.md) for the driver API reference.

### motor-api-tests -- Motor Testing

Tests the **CubeMars AK70-9** brushless motor over CAN bus. The Nucleo board sends motor commands and receives motor feedback, with a UART telemetry interface for monitoring from your computer using the Python script.

**Quick start:**

1. Follow the project instructions to import `testing/stm32/nucleo-f446re/motor-api-tests` into STM32CubeIDE, build, and flash the firmware.
2. Run `python scripts/motor_test.py` to send commands and read motor telemetry.
3. See [`docs/testing-instructions.md`](testing/stm32/nucleo-f446re/motor-api-tests/docs/testing-instructions.md) for full setup, wiring, and CAN transceiver details.
4. See [`docs/ak70-9-api.md`](testing/stm32/nucleo-f446re/motor-api-tests/docs/ak70-9-api.md) for the motor CAN API reference.

## Tools and Hardware

Requirements depend on the project. For the hardware test projects described above:

- **STM32CubeIDE** -- for building and debugging firmware; **STM32CubeMX** for generating or updating board/peripheral configuration
- **Python 3** with `pyserial` (`pip install pyserial`) -- for the UART test scripts
- **Nucleo-F446RE**, the relevant sensor or motor hardware, and project-specific connections

## Contributing

Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on how to contribute to this project.
