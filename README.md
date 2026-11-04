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
│   ├── stm32/                   # Standalone STM32 hardware experiments
│   │   ├── sensor-testing/      # LSM6DS3TR-C IMU testing over I2C
│   │   │   ├── docs/
│   │   │   │   ├── lsm6ds3tr-api.md
│   │   │   │   └── testing-instructions.md
│   │   │   ├── scripts/
│   │   │   │   └── imu_uart_test.py
│   │   │   └── Core/            # STM32CubeIDE firmware source
│   │   └── motor-api-tests/     # AK70-9 motor testing over CAN
│   │       ├── docs/
│   │       │   ├── ak70-9-api.md
│   │       │   └── testing-instructions.md
│   │       ├── scripts/
│   │       │   └── motor_test.py
│   │       └── Core/            # STM32CubeIDE firmware source
│   ├── discovery/               # Discovery-board experiments
│   └── pi/                      # Raspberry Pi experiments
├── workshops/                   # Assignments, starters, and member submissions
├── CONTRIBUTING.md              # Contribution guidelines
├── NOVEMBER_WORKSHOP.md         # November SPI workshop member guide
└── README.md
```

## Hardware Test Projects

### sensor-testing -- IMU Testing

Tests the **LSM6DS3TR-C** 6-axis IMU (accelerometer + gyroscope) over I2C. The Nucleo board reads sensor data continuously and exposes it over UART so you can query values from your computer using the Python script.

**Quick start:**

1. Follow the project instructions to import `testing/stm32/sensor-testing` into STM32CubeIDE, build, and flash the firmware.
2. Run `python scripts/imu_uart_test.py` to send commands and read sensor data.
3. See [`docs/testing-instructions.md`](testing/stm32/sensor-testing/docs/testing-instructions.md) for full setup and wiring details.
4. See [`docs/lsm6ds3tr-api.md`](testing/stm32/sensor-testing/docs/lsm6ds3tr-api.md) for the driver API reference.

### motor-api-tests -- Motor Testing

Tests the **CubeMars AK70-9** brushless motor over CAN bus. The Nucleo board sends motor commands and receives motor feedback, with a UART telemetry interface for monitoring from your computer using the Python script.

**Quick start:**

1. Follow the project instructions to import `testing/stm32/motor-api-tests` into STM32CubeIDE, build, and flash the firmware.
2. Run `python scripts/motor_test.py` to send commands and read motor telemetry.
3. See [`docs/testing-instructions.md`](testing/stm32/motor-api-tests/docs/testing-instructions.md) for full setup, wiring, and CAN transceiver details.
4. See [`docs/ak70-9-api.md`](testing/stm32/motor-api-tests/docs/ak70-9-api.md) for the motor CAN API reference.

## Tools and Hardware

Requirements depend on the project. For the hardware test projects described above:

- **STM32CubeIDE** -- for building and debugging firmware; **STM32CubeMX** for generating or updating board/peripheral configuration
- **Python 3** with `pyserial` (`pip install pyserial`) -- for the UART test scripts
- **Nucleo-F446RE**, the relevant sensor or motor hardware, and project-specific connections

## Contributing

Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on how to contribute to this project.
