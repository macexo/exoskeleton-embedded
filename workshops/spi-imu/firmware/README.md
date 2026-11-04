# SPI IMU CubeIDE project

Complete **NUCLEO-F446RE + LSM6DSO32** project. Import this folder into CubeIDE as **imu-spi**; use a fresh workspace if that name is already imported.

## Where you work

**Before the workshop:** edit only your own `submissions/<name>/imu_spi.c` and notes `README.md`. Follow [NOVEMBER_WORKSHOP.md](../../../NOVEMBER_WORKSHOP.md).

**At the workshop:** paste your implementation into `Core/Src/imu_spi.c`. The header is already finished. Copy fixes back to your submission before committing; member PRs contain only their own folder.

## What the other files do

These are provided support files for this assignment; use this map when the leads walk through the project.

| File/folder | Purpose |
| --- | --- |
| `Core/Inc/imu_spi.h` | Read-only types, function declarations, and expected behavior |
| `Core/Src/main.c`, `spi.c`, `gpio.c`, `usart.c` | Start the application and initialize SPI, pins, and UART |
| `Core/Src/workshop_demo.c` | Call your functions and print their test results |
| `Core/Src/lsm6dso32_*`, `Drivers/` | Existing sensor driver and ST hardware libraries |
| `Core/Startup/`, `*.ld` | Start the processor and define the flash/RAM layout |
| `.ioc`, `.mxproject`, `.project`, `.cproject` | Peripheral configuration and CubeIDE project/build settings |

The untouched starter builds and reports TODO. Completion requires identity/write status 0 and readback `0x44` with MATCH in the `workshop ...` UART lines. The provided motion stream is separate from those checks.

The board project comes from [exoskeleton-sensing-actuation](https://github.com/macexo/exoskeleton-sensing-actuation/tree/98fd9f8902231b3c574fc7759c328db029733137/testing/stm32/imu-spi), revision `98fd9f8`. Its generated setup is retained; the workshop adds `imu_spi.c/.h`, the demo, and its call in `main.c`. Wiring/settings are in the member guide.
