# SPI IMU workshop

**Members: follow [NOVEMBER_WORKSHOP.md](../../NOVEMBER_WORKSHOP.md).** This is an optional directory map.

| Location | Purpose |
| --- | --- |
| `submissions/_template/` | Two-file starter to copy and rename |
| `submissions/<name>/` | Each member's implementation and notes |
| `firmware/` | Complete CubeIDE project, imported as **imu-spi** |
| `firmware/Core/Inc/imu_spi.h` | Read-only student interface |
| `firmware/Core/Src/imu_spi.c` | Selected student implementation; initially the matching starter |
| `firmware/Core/Src/workshop_demo.c` | Explicit identity/write/readback check of student code |

Target: NUCLEO-F446RE + LSM6DSO32, WHO_AM_I `0x6C`, SPI3. The board project is copied from the sensing-actuation reference at `98fd9f8`, with the student files and demo hook added. Before the workshop, edit only your submission folder.

This week covers identity checking and register writing. The provided vendor driver handles configuration/streaming as context; those functions may be future exercises. Previous-season LSM6DS3TR-C/I²C code remains historical reference.
