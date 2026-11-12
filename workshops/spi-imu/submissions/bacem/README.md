# SPI IMU submission: Bacem Karray

Keep this short and link it from your individual PR. This week covers LSM6DSO32 identity checking (`0x6C`) and register writing only.

## Implementation and questions

- `imu_check_identity()` — completed: checks 0x0F register for 0x6C.
- `imu_write_register()` — completed: sends address + value.
- Datasheet sections and reasons for read/write or error-handling choices: 5.1.2 to understand the SPI interface and 9.11 to look at the WHO_AM_I register.
- Worked read why bit 7, a dummy byte, `rx[1]`, and one CS assertion? Bit 7 selects the read operation. The dummy byte is used as a clock to feed data into rx[1]; The CS assertion indicates the entire SPI exchange.
- Blockers or questions to bring to the workshop: N/A

## Workshop results

**Not built or tested on hardware yet.** Replace this with actual results after testing.

- Board, exact IMU/breakout, wiring, CubeIDE version, and tested code commit: not recorded yet.
- Build result: not built yet.
- WHO_AM_I result and register-write readback: not tested; expected identity `0x6C`.
- Error handling checked and remaining limitations: argument, transfer, and identity errors implemented; testing pending.

Configuration and sample reading are deferred to a possible later assignment.
