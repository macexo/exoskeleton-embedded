# SPI IMU submission: Zacharaya Araya

Keep this short and link it from your individual PR. This week covers LSM6DSO32 identity checking (`0x6C`) and register writing only.

## Implementation and questions

- `imu_check_identity()` — completed: Reads the register and checks if the returned identity is the same as 0x6C
- `imu_write_register()` — completed:  Sends the register address with bit 7 cleared, followed by the value to write.
- Datasheet sections and reasons for read/write or error-handling choices: Used the supplied LSM6DSO32 datasheet sections and starter code
- Worked read: why bit 7, a dummy byte, `rx[1]`, and one CS assertion? Bit 7 is set to indicate a read operation, A dummy byte provides clock cycles for the IMU to return data. rx[1] has the recieved register value since the first byte contains the address transfer. CS remains asserted for the full SPI data exchange
- Blockers or questions to bring to the workshop: None

## Workshop results

**Not built or tested on hardware yet.** Replace this with actual results after testing.

- Board, exact IMU/breakout, wiring, CubeIDE version, and tested code commit:
- Build result:
- WHO_AM_I result and register-write readback:
- Error handling checked and remaining limitations:

Configuration and sample reading are deferred to a possible later assignment.
