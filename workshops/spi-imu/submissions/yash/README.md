# SPI IMU submission: Yash Gupta

Keep this short and link it from your individual PR. This week covers LSM6DSO32 identity checking (`0x6C`) and register writing only.

## Implementation and questions

- `imu_check_identity()` — completed / remaining: completed
- `imu_write_register()` — completed / remaining: completed
- Datasheet sections and reasons for read/write or error-handling choices:
- Worked read: why bit 7, a dummy byte, `rx[1]`, and one CS assertion?
  - Bit 7 is set because that's the read/write bit. Set = read, cleared = write.
  - SPI is full duplex, i.e. it sends and receives at the same time. First byte in tx is the register to be read, while second is dummy since we need to read the second byte in rx but don't have any data to send, and to receive we must send simultaneously.
  - rx[1] holds the value because rx[0] is received at the same time tx[0] is sent. Until tx[0] has been sent, the IMU doesn't know what data needs to be read.
  - CS stays low since chip select is active low for SPI

- Blockers or questions to bring to the workshop:

## Workshop results

**Not built or tested on hardware yet.** Replace this with actual results after testing.

- Board, exact IMU/breakout, wiring, CubeIDE version, and tested code commit:
- Build result:
- WHO_AM_I result and register-write readback:
- Error handling checked and remaining limitations:

Configuration and sample reading are deferred to a possible later assignment.
