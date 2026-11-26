# SPI IMU submission: Ranica Chawla

Keep this short and link it from your individual PR. This week covers LSM6DSO32 identity checking (`0x6C`) and register writing only.

## Implementation and questions

- `imu_check_identity()` — completed: checks that the connected IMU returns the expected identity value (0x6C), returns any read error immediately or IMU_ERROR_IDENTITY if the value is not as expected.

- `imu_write_register()` — completed: writes a value to a specified IMU register using SPI, returns IMU_ERROR_TRANSFER if the SPI transfer fails.

- Datasheet sections and reasons for read/write or error-handling choices: 5.1.2 (SPI protocol: Bit 7 is set to 1 for reading and cleared to 0 for writing), 9.11 (WHO_AM_I = 0x6C is used to verify the IMU's identity)

- Worked read:
    - Bit 7: 
        - Bit 7 = 1: Read from the register.
        - Bit 7 = 0: Write to the register.
        - Bits 6–0: The register address to read from or write to.
    - Dummy byte: A dummy byte is sent after the register address to allow time for the IMU to send data back.
        - `rx[1]`: Contains the actual data from the register that was read. 
        - `rx[0]`: Was ignored in the functions
    - One CS assertion: CS stays low for both bytes so the IMU reads the register address and data as one complete SPI transaction. If CS goes high in between, the IMU treats that as the end of the transaction.

- Blockers or questions to bring to the workshop:
    1. What should happen if the IMU fails the identity check or if the SPI transfer fails?
    2. Should we also be checking if the value that was written to the register is correct?

## Workshop results

**Not built or tested on hardware yet.** Replace this with actual results after testing.

- Board, exact IMU/breakout, wiring, CubeIDE version, and tested code commit:
- Build result:
- WHO_AM_I result and register-write readback:
- Error handling checked and remaining limitations:

Configuration and sample reading are deferred to a possible later assignment.
