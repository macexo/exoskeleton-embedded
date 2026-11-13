# SPI IMU submission: ali

Keep this short and link it from your individual PR. This week covers LSM6DSO32 identity checking (`0x6C`) and register writing only.

## Implementation and questions

- `imu_check_identity()` — completed 
- `imu_write_register()` — completed 


write register: Bitwise AND every value of reg with 01111111 (0x7F), to CLEAR msb 7. Bit 7 denotes the read/write flag of a uint8_t variable. 1:read, 0:write. We are writing on reg


HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET); -- HAL_GPIO_WritePin sends GPIO_PIN_RESET to imu IO, this sets CS to low (like an SR latch) 

## Workshop results

**Not built or tested on hardware yet.** Replace this with actual results after testing.

- Board, exact IMU/breakout, wiring, CubeIDE version, and tested code commit:
- Build result:
- WHO_AM_I result and register-write readback:
- Error handling checked and remaining limitations:

Configuration and sample reading are deferred to a possible later assignment.
