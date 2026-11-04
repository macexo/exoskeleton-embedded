#ifndef WORKSHOP_IMU_SPI_H
#define WORKSHOP_IMU_SPI_H

#include "stm32f4xx_hal.h"

typedef enum {
    IMU_OK = 0,
    IMU_ERROR_ARGUMENT,
    IMU_ERROR_TRANSFER,
    IMU_ERROR_IDENTITY,
    IMU_NOT_IMPLEMENTED
} imu_status_t;

/* Target: LSM6DSO32 on the NUCLEO-F446RE reference board project.
 * The workshop project supplies initialized, 8-bit, full-duplex, MSB-first
 * master SPI with software NSS and a CS output initially high.
 * cs_pin is a HAL GPIO mask (e.g. GPIO_PIN_6), not a numeric pin index. */
typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} imu_device_t;

/* Return IMU_ERROR_ARGUMENT for missing device/hardware/output pointers or CS pin.
 * Register addresses must be 0x00..0x7f. Always release CS after a transfer.
 * Return IMU_ERROR_TRANSFER on HAL failure; preserve output values on errors. */

/* Supplied worked example. */
imu_status_t imu_read_register(const imu_device_t *imu, uint8_t reg, uint8_t *value);

/* This week: implement identity first, then register writing. */
imu_status_t imu_check_identity(const imu_device_t *imu);
imu_status_t imu_write_register(const imu_device_t *imu, uint8_t reg, uint8_t value);

#endif
