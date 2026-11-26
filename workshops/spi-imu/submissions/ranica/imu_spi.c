#include "imu_spi.h"

static int valid_device(const imu_device_t *imu)
{
    return imu != NULL && imu->spi != NULL && imu->cs_port != NULL && imu->cs_pin != 0;
}

/* Worked example: one real SPI register read. Explain in your README why
 * bit 7 is set, a dummy byte is sent, rx[1] holds the value, and CS stays low. */
imu_status_t imu_read_register(const imu_device_t *imu, uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(reg | 0x80u), 0};
    uint8_t rx[2] = {0};
    HAL_StatusTypeDef result;

    if (!valid_device(imu) || value == NULL || reg > 0x7fu) {
        return IMU_ERROR_ARGUMENT;
    }
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET); /* CS low. */
    result = HAL_SPI_TransmitReceive(imu->spi, tx, rx, sizeof(tx), 100);
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);   /* CS high, even on error. */
    if (result != HAL_OK) {
        return IMU_ERROR_TRANSFER;
    }
    *value = rx[1];
    return IMU_OK;
}

/* Start here: call the worked read rather than constructing a new transfer. */
imu_status_t imu_check_identity(const imu_device_t *imu)
{
    if (!valid_device(imu)) {
        return IMU_ERROR_ARGUMENT;
    }

    // Declare a uint8_t to hold identity
    uint8_t identity;
    // Declare an imu_status_t for the result
    imu_status_t status;
    
    // Call imu_read_register() with register 0x0F and the output's address
    status = imu_read_register(imu, 0x0F, &identity);

    // Return any read error immediately
    if (status != IMU_OK) {
        return status;
    } 

    // Return IMU_OK for 0x6C; 
    if (identity == 0x6C) {
        return IMU_OK;
    // otherwise return IMU_ERROR_IDENTITY
    } else {
        return IMU_ERROR_IDENTITY;
    }
}

/* Next: adapt the worked read to send a register address and its new value. */
imu_status_t imu_write_register(const imu_device_t *imu, uint8_t reg, uint8_t value)
{
    if (!valid_device(imu) || reg > 0x7fu) {
        return IMU_ERROR_ARGUMENT;
    }
    
    // Prepare two transmit bytes: reg (bit 7 clear) and value.
    uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), value}; // clear bit 7 for write operation
    uint8_t rx[2] = {0};
    HAL_StatusTypeDef result;

    // same as imu_read_register
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET); /* CS low. */
    result = HAL_SPI_TransmitReceive(imu->spi, tx, rx, sizeof(tx), 100);
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);   /* CS high, even on error. */

    // Return IMU_OK on HAL_OK
    if (result == HAL_OK) {
        return IMU_OK;
    // otherwise return IMU_ERROR_TRANSFER
    } else {
        return IMU_ERROR_TRANSFER;
    }
}
