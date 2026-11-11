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
    uint8_t identity = 0;
    imu_status_t status;

    // WHO_AM_I (0x0F) is read-only and should always return 0x6C (datasheet 9.11)
    status = imu_read_register(imu, 0x0Fu, &identity);
    if (status != IMU_OK) {
        return status; // Pass the read error up; identity is not valid. 
    }
    if (identity != 0x6Cu) {
        return IMU_ERROR_IDENTITY; // Wrong chip, wiring fault, or bus reading 0x00/0xFF
    }
    return IMU_OK;
}

// Next: adapt the worked read to send a register address and its new value
imu_status_t imu_write_register(const imu_device_t *imu, uint8_t reg, uint8_t value)
{
    if (!valid_device(imu) || reg > 0x7fu) {
        return IMU_ERROR_ARGUMENT;
    }
    // Bit 7 clear = write (datasheet 5.1.2). Byte 0 is the address, byte 1 the new value
    uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), value};
    uint8_t rx[2] = {0}; // Full duplex: the IMU clocks bytes back, but they mean nothing on a write
    HAL_StatusTypeDef result;

    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET); // CS low: start of transaction
    result = HAL_SPI_TransmitReceive(imu->spi, tx, rx, sizeof(tx), 100);
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);   // CS high, even on error
    if (result != HAL_OK) {
        return IMU_ERROR_TRANSFER;
    }
    return IMU_OK;
}
