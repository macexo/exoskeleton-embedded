#include "imu_spi.h"

static int valid_device(const imu_device_t *imu)
{
    return imu != NULL && imu->spi != NULL && imu->cs_port != NULL && imu->cs_pin != 0;
}


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


imu_status_t imu_check_identity(const imu_device_t *imu)
{
    if (!valid_device(imu)) {
        return IMU_ERROR_ARGUMENT;
    }

    uint8_t identity;
    imu_status_t result;

    result = imu_read_register(imu, 0x0F, &identity); //0x0F: register address for IMU identity

    if(result!=IMU_OK){
        return result;
    }

    //identity should be 0x6C
    if(identity!=0x6C){
        return IMU_ERROR_IDENTITY;
    }

    return IMU_OK;
}

/* Next: adapt the worked read to send a register address and its new value. */
imu_status_t imu_write_register(const imu_device_t *imu, uint8_t reg, uint8_t value)
{
    if (!valid_device(imu) || reg > 0x7fu) {
        return IMU_ERROR_ARGUMENT;
    }

    uint8_t tx[2] = {(uint8_t)(reg & ~(0x80u)), value};
    uint8_t rx[2] = {0};
    HAL_StatusTypeDef result;
    
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET); // CS low.
    result = HAL_SPI_TransmitReceive(imu->spi, tx, rx, sizeof(tx), 100);
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);   // CS high, even on error.
    
    if (result != HAL_OK) {
        return IMU_ERROR_TRANSFER;
    }

    return IMU_OK;
}
