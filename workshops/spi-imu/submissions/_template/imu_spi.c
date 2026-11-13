#include "../../firmware/Core/Inc/imu_spi.h"
#include <stdint.h>

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
    /* TODO 1:
     * - Declare a uint8_t to hold identity and an imu_status_t for the result.
     * - Call imu_read_register() with register 0x0F and the output's address.
     * - Return any read error immediately.
     * - Return IMU_OK for 0x6C; otherwise return IMU_ERROR_IDENTITY.
     */

    uint8_t identity;
    imu_status_t result;

    result= imu_read_register(imu, 0x0F, &identity);
    if(result!= IMU_OK){
        return result; // error type returned in imu_read_register
    }

    
    

    return identity == 0x6C ? IMU_OK : IMU_ERROR_IDENTITY;
}

/* Next: adapt the worked read to send a register address and its new value. */
imu_status_t imu_write_register(const imu_device_t *imu, uint8_t reg, uint8_t value)
{
    if (!valid_device(imu) || reg > 0x7fu) {
        return IMU_ERROR_ARGUMENT;
    }
    /* TODO 2:
     * - Prepare two transmit bytes: reg (bit 7 clear) and value.
     * - Prepare a two-byte receive buffer for HAL_SPI_TransmitReceive().
     * - Hold CS low for the entire transfer; release it even on failure.
     * - Return IMU_OK on HAL_OK, otherwise IMU_ERROR_TRANSFER.
     */

    uint8_t tx[2]={reg & 0x7F, value};
    // Bitwise AND every value of reg with 01111111, to CLEAR msb 7. Bit 7 denotes the read/write flag of a uint8_t variable. 1:read, 0:write. We are writing on reg
    uint8_t rx[2]={0};

    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);   //Reset sets CS to low like an SR latch
    HAL_StatusTypeDef result= HAL_SPI_TransmitReceive(imu->spi, &tx[0], &rx[0], sizeof(uint16_t), 100); //Complete entire transfer
    
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET); // Release CS, unconditionally
    
    if(result==HAL_OK)
        return IMU_OK;
    else    
        return IMU_ERROR_TRANSFER;

}
