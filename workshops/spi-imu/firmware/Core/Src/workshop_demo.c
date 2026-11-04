/* Workshop hook for the LSM6DSO32 reference board project (98fd9f8).
 * main.c calls this after imu_init(), which establishes the sensor power-on
 * delay and enables SPI before chip select is asserted. Status 4 means the
 * selected member driver still has a TODO; status 0 means that call succeeded.
 * Sensor streaming is separate from these explicit student-function checks.
 */
#include "imu_spi.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static void workshop_report(const char *text)
{
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

void workshop_demo_run(void)
{
    const imu_device_t device = {&hspi3, IMU_CS_GPIO_Port, IMU_CS_Pin};
    char line[128];
    uint8_t readback = 0;
    imu_status_t status = imu_check_identity(&device);

    snprintf(line, sizeof(line), "workshop identity: status=%d (0=OK, 4=TODO)\r\n",
             (int)status);
    workshop_report(line);
    if (status != IMU_OK) return;

    status = imu_write_register(&device, 0x12, 0x44);
    snprintf(line, sizeof(line), "workshop write: status=%d (0=OK)\r\n", (int)status);
    workshop_report(line);
    if (status != IMU_OK) return;

    status = imu_read_register(&device, 0x12, &readback);
    snprintf(line, sizeof(line),
             "workshop readback: status=%d value=0x%02X expected=0x44 %s\r\n",
             (int)status, (unsigned)readback,
             status == IMU_OK && readback == 0x44 ? "MATCH" : "FAIL");
    workshop_report(line);
}
