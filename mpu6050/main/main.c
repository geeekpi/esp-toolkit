#include "sdkconfig.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "mpu6050.h"

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Axis3i;

typedef struct sensorData_s
{
    Axis3i acc;
    Axis3i gyro;
} sensorData_t;

static sensorData_t sensorData;

void app_main(void)
{
    mpu6050Init();

    if (mpu6050TestConnection() == true)
    {
        printf("MPU6050 I2C connection [OK].\n");
    }
    else
    {
        printf("MPU6050 I2C connection [FAIL].\n");
    }

    mpu6050Reset();
    vTaskDelay(50);
    // Activate mpu6050
    mpu6050SetSleepEnabled(false);
    // Delay until registers are reset
    vTaskDelay(100);
    // Set x-axis gyro as clock source
    mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    // Delay until clock is set and stable
    vTaskDelay(200);
    // Enable temp sensor
    mpu6050SetTempSensorEnabled(true);
    // Disable interrupts
    mpu6050SetIntEnabled(false);
    // Connect the MAG and BARO to the main I2C bus
    mpu6050SetI2CBypassEnabled(true);
    // Set gyro full scale range
    mpu6050SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    // Set accelerometer full scale range
    mpu6050SetFullScaleAccelRange(MPU6050_ACCEL_FS_16);

    mpu6050SetRate(0);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);

    while (true)
    {
        vTaskDelay(100);
        sensorData.acc.x = mpu6050GetAccelerationX();
        sensorData.acc.y = mpu6050GetAccelerationY();
        sensorData.acc.z = mpu6050GetAccelerationZ();
        sensorData.gyro.x = mpu6050GetRotationX();
        sensorData.gyro.y = mpu6050GetRotationY();
        sensorData.gyro.z = mpu6050GetRotationZ();
        printf("ax = %d,  ay = %d,  az = %d,  gx = %d,  gy = %d,  gz = %d\n", sensorData.acc.x, sensorData.acc.y, sensorData.acc.z, sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z);
    }
}
