#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "iot_servo.h"

/* 可以接8个舵机(PWM),如果是ESP32本身不计算保留资源,可以接16个. */

/* 由于垃圾舵机质量不好,所以运行会有回撤一样的抖动,在好舵机上测试未见异常. */

#define SERVO_CH0_PIN 32
#define SERVO_CH1_PIN 25
#define SERVO_CH2_PIN 26
#define SERVO_CH3_PIN 27
#define SERVO_CH4_PIN 14
#define SERVO_CH5_PIN 12
#define SERVO_CH6_PIN 13
#define SERVO_CH7_PIN 15

void app_main(void)
{
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_CH0_PIN,
                SERVO_CH1_PIN,
                SERVO_CH2_PIN,
                SERVO_CH3_PIN,
                SERVO_CH4_PIN,
                SERVO_CH5_PIN,
                SERVO_CH6_PIN,
                SERVO_CH7_PIN,
            },
            .ch = {
                LEDC_CHANNEL_0,
                LEDC_CHANNEL_1,
                LEDC_CHANNEL_2,
                LEDC_CHANNEL_3,
                LEDC_CHANNEL_4,
                LEDC_CHANNEL_5,
                LEDC_CHANNEL_6,
                LEDC_CHANNEL_7,
            },
        },
        .channel_number = 8,
    };
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);

    float angle = 0.0f;

    while (true)
    {
        for (angle = 0.0f; angle < 180.0f; angle = angle + 5.0f)
        {
            // Set angle to 100 degree - CH0
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, angle);

            // Get current angle of servo - CH0
            iot_servo_read_angle(LEDC_LOW_SPEED_MODE, 0, &angle);

            vTaskDelay(10);

            printf("Change to %d degC\r\n", (int)angle);
        }

        for (angle = 180.0f; angle > 0.0f; angle = angle - 5.0f)
        {
            // Set angle to 100 degree - CH0
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, angle);

            // Get current angle of servo - CH0
            iot_servo_read_angle(LEDC_LOW_SPEED_MODE, 0, &angle);

            vTaskDelay(10);

            printf("Change to %d degC\r\n", (int)angle);
        }
    }
}
