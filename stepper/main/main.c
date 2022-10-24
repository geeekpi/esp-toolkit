#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "stepper.h"

/* 节拍法控制的步进,八步更精细,理解起来非常容易,引脚铀主程序这里规定,传入到自己开发的库里. */

/* 这个电机启动电流相当大,如果出现USB口突然找不到串口,大概是过流了,换个接口. */

/* 为了便于观察,可以在上面绑点东西,比如牙签,否则很难观察到转动.(因为步长本身短,哪怕走100步也没什么感觉.) */

void app_main(void)
{
    struct stepper_pins stepper0;

    /* 这里定义4个用的引脚. */
    stepper0.pin1 = 27;
    stepper0.pin2 = 26;
    stepper0.pin3 = 25;
    stepper0.pin4 = 33;

    stepper_init(&stepper0);

    while (true)
    {

        full_steps(&stepper0, 100, CW);
        vTaskDelay(10);
        full_steps(&stepper0, 100, CCW);
        vTaskDelay(10);
    }
}
