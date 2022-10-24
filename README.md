# ESP-Tookit 例子 (草稿)

------

### 工程描述

| 项目 | 硬件 | 描述 |
| ---- | ---- | --- |
| adc | 旋转摇杆 | 3V3供电,默认使用GPIO34采样,下载运行,然后打开Monitor观测数据,调节旋钮数据变化. |
| lcd1602 | I2C LCD1602 | SDA = IO18,SCL = IO19,下载运行,打开Monitor,运行相应指令可以写内容在屏幕上,具体命令可以查看终端. |
| mpu6050 | MPU6050 模块 | SDA = IO18,SCL = IO19,下载运行打开Monitor观测数据. |
| oled | SSD1306 模块 | LVGL库的移植和演示,默认SDA = 5,SCL = 4. |
| servo | 舵机 | 下载运行可以看到舵机来回运动,默认使用CH0(IO32). |
| stepper | 步进电机和驱动模块 | 下载运行可以看到步进电机单方向运动,默认使用IN0(IO27),IN1(IO26),IN2(IO25),IN3(IO33). |
| ultrasonic | 超声波SR04 | 下载运行打开Monitor可观测测量数值,默认使用TRIG(IO27),ECHO(IO26). |
| ws2812 | WS2812 | 下载并运行观测效果,默认接在IO18,受限于RMT外设,IO不能随便修改. |

### 克隆本工程

```shell
git clone --recursive https://git.magnetic.ac.cn/nickfox-taterli/esp-toolkit.git
```

