---
id: road_map
sidebar_position: 4
---

# Road Map
## README 文档
* [x] Why rm-controls?
* [ ] rm-controls 101
* [ ] Gazebo 底盘
* [ ] rm_engineer
* [ ] rm_dbus

## Doxygen 文档
* [x] rm_hw
* [x] rm_calibration_controllers
* [ ] ...

## IO
本项目的较大缺点是 IO 数量种类太少，因此开发对应的转接模块和对应测试接口
### 模块转 CAN
* [x] HI229 (测试中)
* [x] ICM-20948 (测试中)
* [ ] BIM-88
* [ ] GPIO

### 模块转 i2c 转 USB
通过 [I2C-Tiny-USB](https://github.com/harbaum/I2C-Tiny-USB) 项目，它的驱动是 Linux 主线支持的，可以非常方便地获得 i2c 接口，并使用 I2C/SMBus 子系统。
* [x] pcf8574 GPIO 拓展 (测试中)
* [x] pca9685 PWM 拓展 (测试中)
* [ ] ICM-20948