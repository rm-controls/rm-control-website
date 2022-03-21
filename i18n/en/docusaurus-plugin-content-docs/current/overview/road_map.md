---
id: road_map
sidebar_position: 4
---

# Road Map

## Documentation

### README

- [x] Why rm-controls?
- [ ] rm-controls 101
- [ ] Gazebo Chassis
- [ ] rm_engineer
- [ ] rm_dbus
- [ ] rm_bringup

### Doxygen

- [x] rm_hw
- [ x ] rm_calibration_controllers
- [ ] rm_gazebo
- [ ] rm_chassis_controllers
- [ ] rm_gimbal_controllers
- [ ] rm_shooter_controllers

## IO

The major drawback of this project is that the number of IO types is too small, so the corresponding adapter module and the corresponding test interface are developed

### Module to CAN

- [x] HI229 (under test)
- [x] ICM-20948 (under test)
- [ ] BIM-88
- [ ] GPIO

### Module to i2c to USB

With the [I2C-Tiny-USB](https://github.com/harbaum/I2C-Tiny-USB) project, whose driver is supported by the Linux mainline, it is very easy to get the i2c interface and use the I2C/SMBus subsystem.

- [x] pcf8574 GPIO extension (under test)
- [x] pca9685 PWM extension (under test)
- [ ] ICM-20948
