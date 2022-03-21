# 插件

## 模块介绍

本队伍的 RoboMaster 机器人平台基于模块化设计，各个模块通过插件的形式接入主控程序。

在 rm_base 功能包中，定义了一个 PluginBase 类作为插件的基类，在 rm_plugins 功能包中，创建了相应的插件类。

全部 plugins（插件）放在 rm_plugins 功能包中，一共有 5 种插件（chassis、gimbal、gpio、imu、shooter）。本文档用于说明各个插件的具体功能。

rm_plugins 功能包的文件结构如下：

```
rm_plugins
├── cfg
│   ├── chassis.cfg
|   └── gimbal.cfg
|   └── shooter.cfg
├── include
│   └── chassis
│       ├── chassis_config.h
|       └── chassis_plugins.h
|   └── gimbal
|       ├── gimbal_config.h
|       └── gimbal_plugins.h
|   └── gpio
|       ├── gpio_manager.h
|       └── gpio_plugins.h
|   └── imu
|       └── hi220
|           ├── gpio_manager.h
|           └── gpio_plugins.h
|       └── imu_plugins.h
|   └── shooter
|       ├── shooter_config.h
|       └── shooter_plugins.h
├── param
|       └── chassis
|           └── standard.yaml
|       └── gimbal
|           └── standard.yaml
|       └── joint
|           ├── hero.yaml
|           └── standard.yaml
├── src
|       └── chassis
|           └── chassis_plugins.cpp
|       └── gimbal
|           └── gimbal_plugins.cpp
|       └── gpio
|           ├── gpio_manager.cpp
|           └── gpio_plugins.cpp
|       └── imu
|           └── hi220
|               └── hi220.cpp
|           └── imu_plugins.cpp
|       └── shooter
|           └── shooter_plugins.cpp
├── chassis_plugins.xml
├── gimbal_plugins.xml
├── gpio_plugins.xml
├── imu_plugins.xml
├── shooter_plugins.xml
├── package.xml
└── CMakeLists.txt
```

## 对各 Topic 的功能的简述

|      话题      |  发布消息  |                       功能                       |
| :------------: | :--------: | :----------------------------------------------: |
| “/chassis_cmd” | ChassisCmd |                发布底盘的运行状态                |
| "/gimbal_cmd"  | GimbalCmd  | 发布底盘的运行状态，pitch 轴和 yaw 轴的运动速率  |
|  "shoot_cmd"   |  ShootCmd  | 发布发射机构的运动状态，弹丸发射数量，速度，频率 |
|   "vel_cmd"    |   Twist    |           发布底盘运动的角速度和线速度           |

各消息类型的具体情况可见“rm_msgs/msg”路径下对应的文件。

## 对基类的说明

在“rm_base/include/base/plugin_base.h”路径下，定义了一个名为 PluginBase 的类，该类是所有插件的基类。在 PluginBase 中，定义了 4 个基本成员函数，这是各模块实现基本功能的函数，现对其进行简单说明。

- init：构建一个 ROS 包装器。
- run：启动状态机，根据当前模块的运动状态执行相应功能。
- enble：启动关节（电机）。
- disable：关闭关节（电机）。

## 底盘模块

在 chassis_plugins.h 文件中，创建了底盘模块对应的插件类 StandChassis。现对该插件类的主要成员和运动状态进行说明。

### 1、主要成员

- computerData：使用逆运动学，由关节数据(电机速度)，计算底盘运动数据和 Odom 坐标变换。
- fKine：使用正向运动学，在底盘框架下，根据速度计算关节数据（电机速度）。
- getVelData：获取底盘运动时，x,y,z 方向上的线速度。
- chassisCmdCB：底盘命令回调函数。
- state\_：枚举型变量，含 5 种取值（PASSIVE，RAW，FOLLOW，TWIST，GYRO），表示运动状态。
- chass*cmd_sub*：ROS 订阅器，启动 init 函数后，订阅"/chassis_cmd"话题所发布的消息。
- vel*cmd_sub*：ROS 订阅器，启动 init 函数后，订阅“/vel_cmd_sub”话题所发布的消息。

### 2、底盘状态机

当底盘状态广播器所发布的状态消息改变时，chassis*cmd_sub*接收到消息后，会进入 chassisCmdCB 函数，改变 state\_\_的值，此时，在 run 函数中，便会启动相应状态所对应功能的函数。

- 当 state\_的值为 PASSIVE 时，会启动 passive 函数，此时底盘所有电机断电。

- 当 state\_的值为 RAW 时，启动 raw 函数，会启动 recoverK 函，恢复各节点（电机）的状态，并保存原始数据。

- 当 state\_的值为 FLLOW、TWIST、GYRO 时，启动相应状态下的函数，会启动 recoverK 函数，恢复各节点（电机）的状态

## 云台模块

在 gimbal_plugins.h 文件中，创建了云台模块对应的插件类 StandGimbal。现对该插件类的主要成员和运动状态进行说明。

### 1、基本成员

- computerData：根据关节（电机）数据，计算云台数据并更新 tf 坐标变换。

- cmdCB：云台命令回调函数。

- state\_：枚举型变量，含 3 种取值（PASSIVE，RATE，TRACK），表示运动状态。

- cmd*sub*：ROS 订阅器，启动 init 函数后，订阅“/Gimbal_cmd”话题所发布的消息。

### 2、云台状态机

当云台状态广播器所发布的状态消息改变时，cmd*sub*接收到消息后，会进入 cmdCB 函数，改变 state\_\_的值，此时，在 run 函数中，便会启动相应状态所对应功能的函数。

- 当 state\_的值为 PASSIVE 时，执行 run 函数后，会启动 passive 函数，此时，控制 yaw 轴和 pitch 轴的电机都会断电。

- 当 state\_的值为 RATE 时，执行 run 函数后，会启动 rate 函数，此时，pos loop 启动。

- 当 state\_的值为 TRACK 时，执行 run 函数后，会启动 track 函数，此时，云台工作在跟踪模式下。

## 发射机构

在 shooter_plugins.h 文件中，创建了发射机构对应的插件类 StandShooter。现对该插件类的主要成员和运动状态进行说明。

### 1、基本成员

- shoot：对发射弹丸的数量和发射频率进行设置。

- setSpeed：设置弹丸的发射速度。

- cmdCB：发射命令回调函数。

- state\_：枚举型变量，含 5 种取值（PASSIVE，FEED、READY、PUSH、BLOCK），表示运动状态。

- cmd*sub*：ROS 订阅器，启动 init 函数后，订阅“/Shooter_cmd”话题所发布的消息。

### 2、发射机构状态机说明

当发射机构状态广播器所发布的状态消息改变时，cmd*sub*接收到消息后，会进入 cmdCB 函数。

若所发布的消息中的 mode 为 0，则 state\_的值修改为 PASSIVE。

否则，若当前的 state\_值为 PASSIVE，则将其改为 FEED。

接着，根据发布的发射速度（作为函数的输入）启动 setSpeed 函数，以及根据所发布的弹丸数量和发射频率（作为函数的输入），启动 shoot 函数。

- 当 state\_的值为 PASSIVE 时，执行 run 函数后，电机断电。

- 当 state\_的值为 FEED 时，执行 run 函数后，进行补弹操作。打开拨弹轮电机，将弹丸运输至发射口。

  若运输过程中，拨弹轮发射堵塞，state\_值会被修改为 BLOCK；

  当弹丸运输至发射口前，触碰到发射开关时，state\_的值会被修改为 READY。

- 当 state\_的值为 BLOCK 时，表示拨弹轮堵塞的，执行 run 函数后，拨弹轮会回拨一段时间，解决堵塞的情况。

- 当 state\_的值为 READY 时，执行 run 函数后，会停止拨弹轮电机，启动 pos 循环，同时判断可发射的弹丸数量和发射频率是否达到可进行发射操作的要求，若是，则将 state\_\_的值转变为 PUSH。

- 当 state\_的值为 PUSH 时，执行 run 函数后，摩擦轮转动，将弹丸发射出去。

## Gpio

在该功能包下的 Gpio 文件夹下，定义了两个类 UpboardGpio 和 GpioManager，前者为插件类，后者为是一个 Gpio 管理器，作为 UpboardGpio 的一个 praivate 类型的成员变量。

在 gpio_plugins.h 文件中，创建了 Gpio 对应的插件类 UpboardGpio。现对该插件类的主要成员函数进行说明。

- init：初始化 Gpio 管理器。
- readInput：读取相应接口的数据，并更新 Gpio 管理器。
- setOutput：函数的输入为引脚号，和一个 bool 型变量 output，可根据 output 的值选择是否将该引脚设为输出口。
- setPWM：设置对应引脚号的 PWM 输出占空比。

## IMU

在该功能包下的 IMU 文件夹下，定义了 3 个类 Imu，Hi22Imu、Hi220。其中，Hi220 为插件类，Imu 为基类，Hi220Imu 类是 Imu 功能的一个集合，作为 Hi220 的一个 private 成员变量。

在 hi220.h 中定义了 Hi220Imu 类。该类的一个实例作为插件类的一个成员变量（hi220*imu*），集成了读取 IMU 相关数据的函数，现对该类的主要成员函数进行说明。

- init：开启并设置串口。
- getId：获取用户 ID。
- run：读取缓冲区数据。
- getEular：将当前欧拉角坐标系下的坐标数据拷贝到指定变量中。
- getQuat：将当前四元素坐标系下的坐标数据拷贝到指定变量中。
- getAcc：将当前加速度的数据拷贝到指定变量中。
- getOmega：将当前角速度的数据拷贝到指定变量中。
- getImuData：将 IMU 中的相关数据写入串口中。

在 imu_plugins.h 文件中，创建了 IMU 模块对应的插件类 Hi220。现对该插件类的主要成员函数进行说明。

- init：启动 hi220_imu 的 init 成员函数。
- update：启动 hi220*imu*的 run 成员函数，并读取 IMU 的相关数据，写入串口中，最后进行 odom 坐标系转 base 坐标系操作并设置坐标变换。
