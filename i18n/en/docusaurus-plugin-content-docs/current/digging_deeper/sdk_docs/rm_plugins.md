# 插件

## 模块介绍

本队伍的RoboMaster机器人平台基于模块化设计，各个模块通过插件的形式接入主控程序。

在rm_base功能包中，定义了一个PluginBase类作为插件的基类，在rm_plugins功能包中，创建了相应的插件类。

全部plugins（插件）放在rm_plugins功能包中，一共有5种插件（chassis、gimbal、gpio、imu、shooter）。本文档用于说明各个插件的具体功能。

rm_plugins功能包的文件结构如下：

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



## 对各Topic的功能的简述

|      话题      |  发布消息  |                       功能                       |
| :------------: | :--------: | :----------------------------------------------: |
| “/chassis_cmd” | ChassisCmd |                发布底盘的运行状态                |
| "/gimbal_cmd"  | GimbalCmd  |   发布底盘的运行状态，pitch轴和yaw轴的运动速率   |
|  "shoot_cmd"   |  ShootCmd  | 发布发射机构的运动状态，弹丸发射数量，速度，频率 |
|   "vel_cmd"    |   Twist    |           发布底盘运动的角速度和线速度           |

各消息类型的具体情况可见“rm_msgs/msg”路径下对应的文件。



## 对基类的说明

在“rm_base/include/base/plugin_base.h”路径下，定义了一个名为PluginBase的类，该类是所有插件的基类。在PluginBase中，定义了4个基本成员函数，这是各模块实现基本功能的函数，现对其进行简单说明。

+ init：构建一个ROS包装器。
+ run：启动状态机，根据当前模块的运动状态执行相应功能。
+ enble：启动关节（电机）。
+ disable：关闭关节（电机）。



## 底盘模块

在chassis_plugins.h文件中，创建了底盘模块对应的插件类StandChassis。现对该插件类的主要成员和运动状态进行说明。

### 1、主要成员

+ computerData：使用逆运动学，由关节数据(电机速度)，计算底盘运动数据和Odom坐标变换。
+ fKine：使用正向运动学，在底盘框架下，根据速度计算关节数据（电机速度）。
+ getVelData：获取底盘运动时，x,y,z方向上的线速度。
+ chassisCmdCB：底盘命令回调函数。
+ state_：枚举型变量，含5种取值（PASSIVE，RAW，FOLLOW，TWIST，GYRO），表示运动状态。
+ chass_cmd_sub_：ROS订阅器，启动init函数后，订阅"/chassis_cmd"话题所发布的消息。
+ vel_cmd_sub_：ROS订阅器，启动init函数后，订阅“/vel_cmd_sub”话题所发布的消息。

 

### 2、底盘状态机

当底盘状态广播器所发布的状态消息改变时，chassis_cmd_sub_接收到消息后，会进入chassisCmdCB函数，改变state__的值，此时，在run函数中，便会启动相应状态所对应功能的函数。

- 当state_的值为PASSIVE时，会启动passive函数，此时底盘所有电机断电。

- 当state_的值为RAW时，启动raw函数，会启动recoverK函，恢复各节点（电机）的状态，并保存原始数据。

- 当state_的值为FLLOW、TWIST、GYRO时，启动相应状态下的函数，会启动recoverK函数，恢复各节点（电机）的状态





## 云台模块

在gimbal_plugins.h文件中，创建了云台模块对应的插件类StandGimbal。现对该插件类的主要成员和运动状态进行说明。

### 1、基本成员

+ computerData：根据关节（电机）数据，计算云台数据并更新tf坐标变换。

+ cmdCB：云台命令回调函数。

+ state_：枚举型变量，含3种取值（PASSIVE，RATE，TRACK），表示运动状态。

+ cmd_sub_：ROS订阅器，启动init函数后，订阅“/Gimbal_cmd”话题所发布的消息。

  

### 2、云台状态机

当云台状态广播器所发布的状态消息改变时，cmd_sub_接收到消息后，会进入cmdCB函数，改变state__的值，此时，在run函数中，便会启动相应状态所对应功能的函数。

+ 当state_的值为PASSIVE时，执行run函数后，会启动passive函数，此时，控制yaw轴和pitch轴的电机都会断电。

+ 当state_的值为RATE时，执行run函数后，会启动rate函数，此时，pos loop启动。

+ 当state_的值为TRACK时，执行run函数后，会启动track函数，此时，云台工作在跟踪模式下。



## 发射机构

在shooter_plugins.h文件中，创建了发射机构对应的插件类StandShooter。现对该插件类的主要成员和运动状态进行说明。

### 1、基本成员

+ shoot：对发射弹丸的数量和发射频率进行设置。

+ setSpeed：设置弹丸的发射速度。

+ cmdCB：发射命令回调函数。

+ state_：枚举型变量，含5种取值（PASSIVE，FEED、READY、PUSH、BLOCK），表示运动状态。

+ cmd_sub_：ROS订阅器，启动init函数后，订阅“/Shooter_cmd”话题所发布的消息。

  

### 2、发射机构状态机说明 

当发射机构状态广播器所发布的状态消息改变时，cmd_sub_接收到消息后，会进入cmdCB函数。

若所发布的消息中的mode为0，则state_的值修改为PASSIVE。

否则，若当前的state_值为PASSIVE，则将其改为FEED。

接着，根据发布的发射速度（作为函数的输入）启动setSpeed函数，以及根据所发布的弹丸数量和发射频率（作为函数的输入），启动shoot函数。

- 当state_的值为PASSIVE时，执行run函数后，电机断电。

- 当state_的值为FEED时，执行run函数后，进行补弹操作。打开拨弹轮电机，将弹丸运输至发射口。

  若运输过程中，拨弹轮发射堵塞，state_值会被修改为BLOCK；

  当弹丸运输至发射口前，触碰到发射开关时，state_的值会被修改为READY。

- 当state_的值为BLOCK时，表示拨弹轮堵塞的，执行run函数后，拨弹轮会回拨一段时间，解决堵塞的情况。

- 当state_的值为READY时，执行run函数后，会停止拨弹轮电机，启动pos循环，同时判断可发射的弹丸数量和发射频率是否达到可进行发射操作的要求，若是，则将state__的值转变为PUSH。

- 当state_的值为PUSH时，执行run函数后，摩擦轮转动，将弹丸发射出去。

  

## Gpio

在该功能包下的Gpio文件夹下，定义了两个类UpboardGpio和GpioManager，前者为插件类，后者为是一个Gpio管理器，作为UpboardGpio的一个praivate类型的成员变量。

在gpio_plugins.h文件中，创建了Gpio对应的插件类UpboardGpio。现对该插件类的主要成员函数进行说明。

+ init：初始化Gpio管理器。
+ readInput：读取相应接口的数据，并更新Gpio管理器。
+ setOutput：函数的输入为引脚号，和一个bool型变量output，可根据output的值选择是否将该引脚设为输出口。
+ setPWM：设置对应引脚号的PWM输出占空比。



## IMU

在该功能包下的IMU文件夹下，定义了3个类Imu，Hi22Imu、Hi220。其中，Hi220为插件类，Imu为基类，Hi220Imu类是Imu功能的一个集合，作为Hi220的一个private成员变量。

在hi220.h中定义了Hi220Imu类。该类的一个实例作为插件类的一个成员变量（hi220_imu_），集成了读取IMU相关数据的函数，现对该类的主要成员函数进行说明。

+ init：开启并设置串口。
+ getId：获取用户ID。
+ run：读取缓冲区数据。
+ getEular：将当前欧拉角坐标系下的坐标数据拷贝到指定变量中。
+ getQuat：将当前四元素坐标系下的坐标数据拷贝到指定变量中。
+ getAcc：将当前加速度的数据拷贝到指定变量中。
+ getOmega：将当前角速度的数据拷贝到指定变量中。
+ getImuData：将IMU中的相关数据写入串口中。

在imu_plugins.h文件中，创建了IMU模块对应的插件类Hi220。现对该插件类的主要成员函数进行说明。

+ init：启动hi220_imu的init成员函数。
+ update：启动hi220_imu_的run成员函数，并读取IMU的相关数据，写入串口中，最后进行odom坐标系转base坐标系操作并设置坐标变换。











