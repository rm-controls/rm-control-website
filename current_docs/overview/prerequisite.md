# Prerequisite
下面阐述 ros-control 在实物中和 Gazebo 仿真中的相关机制。
## ros-control 与真实硬件
如下图，ros-control 提供这样的机制： 执行器（Actuator）的编码器等传感器数据被读取后通过 [`TransmissionsInterface`](http://wiki.ros.org/transmission_interface) 映射成关节 (Joint) 等机器人状态，将这些状态接口提供给控制器；经过控制器计算后得到关节指令经过限制，再映射为电机的指令，发送给电机。控制器管理器可以实时加载、开始和停止各种控制器（以动态库的形式编译）。

![ros_control 框图](http://wiki.ros.org/ros_control?action=AttachFile&do=get&target=gazebo_ros_control.png)

### 硬件接口
提供给控制器常见硬件接口
* Joint Command Interface - 关节指令发送接口
    * Effort Joint Interface - 用于向指令是力矩/力的执行器 (如: RoboMaster 3508) 对应的关节发送指令
    * Velocity Joint Interface  - 用于向指令是速度执行器 (如: 部分舵机) 对应的关节
    * Position Joint Interface  - 用于向指令是位置执行器 (如: 大部分舵机) 对应的关节
* Joint State Interfaces - 关节状态获取接口，用于获取关节的位置、速度和作用力（力或扭矩）
* Actuator State Interfaces - 执行器状态获取接口，用于获取执行器的位置、速度和作用力（力或扭矩）
* Actuator Command Interfaces - 执行器指令发送接口，和关节指令接口类似
* Force-torque sensor Interface - 力-力矩传感器接口
* IMU sensor Interface - IMU 传感器接口

### Transmissions
Transmissions 是实际机器人执行器于关节的状态和指令映射，有简单减速比（改变正负可以将电机反向）、差速器（常见于机械臂末端两个关节）、双执行器（两个电机带动一个关节）。下图展示了差分 Transmissions 的简图和计算方法。

![差分 transmission](/img/prerequisite/transmission.png) 

下列代码为步兵机器人 URDF 中云台 pitch 轴执行器： `pitch_joint_motor` 与云台 pitch 关节 `pitch_joint` 的 [`simple_transmission`](http://docs.ros.org/en/melodic/api/transmission_interface/html/c++/classtransmission__interface_1_1SimpleTransmission.html) ，由于 pitch 轴是 6020 电机直驱，减速比为 1.0，又实际电机转向于关节定义的转向相反，取减速比为 -1.0，在校准时实测得偏移为 1.559rad。有了这一个的映射，云台控制器只使用关节状态不需要考虑不同机器人电机安装位置、方向和初始值。

```xml
<transmission name="trans_pitch_joint">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="pitch_joint_motor">
        <mechanicalReduction>-1</mechanicalReduction>
    </actuator>
    <joint name="pitch_joint">
        <offset>1.559</offset>
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
</transmission>
```

## ros-control 与 Gazebo
得益于上述的机制，控制器可以被加载到硬件接口上，也可以被加载到Gazebo模拟器中, 如下图所示。值得一提的是实车硬件和仿真用的控制器都是同一份代码，不存在移植或重新编译的过程，甚至使用的是同一个二进制文件（由服务器CI编译测试并发布到apt源并安装）。
![Gazebo_ros_transmission](https://github.com/osrf/gazebo_tutorials/raw/master/ros_control/Gazebo_ros_transmission.png) 