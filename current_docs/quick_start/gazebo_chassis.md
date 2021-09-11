---
id: gazebo_chassis
sidebar_position: 2
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Gazebo 底盘
:::danger
从包安装可能无法使用。本文还未完成。
:::
本文将带你在 Gazebo 仿真中操控麦克纳姆轮以及舵轮底盘。

## 运行仿真
### 从包安装

    sudo apt install ros-noetic-rm-rm_gazebo ros-noetic-rm-description

### 从源编译

进入你的工作空间（假设名为 `rm_ws`），在你的工作空间中拉取本 `rm_control`，并使用 `rosdep` 安装包的依赖，并编译。

```shell
cd ~/ws_ws/src
git clone https://github.com/rm-controls/rm_control.git
rosdep install --from-paths . --ignore-src
catkin build
```

:::tip
必须确保你的 `rosdep` 安装和初始化是正确的。
:::

:::caution
你**应该**在你的日常开发电脑中执行下述操作；而**不**在机器人上的计算设备上面安装仿真以及可视化相关的包，更不要在上面运行仿真。
:::


将环境变量 `ROBOT_TYPE` 为目标机器人 :
<Tabs
groupId="operating-systems"
defaultValue="麦克纳姆轮"
values={[
{label: '舵轮', value: '舵轮'},
{label: '麦克纳姆轮', value: '麦克纳姆轮'},
]
}>
<TabItem value="麦克纳姆轮">export ROBOT_TYPE=hero</TabItem>
<TabItem value="舵轮">export ROBOT_TYPE=standard4</TabItem>
</Tabs>


![](/img/gazebo_chassis/chassis1.png)

![](/img/gazebo_chassis/chassis2.png)

启动目标机器人的仿真

```shell
 mon launch rm_gazebo empty_world.launch load_gimbal:= false load_shooter:= false
```
启动了目标机器人仿真的同时禁止加载机器人的云台和发射机构（当云台机构未加载的时候，如果加载发射机构会产生报错）
，因为我们此次的目标是在仿真中去控制底盘的运动。
## 运行控制器

### 从包安装

<Tabs
groupId="operating-systems"
defaultValue="麦克纳姆轮"
values={[
{label: '舵轮', value: '舵轮'},
{label: '麦克纳姆轮', value: '麦克纳姆轮'},
]
}>
<TabItem value="麦克纳姆轮">export ROBOT_TYPE=hero</TabItem>
<TabItem value="舵轮">export ROBOT_TYPE=standard4</TabItem>
</Tabs>

### 利用命令行控制底盘

进入你的工作空间（假设名为 `rm_ws`），在你的工作空间中拉取本次教程所要用到的仿真包 `rm_chassis_controllers`，并使用 `rosdep` 安装包的依赖。

```shell
cd ~/ws_ws/src
git clone https://github.com/rm-controls/rm_controllers.git
rosdep install --from-paths . --ignore-src
catkin build
```

加载并开始底盘控制器：

```
 mon launch rm_chassis_controllers load_controllers.launch
```

设置mode的序号去设置底盘模式其中:

1. 0代表RAW模式，是底盘的初始状态，是底盘的初始状态，此模式下底盘无法运动。
2. 1代表FOLLOW模式，是云台跟随模式，底盘跟随着云台的方向运动。
3. 2代表GYRO模式，是小陀螺模式，底盘以自身为中心旋转。
4. 3代表TWIST模式，是扭腰模式，底盘的转动不会受云台方向影响。

设置accel（加速度），去设置底盘的线和角加速度并且在power_limit中去设置底盘的限制功率。

```shell
rostopic pub /controllers/chassis_controller/command rm_msgs/ChassisCmd "mode: 0
accel:
  linear: {x: 0.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}
power_limit: 0.0
follow_source_frame: ''
stamp: {secs: 0, nsecs: 0}" 
```

设置底盘速度，让其运动的指令如下：

```shell
rostopic pub -r 100 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```
发送以上指令去控制底盘的运动，其中需要加入参数-r 100,意思是以频率为100hz的速度去发布命令，当频率设置的较小会观察到底盘运动的一卡一卡的现象。
因为在实车上为了防止机器人失去控制，底盘的逻辑是当未接受到指令的时候，速度会自动置0。

详见[rm_chassis_controllers/README](https://github.com/rm-controls/rm_controllers/blob/master/rm_chassis_controllers/README.md)

示例如下：
示例1：

```shell
rostopic pub /controllers/chassis_controller/command rm_msgs/ChassisCmd "mode: 0
accel:
  linear: {x: 1.0, y: 1.0, z: 1.0}
  angular: {x: 1.0, y: 1.0, z: 1.0}
power_limit: 200.0
follow_source_frame: ''
stamp: {secs: 0, nsecs: 0}" 
```
输入以上指令后会在rm_gazebo的终端中观察到底盘的状态变化为FOLLOW，此时可通过topic“cmd-vel”去控制底盘的运动。

```shell
rostopic pub -r 100 /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```
输入以上指令后观察到底盘直线运动。

示例2：

```shell
rostopic pub /controllers/chassis_controller/command rm_msgs/ChassisCmd "mode: 2
accel:
  linear: {x: 2.0, y: 2.0, z: 2.0}
  angular: {x: 0.0, y: 0.0, z: 10.0}
power_limit: 200.0
follow_source_frame: ''
```
输入以上指令后会在rm_gazebo的终端中观察到底盘的状态变化为GYRO，并且设置了底盘的线加速度和角加速度，此时可通过topic“cmd-vel”去控制底盘旋转着运动。

```shell
rostopic pub -r 100 /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0" 
```
输入以上指令后观察到底盘旋转着做直线运动。
