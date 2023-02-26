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

### 安装机器人仿真包

    sudo apt install ros-noetic-rm-gazebo ros-noetic-rm-description


### 安装rm_control

:::tip
以下两种安装方式选择其一即可。
:::

#### 1.从源编译

进入你的工作空间（假设名为 `rm_ws`），在你的工作空间中拉取本 `rm_control`，并使用 `rosdep` 安装包的依赖，并编译。

```shell
cd ~/rm_ws/src
git clone https://github.com/rm-controls/rm_control.git
rosdep install --from-paths . --ignore-src
catkin build
```
:::tip
必须确保你的 `rosdep` 安装和初始化是正确的。
:::


#### 2.从包安装

    sudo apt install ros-noetic-rm-control
    
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
 mon launch rm_gazebo empty_world.launch load_gimbal:=false load_shooter:=false
```

启动目标机器人仿真的同时禁止加载机器人的云台和发射机构（当云台机构未加载的时候，加载发射机构会产生报错）
，因为我们此次的目标是在仿真中控制底盘的运动。

## 运行控制器

#### 拉取包并安装依赖

进入你的工作空间（假设名为 `rm_ws`），在你的工作空间中拉取本次教程所要用到的仿真包 `rm_chassis_controllers`，并使用 `rosdep` 安装包的依赖。

```shell
cd ~/rm_ws/src
git clone https://github.com/rm-controls/rm_controllers.git
rosdep install --from-paths . --ignore-src
catkin build
```

### 利用命令行控制底盘

#### 加载底盘控制器

<Tabs
groupId="operating-systems"
defaultValue="麦克纳姆轮"
values={[
{label: '舵轮', value: '舵轮'},
{label: '麦克纳姆轮', value: '麦克纳姆轮'},
]
}>

<TabItem value="麦克纳姆轮">

```
 mon launch robot_state_controller load_controllers.launch
 mon launch rm_chassis_controllers load_controllers.launch chassis_type:=mecanum
```
</TabItem>

<TabItem value="舵轮">

```
 mon launch robot_state_controller load_controllers.launch
 mon launch rm_chassis_controllers load_controllers.launch chassis_type:=swerve
```
</TabItem></Tabs>


#### 在 rqt 中的 Controller manager 中启动相关控制器

```
rqt
```

启动 rqt 后，依次选择`Plugins`,`Robot Tools`,`Controller manager`，以打开
Controller manager。在 namespace 中选择/controller_manager 可以看到出现三个处于`initialized`
状态的控制器。右键点击`robot_state_controller`，选择`start`，
接着对`chassis_controller`进行相同的操作，观察到`robot_state_controller`和
`chassis_controller`都处于`running`的状态。

成功加载底盘控制器，可以开始尝试控制底盘运动。

#### 设置底盘的各种参数

```shell
rostopic pub /controllers/chassis_controller/command rm_msgs/ChassisCmd "mode: 0
accel:
  linear: {x: 3.0, y: 3.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 4.0}
power_limit: 200.0
follow_source_frame: ''
command_source_frame: ''
stamp: {secs: 0, nsecs: 0}"
```

在`mode`中，设置底盘运动模式为`RAW`；

在`accel`中，设置底盘线加速度 x 轴和 y 轴方向上为 3m/s2，角加速度为 4rad/s2；

在`power_limit`中， 设置底盘功率限制为 200；

在`follow_source_frame`中，设置底盘跟随的坐标系（默认为`yaw`）。

在`command_source_frame`中，设置速度指令在哪个坐标系下（默认为`yaw`），此参数在RAW模式下无效。

#### 操控底盘运动

```shell
rostopic pub -r 50 /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

在`linear`和`angular`中设置底盘的线速度和角速度并发布后，观察到底盘直线运动。

发送以上指令控制底盘的运动，其中需要加入参数`-r 50`,意思是以频率为 50hz 的速度发布命令，当频率设置的较小会观察到底盘运动卡顿的现象。
因为在实车上为了防止机器人失去控制，底盘的逻辑是当未接受到指令的时候，速度会自动置 0。

此时可以观察到底盘的直线运动。

![](/img/gazebo_chassis/follow.gif)

#### mode 的设置

1. 0 代表 RAW 模式，在此模式下，来自/cmd_vel的速度指令被看作是底盘坐标系（/base_link）下的速度指令。
2. 1 代表 FOLLOW 模式，底盘跟随设定坐标系移动，底盘正面（即底盘坐标的 x 轴方向）与设定坐标系的 x 轴同向（默认跟随yaw坐标系）。
3. 2 代表 GYRO 模式，小陀螺模式下，底盘的直线运动会跟随设置的坐标系运动，同时自身能够旋转。
4. 3 代表 TWIST 模式，扭腰状态下，底盘正面将以刁钻角度面对敌方机器人（很难看见装甲板的角度），并且底盘不断小幅度旋转以防止敌方机器人击中我方机器人。

详见[rm_chassis_controllers/README](https://github.com/rm-controls/rm_controllers/blob/master/rm_chassis_controllers/README.md)

### 底盘操控实例

#### 示例 1： 控制底盘旋转着平移

设置底盘模式为`GYRO`，并设置好加速度

```shell
rostopic pub /controllers/chassis_controller/command rm_msgs/ChassisCmd "mode: 2
accel:
  linear: {x: 3.0, y: 3.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 3.0}
power_limit: 200.0
follow_source_frame: 'map'
command_source_frame: 'map'
stamp: {secs: 0, nsecs: 0}"
```

发布话题底盘线速度和旋转角速度

```shell
rostopic pub -r 50 /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0"
```

可以观察到以下现象：

![](/img/gazebo_chassis/gyro.gif)

#### 示例 2： 底盘的扭腰模式

设置底盘模式为`TWIST`，并设置好加速度

```shell
rostopic pub /controllers/chassis_controller/command rm_msgs/ChassisCmd "mode: 3
accel:
  linear: {x: 3.0, y: 3.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 3.0}
power_limit: 200.0
follow_source_frame: 'odom'
command_source_frame: 'odom'
stamp: {secs: 0, nsecs: 0}"
```

可以观察到以下现象：

![](/img/gazebo_chassis/twist.gif)
