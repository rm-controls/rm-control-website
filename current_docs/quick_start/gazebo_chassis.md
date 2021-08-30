---
id: gazebo_chassis
sidebar_position: 1
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Gazebo 底盘

本文将带你在gazebo仿真中操控麦克纳姆轮以及舵轮底盘。

## 在仿真中生成机器人

### 创建包并安装依赖

进入你的工作空间（假设名为 `rm_ws`），在你的工作空间中拉取本次教程所要用到的仿真包 `rm_gazebo`，并使用 `rosdep` 安装包的依赖。

```shell
cd ~/ws_ws/src
git clone https://github.com/rm-controls/rm_control.git
rosdep install --from-paths . --ignore-src
catkin build
```

:::tip
必须确保你的 `rosdep` 安装和初始化是正确的。
:::

配置好环境变量为目标机器人,hero及其他车是麦克纳姆轮底盘，而standard4是舵轮底盘。
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
 mon launch rm_gazebo empty_world.launch
```

## 运行控制器并操控底盘

### 利用命令行去设置环境变量

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

加载底盘控制器

```
 mon launch rm_chassis_controllers load_controllers.launch 
```

设置底盘的运行模式，并且给其设定一个加速度的指令如下：

```
rostopic pub /controllers/chassis_controller/command rm_msgs/ChassisCmd "mode: 0
accel:
  linear: {x: 0.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}
power_limit: 0.0
follow_source_frame: ''
stamp: {secs: 0, nsecs: 0}" 
```

设置底盘速度，让其运动的指令如下：

```
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

### 利用工具Message Publisher控制底盘

在rqt工具v中找到Message Publisher并打开

找到话题/controllers/chassis_controller/command和/cmd_vel，并设置对应的数据

通过打勾去完成话题消息的发送。

