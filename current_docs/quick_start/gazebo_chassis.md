---
id: gazebo_chassis
sidebar_position: 2
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

:::danger
从包安装可能无法使用。，本文还未完成。
:::
# Gazebo 底盘

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
 mon launch rm_gazebo empty_world.launch
```

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

设置底盘的运行模式，并且给其设定一个加速度的指令如下：

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
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```
