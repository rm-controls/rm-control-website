# rm_base 模块

## 模块介绍

机器人驱动模块是连接各底层插件与电机的桥梁，它的主要功能是从 ros 参数服务器中获得电机参数信息和插件信息，完成参数和插件的加载，然后以 1000Hz 的频率接受、处理 can 总线上的数据，再通过 can 总线实现对电机的控制。

模块文件目录如下所示

```bash
rm_base
├── CMakeLists.txt
├── include
│  ├── dbus_node.h
│  ├── referee_node.h
│  ├── rm_base.h
│  ├── base
│  │ ├───  blackboard.cpp
│  │ ├───  bus.cpp
├── src
│  ├── dbus_node.cpp
│  ├── referee_node.cpp
│  ├── rm_base.cpp
│  ├── rm_base_ndoe.cpp
│  ├── base
│  │ ├───  blackboard.cpp
│  │ ├───  bus.cpp
└── package.xml
```

## 代码原理

在该模块的核心运行节点`rm_base`中，创建所需模块的插件（如底盘、云台）并加载参数到 ros 参数服务器后，即可正常执行通信任务。

- 初始化：从参数服务器中获取 joint、IMU、GPIO 和 plugins 的信息，并加载到 blackboard 里保存。

- 数据接收：从 can 总线上获取数据并解码，更新 blackboard 里面的内容。

- 数据发送：从 blackboard 取出数据进行编码，然后发送到 can 总线上。

## 编译与运行

### 编译

在 ROS 的工作区内编译

```shell
catkin_make  rm_base rm_base
```

### 运行

执行 roborts_base_node 节点

```shell
rosrun rm_base rm_base
```
