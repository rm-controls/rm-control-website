# rm_base模块

## 模块介绍

机器人驱动模块是连接各底层插件与电机的桥梁，它的主要功能是从ros参数服务器中获得电机参数信息和插件信息，完成参数和插件的加载，然后以1000Hz的频率接受、处理can总线上的数据，再通过can总线实现对电机的控制。

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

在该模块的核心运行节点`rm_base`中，创建所需模块的插件（如底盘、云台）并加载参数到ros参数服务器后，即可正常执行通信任务。



+ 初始化：从参数服务器中获取joint、IMU、GPIO和plugins的信息，并加载到blackboard里保存。



+ 数据接收：从can总线上获取数据并解码，更新blackboard里面的内容。



+ 数据发送：从blackboard取出数据进行编码，然后发送到can总线上。


## 编译与运行

### 编译

在ROS的工作区内编译

```shell
catkin_make  rm_base rm_base 
```

### 运行

执行roborts_base_node节点

```shell
rosrun rm_base rm_base
```





