生成IKFast插件
============

本文档将介绍如何为MoveIt配置IKFast插件。

IKFast简介
---------------

MoveIt提供了一些工具，可以使用OpenRAVE生成的cpp文件为MoveIt生成IKFast运动学插件。本教程将逐步通过设置机器人，以利用IKFast。MoveIt-IKFast在ROS-Noetic上进行了6自由度和7自由度机械臂的测试。虽然它在理论上是可行的，但MoveIt-IKFast目前不支持>7个自由度的手臂。

准备
---------------
如没有看入门文档，先看一下 `这个 <https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html>`_.

用Ubuntu 14.04时的docker镜像: ::

    sudo apt-get install docker.io
    sudo service docker start

以下命令将确保您可以使用您的用户帐户运行docker（将$user添加到docker组）: ::

 sudo usermod -a -G docker $USER

下载解算库: ::

 sudo apt-get install ros-noetic-moveit-kinematics


创建创建IKFast MoveIt插件
---------------------------------

在rm_engineer文件夹下生成urdf格式（需删掉已生成的插件）: ::

 rosrun xacro xacro -o engineer.urdf ../rm_description/urdf/engineer/engineer.urdf.xacro


运行auto_create_ikfast_moveit_plugin.sh: ::

 rosrun moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype TranslationDirection5D engineer.urdf arm base_link link5

参数：

arm
 生成IKFast解算器“规划组”的名字，可以和MoveIt Setup Assistant导出时的不一样
TranslationDirection5D
 目前机械臂是五轴串联的，`这里 <http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types>`_ 有详细介绍
base_link
 初始关节
link5
 结束关节

**注：从 auto_create_ikfast_moveit_plugin.sh 中可以了解到参数设置**

生成最后如果提示找不到gineer_moveit_config,没关系


生成后编译代码: ::

 catkin build

 sourse 工作空间的setup.bash

修改engineer_arm_config包下的把 kinematics.yaml文件 的 **kinematics_solver:** 为插件包XX_moveit_ikfast_plugin_description.xml文件下的解算插件名


创建过程简介
-----------

从/opt/ros/noetic/lib/moveit_kinematics包下的
auto_create_ikfast_moveit_plugin.sh中能看出他具体大概有以下几个步骤：

1. 下载openrave 和sympy等的 docker镜像
2. 运行 collada_urdf 生成.dae文件
3. 运行  openrave0.9.py
4. 运行  create_ikfast_moveit_plugin.py

*如果有锅，看卡在哪里。*

 [`参考1`_]
.. _参考1: https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html#tweaking-the-creation-process

 [`参考2`_]
.. _参考2: http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types

 [`参考3`_]
.. _参考3: https://blog.csdn.net/Kalenee/article/details/80740258

:Authors:

- WenWei Ling<1351975866@qq.com>
- Qiayuan Liao<liaoqiayuan@gmail.com>
