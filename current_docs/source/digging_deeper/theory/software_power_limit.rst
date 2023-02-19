软件底盘功率限制
=================

软件功率限制通过限制电机输出力矩来控制底盘功率，以达到在底盘剧烈运动、实际功率即将超过给定的最大功率时，将实际功率限制到最大功率附近以下的目的。

理论
-----------------------
.. image:: ../../images/digging_deeper/theory/software_power_limit/software_power_limit_1.png

实现
-----------------------
.. image:: ../../images/digging_deeper/theory/software_power_limit/software_power_limit_2.png

k_1、k_2 的调试方法：实时读取裁判系统反馈的底盘功率。先让底盘电机堵转，调整 $k_1$ 使底盘实际功率大致与限制功率相等。再让机器人原地小陀螺，调整 $k_2$使底盘实际功率大致与限制功率相等。

具体的代码实现请点
`这里 <https://github.com/rm-controls/rm_controllers/blob/e6774fee52cd831f169ba35a598111b62e54c149/rm_chassis_controllers/src/chassis_base.cpp#L334-L359>`_.
下图展示了步兵机器人在 静止状态下进入高速小陀螺后再进入慢速小陀螺最后在高速小陀螺状态下进行平移的功率（最上）、速度指令（中间）和实际速度（最下）的曲线，可见在高速小陀螺状态下，功率被限制在了红线附近之下，实际速度比速度指令小，进入慢速小陀螺后，功率降低，此时实际速度与速度指令相等，最后在高速小陀螺状态下平移时，功率依然被限制在了青色线(最大功率)附近以下。

.. image:: ../../images/digging_deeper/theory/software_power_limit/software_power_limit_3.png
