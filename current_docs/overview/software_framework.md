# 软件框架
程序由多个ROS package构成，其中有元包： `rm_control` 提供了底层硬件和仿真的通用接口，元包 `rm_controllers` 则为各常用模块控制器，还有负责普通机器人操作的包 `rm_manual` 和哨兵决策包 `rm_fsm`。
* rm_control
  * rm_msgs: 自定义的 ROS 话题消息、服务、动作
  * rm_common: 常用函数、算法、裁判系统收发、ui
  * rm_description: 所有机器人的 URDF，定义了：机器人各坐标系关系、电机与关节的映射、关节的限位、仿真需要的物理属性
  * rm_hw: 同名节点通过 SocketCAN 与执行器进行通信获取数据并发送指令，在实车运行时提供硬件接口给控制器
  * rm_gazebo: 同名Gazebo Plugin在仿真运行时提供硬件接口给控制器
* rm_controllers
  * robot_state_controller: robot_state_publisher 的高性能版，高频维护 tf
  * rm_chassis_controllers: 麦克纳姆轮、舵轮、平衡车的底盘控制器
  * rm_gimbal_controllers: 有射击模型和跟踪滤波预测的云台控制器
  * rm_shooter_controllers: 操作摩擦轮，拨弹盘完成发射控制器的控制器
  * rm_calibration_controllers: 校准执行器位置的控制器
* rm_dbus: dbus数据接收节点
* rm_detection: 装甲板和风车视觉识别
* rm_stone: 矿石和障碍块视觉识别
* rm_manual: 普通机器人决策
* rm_fsm: 哨兵机器人决策

![diagram_rm_controls](/img/software_framework/rm-controls-diagram.png)