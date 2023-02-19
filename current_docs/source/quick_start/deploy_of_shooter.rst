rm_shooter_controller部署流程
==================================

shooter的urdf编写
----------------------------------------------

trigger_joint的力矩、速度
+++++++++++++++++++++++++++++++++++++++

确定最大力矩、速度
__________________________________

拨弹轮的最大转速=电机的转速/传动比
拨弹轮的最大力矩=电机的扭矩*传动比
电机的功率P=FV
减速比可以使电机在相同功率下减小输出的转速，进而输出更大的力矩

在urdf中实现
__________________________________
dynamicx示例:
::

 <limit effort="3.0" velocity="43.54" lower="-1e16" upper="1e16"/>


电机的传动比(拨弹:电机）
+++++++++++++++++++++++++++++++++++++++

数据获取
_________________

传动比不同兵种、同个兵种不同赛季都可能会有不同，这项数据需要向机械组要，要注意正负号，传动比的正负会影响转动的方向
22赛季dynamicx各兵种传动比:

 英雄：4:1

 哨兵：1:1

 步兵：2.5:1

 无人机：2.5:1

在urdf中实现
_________________

dyanmicx示例:
::

 <actuator name="trigger_joint_motor">
            <mechanicalReduction>-2.5</mechanicalReduction>


config配置
----------------------------------------------
shooter_controller中的重要参数
+++++++++++++++++++++++++++++++++++++++
push_per_rotation:拨弹轮每旋转一圈发射的子弹数
每个拨弹轮都不同，这个参数需要实时调整

block_effort:判断拨弹轮阻塞的力矩
扳机的实时力矩受到电机和其传动比控制，根据block_effort=电机堵转扭矩*传动比可以进行初步估算

qd_10、qd_15、qd_20
不同子弹速度下的摩擦轮的角速度，qd后面的是对应的子弹速度

电机id、type
+++++++++++++++++++++++++++++++++++++++

在rm_config找到rm_hw中对应机器人的配置文件填写电机id、type
观察电调闪烁情况，闪烁n次，对应的电机id尾号便设为n
::

 id: 0x201

与实车上电机类型对应
::

 type: rm_3508


实车调试
----------------------------------------------
1.首先确保你的环境变量与当前调试的机器人类型相同
::

 sudo vim .bashrc

在你的bashrc文件中将ROBOT_TYPE设置为你需要调试的机器人类型

2.usb2can连接或远程连接nuc

3.运行rm_hw.launch
::

 mon launch rm_config rm_hw.launch

4.加载shooter_controller
::

 mon launch rm_config load_controllers.launch

5.运行rqt
::

 rqt

6.在rqt中发布指令

打开rqt后左上角点击Plugins，找到Robot Tools中的Controller Manager打开，如果发现rqt中没有Controller Manager，手动安装一下，运行命令:
::

 sudo apt-get install ros-noetic-rqt-controller-manager

然后打开Robot_State_Controller和Joint_State_Controller再打开Shooter_Controller，打开rqt左上角Plugins找到Topics中的Message Publisher，点击打开在其中找到需要的话题发布指令:

mode设置为0对应STOP状态，1对应READY状态，2对应PUSH状态
speed、hz根据实际情况调整即可

