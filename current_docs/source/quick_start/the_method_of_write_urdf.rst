urdf编写流程
===============
在编写urdf前，需要对实车的结构有一定了解，没有实车时可通过图纸或者向设计者了解
在出urdf时，必须要和机械组的组员面对面出，将每一个link载入，在rviz中观察偏移量以及朝向位置有没有出现错误，最后再在gazebo中检查惯量，确保一个link的stl以及参数都没有问题再出下一个link，直到全部正确为止

urdf规范
--------------------

link
++++++++++++

每一个link需要包含visual、collision、inertial三个标签

visual
_______________

在visual标签中载入机械组给出的stl模型
示例:
::

    <visual>
       <origin xyz="0 0 0" rpy="0 0 0"/>
       <geometry>
           <mesh filename="package://rm_description/meshes/sentry/catapult.stl" scale="0.001 0.001 0.001"/>
       </geometry>
    </visual>

其中origin xyz rpy中可调整stl模型相对于xyz轴或rpy轴的偏移量，这个调整仅提供视觉上的偏移，不会对实际仿真有影响，所以一般不会进行修改

inertial
_______________

inertial标签中需要填入机械组给出的所属link部分的质量、质心以及惯性矩阵（要填入质心处的惯性矩阵）
示例:
::

    <inertial>
        <mass value="451.658e-3"/>
        <origin xyz="-10.414e-3 -18.637e-3 -4.243e-3"/>
        <inertia ixx="9.679e-4" ixy="-3.152e-4" ixz="-5857.572e-9" iyy="2.66e-3"
                 iyz="46492.352e-9" izz="2.71e-3"/>
    </inertial>

collision
_______________

在collision标签中载入机械组给出的stl模型，保证碰撞箱完全覆盖visual中载入的stl模型
示例:
::

    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <mesh filename="package://rm_description/meshes/sentry/catapult.stl" scale="0.001 0.001 0.001"/>
        </geometry>
    </collision>

joint
++++++++++++

urdf中每两个link需要一个joint进行连接，joint要有子link以及父link

joint类型
______________

joint常用的类型有平动以及转动，清楚joint的运动情况，选择正确的关节类型
转动关节:
continous 没有限位的转动关节，可以绕单轴无限旋转
revolute  有限位的转动关节，有一定的旋转角度限制
平动关节:
prismatic 沿某一轴滑动的关节
固定关节:
fixed     不允许运动的关节
示例:
::

 <joint name="drive_wheel_joint" type="revolute">

axis
_______________

设置关节围绕哪一个轴运动
::

 <axis xyz="0 1 0"/>

子link、父link
______________________________

示例:
::

    <parent link="base_link"/>
    <child link="catapult_link"/>

origin
_______________

joint中写入的偏移量是指子link的坐标系到父link坐标系的偏移量，这项数据需要机械组给出并且要测量准确
示例:
::

 <origin xyz="0.7e-3 -77e-3 -49.8e-3" rpy="0 0 0"/>


stl规范
--------------------

机械组给出的stl不能超过2M，在尽量保证美观的前提下删除不需要的部分
有电机操纵的部分（例如A）需要单独写成一个A的link，需要单独出一份A的stl（可以把电机当成joint来看）
跟随A运动的部分包含控制其运动的电机归属于A的link，其余部分归属于A的父link

数据规范
--------------------

机械所给出的stl属于高度简化的模型，但每一个link的质量、质心以及惯性矩阵是需要未简化的
填入的所有数据都要换算成国际单位

坐标系规范
--------------------

base_link的坐标系出在几何中心，其他link的坐标系出在与父link连接处
                    
