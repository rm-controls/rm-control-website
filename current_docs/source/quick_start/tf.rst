验证tf
============

本文档将介绍tf查询的机制

准备
---------------

如对 tf 不是很了解，可以看一下 `这个 <http://wiki.ros.org/tf>`_.

目的
---------------

假设我们目前有三个joint，分别是map、yaw、pitch，让map2yaw，yaw2pitch在不同的时刻向tf树中发送数据，在tf树中lookup map2pitch，查看map2pitch的时刻和位置，以此来探究在ros::Time(0)时刻下lookup到的时间和位置。

验证过程
---------------

第一步：创建tf树: ::

    map2yaw.transform.translation.x = 1.0;
    map2yaw.transform.rotation.w = 1.0;
    map2yaw.header.stamp = ros::Time().fromSec(1.0);
    map2yaw.header.frame_id = "map";
    map2yaw.child_frame_id = "yaw";
    tf_Buffer.setTransform(map2yaw, "test");

    yaw2pitch.transform.translation.x = 1.0;
    yaw2pitch.transform.rotation.w = 1.0;
    yaw2pitch.header.stamp = ros::Time().fromSec(1.0);
    yaw2pitch.header.frame_id = "yaw";
    yaw2pitch.child_frame_id = "pitch";
    tf_Buffer.setTransform(yaw2pitch, "test");

第二步：lookup map2pitch并打印相关数值: ::

    try {
    map2pitch = tf_Buffer.lookupTransform("map", "pitch", ros::Time(0));
    }
    catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }

    double time = map2pitch.header.stamp.toSec();
    ROS_INFO("tf_stamp is %f", time);
    ROS_INFO("translation.x is %f", map2pitch.transform.translation.x);

结果: ::

    tf_stamp is 1.0
    translation.x is 2.0

第三步：为map2yaw更新时间和位置，获取map2pitch的时间的位置: ::

    map2yaw.transform.translation.x = 3.0;
    map2yaw.transform.rotation.w = 1.0;
    map2yaw.header.stamp = ros::Time().fromSec(3.0);

结果: ::

    tf_stamp is 1.0
    translation.x is 2.0

结果分析：map2pitch的时间和位置并没有因为map2yaw的更新而改变，依旧获取到了tf树刚建立时的数据

第四步：为yaw2pitch更新时间和位置，获取map2pitch的时间的位置: ::

    yaw2pitch.transform.translation.x = 4.0;
    yaw2pitch.transform.rotation.w = 1.0;
    yaw2pitch.header.stamp = ros::Time().fromSec(4.0);

结果: ::

    tf_stamp is 3.0
    translation.x is 6.0

结果分析：此时时刻和位置均改变，时刻是3.0，可以联想到map2yaw在3.0时刻向tf树更新了数据，此时获取到的位置应该也是3.0时刻的位置，map2yaw在stamp = 3.0时刻的位置是已知的，而yaw2pitch在stamp = 3.0 的translation.x 的位置是通过stamp = 1.0和stamp = 4.0时刻的位置来计算获得的。

结论：
>>>>>>>>

在tf树建立之后，同一tf的两次更新之间的任意时刻都是存在tf的，每次ros::Time(0)获取到的数值是能拿到的最新的值,任意时刻的位置都是通过最近两次维护的时间差和位置差计算来自动更新的。如第四步中，能拿到的最新的值是stamp = 3.0时刻的。

:Authors:

- Zhengying Yan<1697203829@qq.com>
- Qiayuan Liao<liaoqiayuan@gmail.com>
