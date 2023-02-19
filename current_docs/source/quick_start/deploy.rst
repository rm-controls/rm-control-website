代码部署
=================

快速测试
-----------------------

测试can
++++++++++++++++++++++++++

1. 初始化can0和can1

::

    sudo ip link set can0 up type can bitrate 1000000
    sudo ip link set can1 up type can bitrate 1000000


2. 监听can上的数据

::

    sudo apt-get install can-utils
    candump can0


Tip

 此时转动电机可监听到电机参数变化

3. 发送数据驱动电机

::

    cansend can0 200#05000000000000000

具体参见 `官方文档 <https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E4%B8%AD%E8%8B%B1%E6%97%A5%EF%BC%89V1.01.pdf>`_


修改串口物理地址
++++++++++++++++++++++++++

1. pc连接usbtoxxx.

2. 查询usbDbus和usbReferee物理地址

::

    ls /sys/class/tty/ttyUSB* -l


得到如下：

.. image:: ../images/quick_start/deploy/deploy_1.png

其中，'3-2.3:1.0'为usbDbus的物理地址

3. 修改usbDbus和usbReferee物理地址

::

    cd /home/chenzheng/RM-Software/rm_ws/src/rm_bringup/scripts/udev/
    vim rm.rules


修改KERNELS=="${usb物理地址}"

::

    ./create_udev_rules.sh


4. 查看是否映射成功

::

    ls /dev/ | grep usb


存在usbDbus和usbReferee即映射成功

Tip
 若找不到，重启；还是没有，换usb口重新执行上述步骤

运行代码
++++++++++++++++++++++++++

::

 roslaunch rm_bringup standard.launch


如果出现如下错误：

.. image:: ../images/quick_start/deploy/deploy_2.png

需要设置超级用户免密：

::

 sudo visudo


添加如下内容：

.. image:: ../images/quick_start/deploy/deploy_3.png

重新运行代码

实机部署
-----------------------

1.  配置实时内核，参考此 `文档 <../rt_kernel.rst>`_


2. 配置开机自动运行代码脚本

::

    cd /home/chenzheng/RM-Software/rm_ws/src/rm_bringup/scripts/auto_start/
    ./create_rm_start_service.sh


3. 调试

    参考快速测试