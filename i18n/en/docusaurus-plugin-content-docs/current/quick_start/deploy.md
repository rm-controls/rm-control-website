# 代码部署

## 快速测试

### 测试 can

1. 初始化 can0 和 can1

   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   sudo ip link set can1 up type can bitrate 1000000
   ```

2. 监听 can 上的数据

   ```bash
   sudo apt-get install can-utils
   candump can0
   ```

   > [!Tip]
   > 此时转动电机可监听到电机参数变化

3. 发送数据驱动电机

   ```bash
   cansend can0 200#05000000000000000
   ```

   具体参见[官方文档](https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E4%B8%AD%E8%8B%B1%E6%97%A5%EF%BC%89V1.01.pdf)

### 修改 can 物理地址

1. pc 连接 usb2xxx

2. 查询 can0 和 can1 物理地址

   ```bash
   ls /sys/class/tty/ttyUSB* -l
   ```

   得到如下：

   ![](https://ftp.bmp.ovh/imgs/2020/11/7f51b4bda7bb8037.png)

   其中，'3-2.3:1.0'为 can0 的物理地址

3. 修改 can0 和 can1 物理地址

   ```bash
   cd /home/chenzheng/RM-Software/rm_ws/src/rm_bringup/scripts/udev/
   vim rm.rules
   ```

   修改 KERNELS=="${can 物理地址}"

   ```bash
   ./create_udev_rules.sh
   ```

4. 查看是否映射成功

   ```bash
   ls /dev/ | grep usb
   ```

   存在 usbDbus 和 usbImu 即映射成功

   > [!Tip]
   > 若找不到，重启；还是没有，换 usb 口重新执行上述步骤

### 运行代码

```bash
roslaunch rm_bringup standard.launch
```

如果出现如下错误：

![](https://ftp.bmp.ovh/imgs/2020/11/06ff5fa7f0ca50a2.png)

需要设置超级用户免密：

```bash
sudo visudo
```

添加如下内容：

![](https://ftp.bmp.ovh/imgs/2020/11/912617950455359d.png)

重新运行代码

## 实机部署

1.  配置实时内核，参考此[文档](dev_guide/rt_kernel.md)

2.  配置开机自动运行代码脚本

    ```bash
    cd /home/chenzheng/RM-Software/rm_ws/src/rm_bringup/scripts/auto_start/
    ./create_rm_start_service.sh
    ```

3.  调试

    参考快速测试
