---
id: hardware_specifications
sidebar_position: 2
---

# 硬件规格

## CAN 总线

### 內置 CAN

Jetson AGX Xavier/NX/TX2 和 妙算 2-G（TX2）都具有两个 build-in 的 CAN 总线接口。下图中 **9** 展示了 妙算 2 的内置 CAN 的物理接口位置。

![manifold2](/img/hardware_specifications/manifold2.png)

下图展示了自制的 Jetson AGX Xavier 接线板。

![jeston_agx](/img/hardware_specifications/jeston_agx.jpg)

上述的方式都可以很方便地通过 Linux 主线提供的 SocketCAN 机制使用 CAN 接口。

### USB 转 CAN

我们还给 Intel NUC 和队员的调试电脑开发了 [USB 转 CAN 模块](https://github.com/rm-controls/rm_usb2can) ，该模块的驱动已经被包含进 Linux 主线，仅需插上就可以通过 SocketCAN 访问 CAN。
下图展示了：使用 NUC [主板上的 USB 1.25mm 端子](https://www.intel.com/content/www/us/en/support/articles/000006933/intel-nuc.html) 稳定连接 USB 转 CAN 模块。
![Intel NUC with usb2can](/img/hardware_specifications/nuc_with_usb2can.jpg)

## 其他接口

GPIO、PWM、I2C 接口正在开发中，详见 [road map](TODO)。

## 模块拓扑

下图展示的是整车基本硬件拓扑框图（工程机器人整车拓扑的差异较大，在此不给出）。

![hardware_typology_schematic](/img/hardware_specifications/hardware_typology_schematic.png)

机器人基本硬件拓扑结构是以云台和底盘两大部分组成的，这两个部分以导电滑环为分割线隔开。车上的 12V、19V、5V、3.3V 电压组分别由 LM25116、LM3150、SY8303、RT9193 提供。
