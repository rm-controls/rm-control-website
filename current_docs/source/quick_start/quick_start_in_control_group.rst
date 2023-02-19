====
控制组快速入门（从入门到退队）
====

nuc基本配置
====

安装Ubuntu服务器版
----

插入装有ubuntu服务器版的u盘，进入u盘启动，正常安装即可。给/swap分配16g，/efi分配512m，剩下的硬盘空间全部挂载到/目录下。设备名看这台nuc给哪个机器人用，用户名dynamicx，密码dynamicx。安装完成后按照提示移除u盘，重启。

安装ssh，并通过ssh用你的电脑操控nuc
----

1、将自己的电脑连接上wifi，然后用一根网线将nuc和你的电脑连接。这时你的电脑左上方会出现有线连接的图标。进入有线连接，点击设置，点击IPV4,设置“与其他计算机共享网络”（类似的意思）。这样nuc就能上网了。可用`ping baidu.com`来检查是否已联网。

2、安装ssh服务器端
::
    sudo apt install openssh-server

**如需要安装依赖，按提示安装**

3、在nuc上使用`ip a`查看nuc的ip地址。在你的电脑上输入指令`ssh dynamicx@“nuc的ip地址”`，这样你就能在你的电脑上操控nuc了。

换源
----

网上搜索国内的源，例如清华源，阿里源，换源。网上的清华源有的能用，有的是坏的，如果装清华源锅了就换一个。

安装easywifi
----

1、在github上搜索easywifi,第一个就是。将源代码clone下来，注意要用http。

2、安装easywifi依赖：
::
    sudo apt-get install network-manager-config-connectivity-ubuntu

3、进入easywifi文件夹，输入
::
    sudo python3 easywifi.py

4、成功运行easywifi，运行  *1*<!--Scan for networks-->  搜索wifi，然后运行  *5*<!--Setup new network-->  输入wifi名称和密码，让nuc连上wifi。

5、和nuc连上同一个wifi，继续用ssh操控nuc。

安装ros
----

安装ros请主要参考ros_wiki上的安装教程。请注意安装base而不是full-desktop。

安装catkin tools
^^^^

catkin tools官方文档：https://catkin-tools.readthedocs.io/en/latest/
如果你使用catkin build时需要你安装osrf-pycommon>0.1.1这个依赖，请输入以下指令：
::
    sudo apt-get install python3-pip
    pip3 install osrf-pycommon

rosdep
^^^^

rosdep update 失败的参考解决方法：https://github.com/SparkChen927/rosdep
::
    git clone https://gitclone.com/github.com/SparkChen927/rosdep.git

为机器人安装软件包
----

我们团队为 **rm_control** 和 **rm_controllers** 搭建了软件源，请根据 `这个网站 <https://rm-control-docs.netlify.app/quick_start/rm_source>`_ 给nuc添加软件源并把相应的软件包拉下来。

优化
----

1、你会发现开机很慢，这是一个系统服务导致的，可以设置将其跳过。
::
    $ sudo vim /etc/netplan/01-netcfg.yaml`
注：这个文件可能不叫这个名字，可能需要转到/etc/netplan这个目录下看看。

在网卡的下一级目录中增加
::
    optional: true

修改完后生效设置
::
    $ sudo netplan apply

2、阻止nuc休眠

nuc长时间不用会休眠，会给工作带来一定麻烦。因此需要设置阻止nuc休眠。输入以下指令：
::
    sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

换内核
----

1、使用搜索引擎搜索xanmod，通常搜索结果第一个就是，打开 `此网站 <https://xanmod.org>`_ 。

2、我们需要更换一个实时性更强的内核，这样的内核通常名字里会带有“rt”（realtime）。在这个网站往下拉会看到“Install via Terminal”(通过命令行安装)。根据提示安装自己想要的内核。

3、使用指令 ``sudo dpkg --get-selections | grep linux-image`` 来查看你想要安装的内核是否安装成功。

4、重启，按F2进入BIOS模式。在boot->Boot Priority勾选Fast boot。Power选项里勾选Max Performance Enabled,Dynamic Power Technology设为最长的那个，Power->Secondary Power Settings将After Power Failure设为Power on。cooling选项里将Fan Control Mode设置为Fixed，Fixed Duty Cycle设为100。***关闭安全启动***然后退出BIOS，正常启动。

5、测试新内核的实时性和can总线传输速率

P.S:如果进不了BIOS可尝试长按开机键直至指示灯变成橙色。

关于Clion的远程传输配置
====

免密登陆设置
----

当我们远程连接NUC的时候，需要输入密码，但是这样的话可能会比较麻烦，因此最好我们配置免密登陆，可以省去很多麻烦

命令行操作：
::
    ssh-copy-id dynamicx@host

回车之后还需要输入一次密码，输入完之后就可以了，以后每次都可以免密登陆

配置CLion流程
----

首先在setting处的CMake一栏配置，在原有的debug下面点击添加符号，并命名为remote

.. image:: /images/quick_start/quick_start_in_control_group/clion_setting.png

然后设置remote中的CMake options为
::
    -DCATKIN_DEVEL_PERFIX=../remote_devel

设置Build directory为
::
    ./remote_build

操作完之后CMake就配好了，配置好CMake之后，就会在工作空间下的src中生成remote_build这个文件，这里是远程主机中间生生成的编译文件

完成配置之后，如果在Build菜单栏中选择

Build Project则是本地以及远程主机中的包一起编译，

Build all Debug则是只编译本地的文件

接下来还需要配置deployment

配置deployment流程
^^^^

部署远程文件地址和本地文件地址（能够在本地文件和远程文件进行同步）。

root path：远程文件的根路径
deployment path：远程文件相对于根路径的相对路径
local path：本地文件的路径

首先，在deployment处点击添加符号，文件类型选择SFTP，并命名为standard4（命名取决于是什么机器人，比如standard4是四号步兵），接下来是配置connection

点击SSH configuration处的省略号配置SSH如图

.. image:: /images/quick_start/quick_start_in_control_group/ssh_configuration1.png


然后同样点击添加符号，开始配置相关信息

如图

.. image:: /images/quick_start/quick_start_in_control_group/ssh_configuration2.png


Host处填写的是你需要远程连接的服务器的ip，用户名和密码都是队名，如图配置好之后点击Apply应用，之后点击OK，

接下来在配置Mapping，如图

.. image:: /images/quick_start/quick_start_in_control_group/mapping.png


现在这一步是把本地的路径映射到目标主机上去

本地路径分别是：/home/username/工作空间名/src             对应的deployment path为/home/dynamicx/rm_ws/src（固定的）

/home/username/工作空间名/src/remote_devel/lib         对应的deployment path为/home/dynamicx/rm_ws/src/devel/lib

/home/username/工作空间名/src/remote_devel/include       对应的deployment path为/home/dynamicx/rm_ws/devel/include

配置好之后应用再点击OK

最后一步是配置Excluded path

同样是点击添加符号，然后输入路径为/home/username/工作空间名/src/remote_build

然后配置好SSH之后退出到deployment界面，同样点击Apply，再点击OK，完成上述流程之后就完全配置好了，如果需要进行远程传输，则进行如下

流程

远程传输流程
----

在CLion中的工作空间中，点击src，选择你想要进行远程传输的功能包，鼠标右击，选择菜单栏中的deployment，如果需要从本地传输文件到远程主机则选择upload to “目标主机”，如果需要从目标主机中传输到本地，则选择download from “目标主机”

can总线的连接
====

**初始化can**
::

    sudo ip link set can的编号 up type can bitrate 1000000

**检查can是否已连接**
::
    candump can的编号`

**发指令**
::
    cansend can的编号 标识符#16位数字  `

标识符有两种，分别是1FF和200，两种标识符对应两组电机。16位数字每4位对应一个电机的电流大小，共四个电机，即使电机不足四个也要补满16位。由此可见一根can线最多挂载八个电机。


