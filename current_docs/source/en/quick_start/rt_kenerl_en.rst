System modification
==================================

RT-Preempt Patch is based on the Linux community kernel, plus related patches to enable Linux to meet hard real-time requirements. The following is the process of compiling and configuring RT linux kernel, taking kernel 5.6.19 as an example.

Download kernel and rt patch
--------------------------------------------------------
1. Create a new folder to store the kernel and patches
::

    mkdir ~/rt-kernel && cd ~/rt-kernel


Tip

 Use VPN to access, if there is no VPN, use mobile hotspot to access.

2. Download
`rt patch <https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/>`_

3. Download
`Kernel source code <https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/>`_


Warning
 The kernel version and the patch version need to correspond strictly.

4. Patch
::

    sudo apt-get install libncurses-dev #Install dependencies
    tar -xzvf linux-5.6.19.tar.gz #Unzip the kernel
    gunzip patch-5.6.19-rt12.patch.gz #Unzip the patch
    cd linux-5.6.19/
    patch -p1 < ../patch-5.6.19-rt12.patch #Patch

Note
 The kernel used in this article is linux-5.6.19.tar.gz, and the rt patch is patch-5.6.19-rt12.patch.gz.

Configure the kernel
--------------------------
1. Open the kernel configuration interface
::

    make menuconfig


2. Select 'General setup', if the kernel version is older and there is no option in the next step, select 'Processor Type and features'

.. image:: ../../images/en/quick_start/rt_kernel_en/rt_kernel_en_1.png

3. Select 'Preemption Model (Voluntary Kernel Preemption (Desktop))'

.. image:: ../../images/en/quick_start/rt_kernel_en/rt_kernel_en_2.png

4. Select 'Fully Preemptible Kernel (RT)', then press 'esc' to return to the main page

.. image:: ../../images/en/quick_start/rt_kernel_en/rt_kernel_en_3.png

5. Select 'Kernel hacking'

.. image:: ../../images/en/quick_start/rt_kernel_en/rt_kernel_en_4.png

6. Select 'Memory Debugging'

.. image:: ../../images/en/quick_start/rt_kernel_en/rt_kernel_en_5.png

7. Deselect 'Check for stack overflows', there is no choice to ignore

Kernel compilation
----------------------------------------------
1. Compile and install the kernel
::

    CONFIG_DEBUG_INFO=n #Prevent compilation of debug files
    make -j`nproc` && make -j`nproc` bindeb-pkg #Compile and package


Tip
 'nproc' is the number of cpu threads, use the nproc command to view.For example, if the cpu is 4 threads, then make -j'nproc'=make -j4

Then you will get

::

    linux-firmware-image-5.6.19-rt12_5.6.19-rt12-1_amd64.deb
    linux-headers-5.6.19-rt12_5.6.19-rt12-1_amd64.deb
    linux-image-5.6.19-rt12_5.6.19-rt12-1_amd64.deb
    linux-libc-dev_5.6.19-rt12-1_amd64.deb


Tip
  At this time, you can copy the .deb package to other devices for installation with a USB flash drive, and no need to compile

Go to the package folder and install the kernel
::

    sudo dpkg -i linux-*.deb


2. Update grub and reboot
::

    sudo update-grub
    sudo reboot


3. View kernel version
::

    uname -a

    
At this point, you can see that there is a'PREEMPT RT' logo in the kernel version

Error collection
----------------------------------------------
1. Unable to open the kernel configuration interface 'menuconfig'

Q1:（linux-4.17.2 kernel as an example）
::

    root@simon-virtual-machine:/home/simon/Src/linux-4.17.2# make menuconfig
    YACC scripts/kconfig/zconf.tab.c
    /bin/sh: 1: bison: not found
    scripts/Makefile.lib:196: recipe for target 'scripts/kconfig/zconf.tab.c' failed
    make[1]: *** [scripts/kconfig/zconf.tab.c] Error 127
    Makefile:528: recipe for target 'menuconfig' failed
    make: *** [menuconfig] Error 2

A1：
::

    apt-get install bison -y

Q2：
::

    root@simon-virtual-machine:/home/simon/Src/linux-4.17.2# make menuconfig
    YACC scripts/kconfig/zconf.tab.c
    LEX scripts/kconfig/zconf.lex.c
    /bin/sh: 1: flex: not found
    scripts/Makefile.lib:188: recipe for target 'scripts/kconfig/zconf.lex.c' failed
    make[1]: *** [scripts/kconfig/zconf.lex.c] Error 127
    Makefile:528: recipe for target

A2：
::

    sudo apt-get install flex
