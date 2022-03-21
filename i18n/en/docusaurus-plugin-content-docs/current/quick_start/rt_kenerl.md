# System modification

RT-Preempt Patch is based on the Linux community kernel, plus related patches to enable Linux to meet hard real-time requirements. The following is the process of compiling and configuring RT linux kernel, taking kernel 5.6.19 as an example.

## Download kernel and rt patch

1. Create a new folder to store the kernel and patches

   ```bash
   mkdir ~/rt-kernel && cd ~/rt-kernel
   ```

   > [!Tip]
   >
   > Use VPN to access, if there is no VPN, use mobile hotspot to access.

2. Download [rt patch](https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/)

3. Download [Kernel source code](https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/)

   > [!Warning]
   >
   > The kernel version and the patch version need to correspond strictly.

4. Patch

   ```bash
   sudo apt-get install libncurses-dev #Install dependencies
   tar -xzvf linux-5.6.19.tar.gz #Unzip the kernel
   gunzip patch-5.6.19-rt12.patch.gz #Unzip the patch
   cd linux-5.6.19/
   patch -p1 < ../patch-5.6.19-rt12.patch #Patch
   ```

   > [!Note]
   >
   > The kernel used in this article is linux-5.6.19.tar.gz, and the rt patch is patch-5.6.19-rt12.patch.gz.

   ## Configure the kernel

5. Open the kernel configuration interface

   ```bash
   make menuconfig
   ```

6. Select 'General setup', if the kernel version is older and there is no option in the next step, select 'Processor Type and features'
   ![image1](https://ftp.bmp.ovh/imgs/2020/10/489e6a9ff0a684f1.png)

7. Select 'Preemption Model (Voluntary Kernel Preemption (Desktop))'
   ![image2](https://ftp.bmp.ovh/imgs/2020/10/1b18aa2359246159.png)

8. Select 'Fully Preemptible Kernel (RT)', then press 'esc' to return to the main page
   ![image3](https://ftp.bmp.ovh/imgs/2020/10/66924a6b92b55753.png)

9. Select 'Kernel hacking'
   ![image4](https://ftp.bmp.ovh/imgs/2020/10/e1c825922419dbb8.png)

10. Select 'Memory Debugging'
    ![image5](https://ftp.bmp.ovh/imgs/2020/10/4b59c4383bb00e15.png)

11. Deselect 'Check for stack overflows', there is no choice to ignore

## Kernel compilation

1. Compile and install the kernel

   ```bash
   CONFIG_DEBUG_INFO=n #Prevent compilation of debug files
   make -j`nproc` && make -j`nproc` bindeb-pkg #Compile and package
   ```

   > [!Tip]
   >
   > 'nproc' is the number of cpu threads, use the nproc command to view.For example, if the cpu is 4 threads, then make -j'nproc'=make -j4

   Then you will get

   ```bash
   linux-firmware-image-5.6.19-rt12_5.6.19-rt12-1_amd64.deb
   linux-headers-5.6.19-rt12_5.6.19-rt12-1_amd64.deb
   linux-image-5.6.19-rt12_5.6.19-rt12-1_amd64.deb
   linux-libc-dev_5.6.19-rt12-1_amd64.deb
   ```

   > [!Tip]
   >
   > At this time, you can copy the .deb package to other devices for installation with a USB flash drive, and no need to compile

   Go to the package folder and install the kernel

   ```bash
   sudo dpkg -i linux-*.deb
   ```

2. Update grub and reboot

   ```bash
   sudo update-grub
   sudo reboot
   ```

3. View kernel version

   ```bash
   uname -a
   ```

   At this point, you can see that there is a'PREEMPT RT' logo in the kernel version

## Error collection

1. Unable to open the kernel configuration interface 'menuconfig'

   Q1:（linux-4.17.2 kernel as an example）

   ```bash
   root@simon-virtual-machine:/home/simon/Src/linux-4.17.2# make menuconfig
   YACC scripts/kconfig/zconf.tab.c
   /bin/sh: 1: bison: not found
   scripts/Makefile.lib:196: recipe for target 'scripts/kconfig/zconf.tab.c' failed
   make[1]: *** [scripts/kconfig/zconf.tab.c] Error 127
   Makefile:528: recipe for target 'menuconfig' failed
   make: *** [menuconfig] Error 2
   ```

   A1：

   ```bash
   apt-get install bison -y
   ```

   Q2：

   ```bash
   root@simon-virtual-machine:/home/simon/Src/linux-4.17.2# make menuconfig
   YACC scripts/kconfig/zconf.tab.c
   LEX scripts/kconfig/zconf.lex.c
   /bin/sh: 1: flex: not found
   scripts/Makefile.lib:188: recipe for target 'scripts/kconfig/zconf.lex.c' failed
   make[1]: *** [scripts/kconfig/zconf.lex.c] Error 127
   Makefile:528: recipe for target
   ```

   A2：

   ```bash
   sudo apt-get install flex
   ```
