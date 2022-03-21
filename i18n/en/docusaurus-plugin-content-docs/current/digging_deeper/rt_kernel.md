---
id: rt_kernel
sidebar_position: 3
---

# Real-time kernel compilation

The [RT-Preempt](https://rt.wiki.kernel.org/index.php/Main_Page) is based on the Linux community kernel, with relevant patches to make Linux meet hard real-time requirements. Here is the compilation and configuration process, using kernel `5.6.19` as an example.

## Download kernel and rt patches

1. Create a new folder for the kernel and patches

``bash
mkdir ~/rt-kernel && cd ~/rt-kernel

````

:::tip
Use extranet access, or use mobile hotspot access if no extranet is available.
:::

2. download [rt patch](https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/)

3. download [kernel source code](https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/)


:::caution
Kernel version and patch version need to strictly correspond
:::

4. apply the patch

```shell
 sudo apt-get install libncurses-dev #Install the dependencies
 tar -xzvf linux-5.6.19.tar.gz #Unpack the kernel
 gunzip patch-5.6.19-rt12.patch.gz #Unzip the patch
 cd linux-5.6.19/
 patch -p1 < ... /patch-5.6.19-rt12.patch #patch
````

:::info
The kernel used in this article is `linux-5.6.19.tar.gz` and the rt patch is `patch-5.6.19-rt12.patch.gz`.
:::

## Configure the kernel

1. Open the kernel configuration screen

```bash
make menuconfig
```

2. Select General setup, and if the kernel version is old and does not have the options in the next step, select Processor Type and features

If the kernel version is a bit older, then select the Processor Type and features. [Figure 1](https://ftp.bmp.ovh/imgs/2020/10/489e6a9ff0a684f1.png)

3. select Preemption Model (Voluntary Kernel Preemption (Desktop))

![Figure 2](https://ftp.bmp.ovh/imgs/2020/10/1b18aa2359246159.png)

4. Select Fully Preemptible Kernel (RT), then press esc all the way back to the main page

![Figure 3](https://ftp.bmp.ovh/imgs/2020/10/66924a6b92b55753.png)

5. Select Kernel hacking

![Figure 4](https://ftp.bmp.ovh/imgs/2020/10/e1c825922419dbb8.png)

6. Select Memory Debugging

![Figure 5](https://ftp.bmp.ovh/imgs/2020/10/4b59c4383bb00e15.png)

7. Deselect Check for stack overflows, you can ignore it if it is not selected.

8. Press '/' to search for DEBUG_INFO

![Figure 6](https://ftp.bmp.ovh/imgs/2020/11/0fe2f71cd666f178.png)

9. Press '1'

![Figure 7](https://ftp.bmp.ovh/imgs/2020/11/94f53ecb38a69642.png)

10. Press 'n' on the Compile the kernel with debug info option to cancel the generation of debug files during compilation

! Figure 8](https://ftp.bmp.ovh/imgs/2020/11/f90a6d57f2800bf1.png)

:::tip

Compiling the kernel generates a very large debug file, which is not needed for the actual installation, so you can simply prevent it from being generated

:::

## Kernel compilation

1. Compile and install the kernel

```bash
CONFIG_DEBUG_INFO=n #Block compilation of debug files
make -j`nproc` && make -j`nproc` bindeb-pkg #compile and package
```

:::tip
'nproc' is the number of CPU threads.
:::

Then you will get

```bash
linux-firmware-image-5.6.19-rt12_5.6.19-rt12-1_amd64.deb
linux-headers-5.6.19-rt12_5.6.19-rt12-1_amd64.deb
linux-image-5.6.19-rt12_5.6.19-rt12-1_amd64.deb
linux-libc-dev_5.6.19-rt12-1_amd64.deb
```

## Installing the kernel

:::tip
At this point, you can copy the .deb package from a USB stick to another device and install it without having to compile it again
:::

Go to the package folder and install the kernel

```bash
sudo dpkg -i linux-*.deb
```

2. Update grub and reboot

```bash
sudo update-grub
sudo reboot
```

3. Check the kernel version

```shell
uname -a
```

At this point, you can see that the kernel version is marked with ` PREEMPT RT` and you can perform [real-time testing](digging_deeper/rt_test.md)

## Error collection

1. Unable to open kernel configuration interface menuconfig

   Q1:(linux-4.17.2 kernel for example)

   ```bash
   root@simon-virtual-machine:/home/simon/Src/linux-4.17.2# make menuconfig
   YACC scripts/kconfig/zconf.tab.c
   /bin/sh: 1: bison: not found
   scripts/Makefile.lib:196: recipe for target 'scripts/kconfig/zconf.tab.c' failed
   make[1]: *** [scripts/kconfig/zconf.tab.c] Error 127
   Makefile:528: recipe for target 'menuconfig' failed
   make: *** [menuconfig] Error 2
   ```

   A1.

   ```bash
   apt-get install bison -y
   ```

   Q2.
   ``bash
   root@simon-virtual-machine:/home/simon/Src/linux-4.17.2# make menuconfig
   YACC scripts/kconfig/zconf.tab.c
   LEX scripts/kconfig/zconf.lex.c
   /bin/sh: 1: flex: not found
   scripts/Makefile.lib:188: recipe for target 'scripts/kconfig/zconf.lex.c' failed
   make[1]: \*\*\* [scripts/kconfig/zconf.lex.c] Error 127
   Makefile:528: recipe for target

   ````
   A2.
   ```bash
   sudo apt-get install flex
   ````
