---
label: 'clion_ide_config'
sidebar_position: 2
---

# CLion IDE 配置
我们推荐使用 CLion IDE 作为电控和视觉的开发IDE，安装方式见官方教程 [安装CLion](https://www.jetbrains.com/help/clion/installation-guide.html)。

## ROS 配置
参考 [ROS setup tutorial](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html) 此外，为了兼容 ``catkin build`` 和 ``catkin_make`` 应作如下设置
+ 打开 **File | Settings | Build, Execution, Deployment | CMake**
+ 将 **CMake options** 的值更改为 `-DCATKIN_DEVEL_PREFIX=../devel`
+ 将 **Build directory** 的值更改为`../build`

## 保存时自动格式化

### 开启 clang-format
打开 **File | Settings | Editor | Code Style**，在 **ClangFormat** 勾选 **Enable ClangFormat(only for C/C++/Objective-C)**

### 开启 Save Actions
+ 打开 **File | Settings | Plugins**，在 **Marketplace** 中搜索并安装 **Save Actions**
+ 重启IDE，转到 **File | Setttings | Save Actions**，勾选以下选项：
![save_actions](https://s3.ax1x.com/2020/11/16/Dk9fXD.png)

## 代码同步
在调试时机器人时，有时为了快速修改参数和测试，不会通过CI进行构建和发布再安装到机器人上。我们会通过scp或CLion提供的代码同步，将需要调试的**单个包**传到机器人上的工作空间，在机器人上面编译和运行。

## 远程开发
在使用本地IDE在远程机器人上编译、运行开发和调试较为不常用，一般可参考[Full remote mode](https://www.jetbrains.com/help/clion/remote-projects-support.html)。