---
id: clion_ide_config
sidebar_position: 2
---

# CLion IDE 配置

我们推荐使用 CLion IDE 作为电控和视觉的开发 IDE，安装方式见官方教程 [安装 CLion](https://www.jetbrains.com/help/clion/installation-guide.html)。

## ROS 配置

参考 [ROS setup tutorial](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html) 此外，为了兼容 `catkin build` 和 `catkin_make` 应作如下设置

- 打开 **File | Settings | Build, Execution, Deployment | CMake**
- 将 **CMake options** 的值更改为 `-DCATKIN_DEVEL_PREFIX=../devel`
- 将 **Build directory** 的值更改为`../build`

## 保存时自动格式化

### 开启 clang-format

打开 **File | Settings | Editor | Code Style**，在 **ClangFormat** 勾选 **Enable ClangFormat(only for C/C++/Objective-C)**

### 开启 Save Actions

- 打开 **File | Settings | Plugins**，在 **Marketplace** 中搜索并安装 **Save Actions**
- 重启 IDE，转到 **File | Setttings | Save Actions**，勾选以下选项：
  ![save_actions](https://s3.ax1x.com/2020/11/16/Dk9fXD.png)

## 代码同步

在调试时机器人时，有时为了快速修改参数和测试，不会通过 CI 进行构建和发布再安装到机器人上。我们会通过 scp 或 CLion 提供的代码同步，将需要调试的**单个包**传到机器人上的工作空间，在机器人上面编译和运行。

### 关于 Clion 的远程传输配置

1. 首先，在 deployment 处添加文件，文件类型选择 SFTP，并命名（命名取决于是什么机器人，比如 standard4 代表四号步兵。
2. 点击 SSH configuration 处的省略号配置 ssh。

![Alt text](/img/CLion_config/ssh_config.png)

3. 配置 Connection，配置好之后点击 Autodetect 去获取 ssh 账号下的根目录。

![Alt text](/img/CLion_config/connection_config.png)

4. 配置 Mapping。

![Alt text](/img/CLion_config/mapping_config.png)

### 远程传输流程

在 CLion 中的工作空间中，选择远程传输的功能包，鼠标点击右键，选择菜单栏中的 deployment ，upload to “目标主机”。

## 远程开发

在使用本地 IDE 在远程机器人上编译、运行开发和调试较为不常用，一般可参考[Full remote mode](https://www.jetbrains.com/help/clion/remote-projects-support.html)。
