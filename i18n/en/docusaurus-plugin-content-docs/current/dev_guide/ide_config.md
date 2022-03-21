---
id: clion_ide_config
sidebar_position: 2
---

# CLion IDE configuration

We recommend using CLion IDE as the development IDE for electric and vision, see the official tutorial [Installing CLion](https://www.jetbrains.com/help/clion/installation-guide.html) for the installation method.

## ROS configuration

Refer to the [ROS setup tutorial](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html) In addition, for compatibility with `catkin build` and `catkin_make` the following settings should be made

- open **File | Settings | Build, Execution, Deployment | CMake**
- Change the value of **CMake options** to `-DCATKIN_DEVEL_PREFIX=. /devel`
- Change the value of **Build directory** to `... /build`

## Auto-format on save

### Turn on clang-format

Open **File | Settings | Editor | Code Style** and check **Enable ClangFormat(only for C/C++/Objective-C)** under **ClangFormat**

### Turn on Save Actions

- Open **File | Settings | Plugins**, search and install **Save Actions** in **Marketplace**.
- Restart the IDE, go to **File | Settings | Save Actions**, and check the following options.
  ![save_actions](https://s3.ax1x.com/2020/11/16/Dk9fXD.png)

## Code synchronization

When debugging a robot, sometimes for quick parameter changes and testing, you don't build and release via CI before installing to the robot. We will transfer the **single package** that needs to be debugged to the workspace on the robot, compile and run it on top of the robot via scp or the code synchronization provided by CLion.

### About Clion's remote transfer configuration

1. First, add a file to the deployment, choose SFTP as the file type, and name it (the name depends on the robot, e.g. standard4 for Infantry IV.
2. Configure ssh by clicking the ellipsis in SSH configuration.

![Alt text](/img/CLion_config/ssh_config.png)

3. Configure Connection and click Autodetect to get the root directory under the ssh account after the configuration is done.

![Alt text](/img/CLion_config/connection_config.png)

4. Configure Mapping.

![Alt text](/img/CLion_config/mapping_config.png)

### Remote transfer process

In the workspace in CLion, select the feature package for remote transfer, right mouse click, select deployment in the menu bar , upload to "target host".

## Remote Development

It is less common to compile, run development and debug on a remote robot using a local IDE, generally refer to [Full remote mode](https://www.jetbrains.com/help/clion/remote-projects-support.html).
