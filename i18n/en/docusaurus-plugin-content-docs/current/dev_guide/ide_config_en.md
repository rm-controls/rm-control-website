# IDE configuration

For the download and basic configuration of CLion, please refer to [Install CLion](https://www.jetbrains.com/help/clion/installation-guide.html).

## Auto format

- Go to **File | Settings | Plugins**, search for and install **Save Actions** in the marketplace
- Restart IDE, go to **File | Settings | Save Actions**, and check the following options:

![save_actions](https://s3.ax1x.com/2020/11/16/Dk9fXD.png)

- Go to **File | Settings | Editor | Code Style | C/C++**, click **Set from...** on the right and select **Google**.

## ROS configuration

- Go to **File | Settings | Build, Execution, Deployment | CMake**
- Change the value of **CMake options** to `-DCATKIN_DEVEL_PREFIX=../devel`
- Change the value of **Build directory** to `../build`

## Remote development

- For the remote development, please refer to [Full remote mode](https://www.jetbrains.com/help/clion/remote-projects-support.html).
