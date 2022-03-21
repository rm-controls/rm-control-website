# Install

## Software Dependency Configuration

ROS melody is installed by default. Refer to the [Installation Guide](http://wiki.ros.org/melodic/Installation) to install ROS

After installing ROS, install the required dependency packages

    sudo apt-get install ros-melodic-apriltag-ros              \
                         ros-melodic-behaviortree-cpp-v3       \

## Pull and compile from Git

- Code of clone warehouse from GitHub

  `git clone https://github.com/QiayuanLiao/RM-Software.git`

> [!Tip]
>
> The warehouse contains a catkin\_ Workspace, which needs to be added in '. Bashrc'.

- Initialization submodule

  `git submodule update --init --recursive `

> [!Note]
>
> Camera driver and visual algorithm are managed in the form of submodule.

Common problem：

During initialization，If there is a problem`Unable to read remote warehouse. Please confirm that you have the correct access rights and that the warehouse exists`，Please refer to [the solution](https://blog.csdn.net/qq_36770641/article/details/88638573)

- Compile

  ```
  cd RM-Software/rm_ws/
  catkin_make
  # Load environment variables
  source devel/setup.bash
  ```
