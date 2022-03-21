---
id: code_style
sidebar_position: 1
---

# 代码规范

我们使用在 C++开发中使用 [ROS C++ Style guide](http://wiki.ros.org/CppStyleGuide) ，在 Python 开发中（很少）使用 [ROS Python Style guide](http://wiki.ros.org/PyStyleGuide) , 你还可以将 [ros_best_pracitces](https://github.com/leggedrobotics/ros_best_practices) 作为模版参考.

为了让你的开发更方便, 我们推荐你使用 带有 ROS 配置的 `clang-format` 和 CLion IDE - 详见下方.

此外 rm-controls 有一些额外的样式偏好：

## C++

- 我们使用 C++14
- 尽可能使用 C++ 标准库 (`std::`)
- 避免使用 C 风格的函数，比如： `FLT_EPSILON` 和 `(int)()`- 而是使用 `std::numeric_limits<double>::epsilon()` 和 `static_cast<int>()`
- 当功能在标准库中不可用时，鼓励使用 Boost 库
- 在标题中使用 `#pragma once` 而不是 "include guards"

## 代码内文档

- 我们使用 Doxygen 风格的注释
- 对于未来的工作，使用 `TODO(username): description`
- 添加大量注释以解释复杂的代码部分
- 更偏向使用完整的、描述性的变量名称而不是简短的字母 - 例如 `robot_state_` 优于 `rs_`
- 如果变量类型不能从上下文中立即变得清晰（例如从 `make_shared<...>`），避免使用 `auto`

## Exceptions

- 捕获已知异常并详细记录它们。 避免使用 `catch (...)` 因为它隐藏了有关可能故障的信息。 我们想知道是否有问题。

## Logging

- 使用 ROS logging 功能和命名空间，如： `ROS_INFO_NAMED(LOGNAME, "Starting listener...`.
  - 这使得在命令行上更容易理解输出来自哪里，并允许对终端输出的信息进行更详细的筛选。
  - 日志命名空间应该被定义为一个 `const` 变量。 （例如：`constexpr char LOGNAME[] = "robot_state";`)
  - 使用文件名作为 LOGNAMED 命名空间是最佳的，即 `hardware_interface.cpp` 将使用 `"hardware_interface"`
  - 避免使用包名称作为命名空间，因为它已经会被 logger 输出。

## 弃用

- 弃用函数使用 C++14 [ `[[deprecated]]` ](https://en.cppreference.com/w/cpp/language/attributes/deprecated) 属性
- 添加一条有用的消息来描述如何处理这种情况：

```cpp
[[deprecated("use bar instead")]] void foo() {}
```

这将导致：

      warning: 'foo' is deprecated: use bar instead [-Wdeprecated-declarations] foo(); ^ note: 'foo' has been explicitly marked deprecated here void foo() {} ^

- 添加相关的 TODO，描述何时删除该功能（日期和/或 ROS 版本）

## pre-commit 格式检查器

在我们的许多存储库中，有一个在 CI 中运行的 [pre-commit](https://pre-commit.com/) 检查。
你可以在本地使用它并将其设置为在提交之前自动运行，从而避免因为格式错误而无法通过 CI。
要安装，请使用 pip：

    pip3 install pre-commit

要手动运行 repo 中的所有文件：

    pre-commit run -a

要在本地存储库中提交之前自动运行 pre-commit，请在本地仓库目录中运行以下指令安装 git hooks：

    pre-commit install

## clang-format 自动代码格式化

请注意，如果您如上所述使用预提交，则每次提交之前都会自动运行 clang-format（如果安装了 `clang-format-10`）。 本节介绍如何手动使用它。

您可以通过多种方式运行 **clang-format**。 要在 Ubuntu 上安装，只需运行：

    sudo apt install clang-format-10

请注意，我们依赖 clang 格式版本 **10**。 遗憾的是，较新的版本并不完全向后兼容。

clang-format 需要在 catkin 工作区的根目录下有一个配置文件，rm-controls 的每个仓库的根目录中都提供了相同的配置文件。

### Command Line

格式化单个文件：

    clang-format-10 -i -style=file MY_FLIE_NAME.cpp

递归格式化整个目录，包括子文件夹：

    find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-10 -i -style=file $1

### clang-format 的例外

有时， clang-format 使用的自动格式化可能没有意义，例如 用于在单独的行上更容易阅读的项目列表。 它可以用以下命令覆盖：

```cpp
// clang-format off
... some untouched code
// clang-format on
```

不过请谨慎使用。

:::tip

使用命令行太头疼了? 试试 [CLion IDE 配置 ](./ide_config).

:::

## clang-tidy Linting

**clang-tidy** 是 C++ 的 linting 工具。 **clang-format** 用于修复代码格式
（错误的缩进、行长等），而**clang-tidy** 将修复编程错误以使您的代码
更现代，更易读，更不容易出现常见错误。

您可以安装 clang-tidy 和其他与 clang 相关的工具
`sudo apt install clang-tidy clang-tools`

与 clang-format 类似，clang-tidy 使用在向上遍历源文件夹层次结构时首先找到的配置文件 `.clang-tidy`。 所有 rm-controls 存储库都在存储库根文件中提供相同的文件。

与 clang-format 不同，clang-tidy 需要知道用于构建项目的确切编译器选项。 要提供它们，请使用 `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 配置 cmake，cmake 将在包的构建文件夹中创建一个名为 `compile_commands.json` 的文件。 构建完成后，您可以运行 clang-tidy 来分析您的代码，甚至可以自动修复问题，如下所示：

```sh
for file in $(find $CATKIN_WS/build -name compile_commands.json) ; do
	run-clang-tidy -fix -header-filter="$CATKIN_WS/.*" -p $(dirname $file)
done
```

您还可以通过指定正则表达式来匹配文件名，在选定的文件夹或包文件上运行它：

```sh
run-clang-tidy -fix -header-filter="$CATKIN_WS/.*" -p $CATKIN_WS/build/rm_hw rm_hw
```

请注意，如果您有多层嵌套的“for”循环需要转换，clang-tidy
一次只会修复一个。 所以一定要多次运行上面的命令来转换所有代码。

如果您只对警告感兴趣，clang-tidy 也可以在构建期间直接运行。
您可以使用以下命令进行特定的 clang-tidy 构建：

```
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_CLANG_TIDY=clang-tidy
catkin build
```

:::tip

使用命令行太头疼了? [CLion IDE 自带 clang-tidy](https://www.jetbrains.com/help/clion/clang-tidy-checks-support.html)，直接使用即可。

:::

### clang-tidy 的例外

可以通过使用 **NOLINT** 或 **NOLINTNEXTLINE** 注释来抑制不需要的 clang-tidy 检查。 请在注释后的括号中明确指定 check 的名称：

```cpp
const IKCallbackFn solution_callback = 0; // NOLINT(modernize-use-nullptr)

// NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
robot_state::RobotState robot_state(default_state);
```

请注意，`modernize-loop-convert` 检查可能会将 `for (...; ...; ...)` 循环转换为 `for (auto & ... : ...)`。
然而，`auto` 有时候不是一个高度可读的表达式。
如果它不能立即从上下文中变得清晰，请明确指定变量类型：

```cpp
for (const int & item : container)
  std::cout << item;
```

## Credits

这个文件翻译修改自 [MoveIt Code Style Guidelines](https://moveit.ros.org/documentation/contributing/code/).
