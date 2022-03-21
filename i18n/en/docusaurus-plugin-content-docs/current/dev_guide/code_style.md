---
id: code_style
sidebar_position: 1
---

# MoveIt Code Style Guidelines

We use the [ROS C++ Style guide](http://wiki.ros.org/CppStyleGuide) for C++ development and (rarely) the [ROS Python Style guide](http://wiki.ros.org/) for Python.

To make your development easier, we recommend using `clang-format` with ROS configuration and the CLion IDE - to see below for details.

In addition rm-controls has some additional style preferences.

## C++

- We use C++14
- Use the C++ standard library (`std::`) whenever possible
- Avoid using C-style functions such as `FLT_EPSILON` and `(int)()` - instead use `std::numeric_limits<double>::epsilon()` and `static_cast<int>()`
- When functions are not available in the standard library, use of the Boost library is encouraged
- Boost is an encouraged library when functionality is not available in the standard library
- Use `#pragma once` in the title instead of "include guards"

## Inline documentation

- We use Doxygen-style comments
- For future work, use `TODO(username): description`
- Add lots of comments to explain complex sections of code
- Prefer full, descriptive variable names over short letters - e.g. `robot_state_` is preferred over `rs_`
- If variable types are not immediately clear from context (e.g. from `make_shared<... >`), avoid using `auto`

## Exceptions

- Catch known exceptions and document them in detail. Avoid using `catch (...) ` because it hides information about possible failures. We want to know if there is a problem.

- We don’t catch exceptions that don’t derive from `std::exception`in MoveIt. It is the responsibility of the plugin provider to handle non-`std::exception`-derived exceptions locally.

## Logging

- Use ROS logging functions and namespaces like: `ROS_INFO_NAMED(LOGNAME, "Starting listener... `.
  - This makes it easier to understand where the output is coming from on the command line, and allows more detailed filtering of the terminal output.
  - The log namespace should be defined as a `const` variable. (For example: `constexpr char LOGNAME[] = "robot_state";`)
  - Using the filename as the LOGNAMED namespace is optimal, i.e. `hardware_interface.cpp` will use `"hardware_interface"`
  - Avoid using the package name as a namespace, as it will already be logged out.

## Deprecation

- Deprecated functions use the C++14 [ `[[deprecated]]` ](https://en.cppreference.com/w/cpp/language/attributes/deprecated) attribute
- Add a useful message to describe how to handle this situation.

```cpp
[[deprecated("use bar instead")]] void foo() {}
```

This will result in.

      warning: 'foo' is deprecated: use bar instead [-Wdeprecated-declarations] foo(); ^ note: 'foo' has been explicitly marked deprecated here void foo() { } ^

- Add an associated TODO describing when the feature will be removed (date and/or ROS version)

## Shared Ptrs

- For creating `shared_ptr` of any object, use MoveIt’s standard `macro` `MOVEIT_CLASS_FORWARD(ClassName)` before the class declaration, and add the include `#include <moveit/macros/class_forward.h>`. This will create two typedefs of shared pointers - `<ClassName>Ptr` and `<ClassName>ConstPtr` using either `boost` or `std`.

## pre-commit format checker

In many of our repositories, there is a [pre-commit](https://pre-commit.com/) check that runs in CI.
You can use it locally and set it to run automatically before committing to avoid failing CI due to formatting errors.
To install it, use pip.

    pip3 install pre-commit

To run all files in the repo manually.

    pre-commit run -a

To automatically run pre-commit before committing in the local repository, run the following command in the local repository directory to install the git hooks.

    pre-commit install

## clang-format Automatic code formatting

Note that if you use pre-commit as described above, clang-format will be run automatically before each commit (if `clang-format-10` is installed). This section describes how to use it manually.

You can run **clang-format** in a number of ways. To install it on Ubuntu, simply run.

    sudo apt install clang-format-10

Please note that we rely on clang-format version **10**. Unfortunately, newer versions are not fully backward compatible.

clang-format requires a configuration file in the root of the catkin workspace, the same configuration file is provided in the root of each repository of rm-controls.

### Command Line

Formatting a single file.

    clang-format-10 -i -style=file MY_FLIE_NAME.cpp

Recursively format the entire directory, including subfolders.

    find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-10 -i -style=file $1

### Exceptions to clang-format

Sometimes the automatic formatting used by clang-format may not make sense, for example for a list of items that are easier to read on a separate line. It can be overridden with the following command.

```cpp
// clang-format off
... some untouched code
// clang-format on
```

But please use it carefully.

:::tip

Too much of a headache to use the command line? Try [CLion IDE configuration](. /ide_config).

:::

### QtCreator Editor Configuration

- Navigate to `Tools > Options > Beautifier` On the `General` tab, enable auto format on file save, using `ClangFormat`. On the `Clang Format` tab, configure `clang-format-10` as your executable and choose `Use predefined style` from `File`.

## clang-tidy Linting

**clang-tidy** is a linting tool for C++. **clang-format** is used to fix code formatting
(incorrect indentation, line length, etc.), while **clang-tidy** will fix programming errors to make your code
more modern, more readable, and less prone to common errors.

You can install **clang-tidy** and other **clang-related** tools
`sudo apt install clang-tidy clang-tools`

Similar to **clang-format**, **clang-tidy** uses the configuration file `.clang-tidy` that is found first when traversing up the source folder hierarchy. All rm-controls repositories provide the same file in the repository root file.

Unlike **clang-format**, **clang-tidy** needs to know the exact compiler options used to build the project. To provide them, configure cmake with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`, which will create a file named `compile_commands.json` in the package's build folder. Once the build is complete, you can run clang-tidy to analyze your code and even fix problems automatically, as follows.

```sh
for file in $(find $CATKIN_WS/build -name compile_commands.json) ; do
	run-clang-tidy -fix -header-filter="$CATKIN_WS/. *" -p $(dirname $file)
done
```

You can also run it on the selected folder or package file by specifying a regular expression to match the file name: ``sh

```sh
run-clang-tidy -fix -header-filter="$CATKIN_WS/. *" -p $CATKIN_WS/build/rm_hw rm_hw
```

Please note that if you have multiple nested "for" loops to convert, clang-tidy
will only fix one at a time. So be sure to run the above command multiple times to convert all the code.

If you are only interested in warnings, clang-tidy can also be run directly during the build.
You can do specific clang-tidy builds with the following command.

```
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_CLANG_TIDY=clang-tidy
catkin build
```

:::tip

Too much of a headache to use the command line? The [CLion IDE comes with clang-tidy](https://www.jetbrains.com/help/clion/clang-tidy-checks-support.html), just use it directly.

:::

### Exceptions to clang-tidy

Unwanted clang-tidy checks can be suppressed by using **NOLINT** or **NOLINTNEXTLINE** annotations. Please specify the name of the check explicitly in parentheses after the comment.

```cpp
const IKCallbackFn solution_callback = 0; // NOLINT(modernize-use-nullptr)

// NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
robot_state::RobotState robot_state(default_state);
```

Note that the `modernize-loop-convert` check may set `for (... ; ... ; ...) ` loop to `for (auto & ... : ...) `.
However, `auto` is sometimes not a highly readable expression.
If it does not become immediately clear from context, explicitly specify the variable type as

```cpp
for (const int & item : container)
  std::cout << item;
```

## Credits

This file come from the [MoveIt Code Style Guidelines](https://moveit.ros.org/documentation/contributing/code/).
