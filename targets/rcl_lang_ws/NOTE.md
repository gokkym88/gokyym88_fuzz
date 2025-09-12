# Workspace for RCL APi consistency testing

## Steps

1. Compile `ros2_foxy` workspace (assuming all deps are already installed)
```sh
cd ~/workspace/ros2_foxy
mkdir src
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos
colcon build --symlink-install
```

2. Re-compile `rcl` with instrumentation pass
- using clang + pass
```sh
colcon build --packages-select rcl --cmake-clean-cache --cmake-force-configure
--cmake-args -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
-DCMAKE_BUILD_TYPE=Debug
-DCMAKE_C_FLAGS='/home/seulbae/workspace/drone-sec/src/cov/cov-fs.o
-fsanitize-coverage=trace-pc-guard'
-DCMAKE_CXX_FLAGS='/home/seulbae/workspace/drone-sec/src/cov/cov-fs.o
-fsanitize-coverage=trace-pc-guard' --symlink-install
```

- alternatively, using gcc + gcov
```sh
colcon build --packages-select rcl --cmake-clean-cache --cmake-force-configure
--cmake-args -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
-DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS='--coverage'
-DCMAKE_CXX_FLAGS='--coverage' --symlink-install
```

3. Source env
```sh
source ~/workspace/ros2_foxy/install/setup.zsh
```

4. Compile this workspace
```sh
colcon build --symlink-install
```

5. Also check `NOTE.cov.md`

6. Run harness

## Results

* The code coverage of `librcl.so` is quite different across languages
  * How `rclcpp` and `rclpy` interfaces are implementated differently in terms
    of their usage of the `rcl` APIs.
  * e.g., during Node initialization, `rclcpp`'s `TimeSource` internally
    invokes `rcl`'s subscription interface to subscribe to topic
    `/parameter_events` when attaching a node. `rclpy` does not create a
    subscription.
    * why subscribe when there's already service & client for parameter
      setting events? investigation in progress.

## Bugs
* rclpy `_on_parameter_event` always returns `success=True`
  * details: https://github.com/ros2/rclpy/pull/817
