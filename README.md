# vicon_interface

Vicon ROS2 Python Driver

# Install

1. Use rosdep to install the ROS dependencies: `rosdep install --from-paths ./ --ignore-src -r -y`
2. For older ROS2 versions (before Jazzy), execute `sudo pip3 install transforms3d` (required by [`tf_transformations`](https://github.com/DLu/tf_transformations))

# Configure

Please update `vicon_tracker_ip` and `vicon_object_name` to match the Vicon PC's IP, and the name of the object.

# Build

```bash
source /opt/ros/<ros_distro>/setup.bash
colcon build
```

# Run

```bash
source install/setup.bash
ros2 launch vicon_interface vicon_interface.launch.py
```
