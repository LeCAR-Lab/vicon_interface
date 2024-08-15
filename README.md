# vicon_interface

Vicon ROS2 Python Driver

# Install

1. Use rosdep to install the ROS dependencies
2. For older ROS2 versions (before Jazzy), execute `sudo pip3 install transforms3d` (required by [`tf_transformations`](https://github.com/DLu/tf_transformations))

# Configure

Please update `vicon_tracker_ip` and `vicon_object_name` to match the Vicon PC's IP, and the name of the object.

# Build

```bash
colcon build
```

# Run

```bash
ros2 launch vicon_interface vicon_interface.launch.py
```
