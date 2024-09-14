import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    vicon_interface_param_file = os.path.join(
        get_package_share_directory("vicon_interface"),
        "param",
        "vicon_interface.param.yaml",
    )

    return launch.LaunchDescription(
        [
            Node(
                package="vicon_interface",
                executable="vicon_stream_node",
                name="vicon_stream_node",
                output="screen",
                parameters=[vicon_interface_param_file],
                remappings=[("vicon_mpc_state", "/vehicle_state")],
                emulate_tty=True,
            ),
        ]
    )
