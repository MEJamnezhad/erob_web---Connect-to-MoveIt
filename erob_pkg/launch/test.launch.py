from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='erob_pkg',
            executable='erob_node',
            name='erob_node',
            prefix=[  # Sudo command cause need to be sudoer when we do this node cause it real time
            "sudo -E env PATH=",
            EnvironmentVariable("PATH", default_value="${PATH}"),
            " LD_LIBRARY_PATH=",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value="${LD_LIBRARY_PATH}"),
            " PYTHONPATH=",
            EnvironmentVariable("PYTHONPATH", default_value="${PYTHONPATH}"),
            " HOME=/tmp ",
            ],
        ),
    ])