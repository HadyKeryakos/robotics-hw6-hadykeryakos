import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('perception_manager'),
        'config',
        'params.yaml',
    )

    return LaunchDescription([
        Node(
            package='perception_manager',
            executable='camera_tf_broadcaster',
            name='camera_tf_broadcaster',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='perception_manager',
            executable='perception_manager_node',
            name='perception_manager_node',
            parameters=[params_file],
            output='screen',
        ),
    ])
