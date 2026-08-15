import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('object_slam')
    cfg = os.path.join(pkg, 'config', 'object_slam.yaml')

    return LaunchDescription([
        Node(
            package='object_slam',
            executable='object_slam_node',
            name='object_slam_node',
            parameters=[cfg],
            output='screen',
        ),
    ])
