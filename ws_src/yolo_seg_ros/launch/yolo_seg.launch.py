import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('yolo_seg_ros')
    cfg = os.path.join(pkg, 'config', 'yolo.yaml')

    return LaunchDescription([
        Node(
            package='yolo_seg_ros',
            executable='yolo_seg_node',
            name='yolo_seg_node',
            parameters=[cfg],
            output='screen',
        ),
    ])
