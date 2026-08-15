import os
from glob import glob
from setuptools import setup

package_name = 'yolo_seg_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dxr',
    maintainer_email='dxr@example.com',
    description='YOLOv8-seg 实例分割 ROS2 节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_seg_node = yolo_seg_ros.yolo_seg_node:main',
            'export_trt = yolo_seg_ros.export_trt:main',
        ],
    },
)
