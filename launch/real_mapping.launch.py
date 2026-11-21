"""STEP 1 - drive the car around and build the map with slam_toolbox."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('self_driving_car')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'real_bringup.launch.py')))

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[os.path.join(pkg, 'config', 'real', 'slam_mapping.yaml')])

    return LaunchDescription([bringup, slam])
