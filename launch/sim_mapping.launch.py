"""SIMULATION - build a map: Gazebo + robot + slam_toolbox (mapping mode). Drive with teleop."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("self_driving_car")

    # Gazebo + robot + controllers + tf_relay (started inside launch_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "launch_sim.launch.py")))

    twist_relay = Node(
        package="self_driving_car", executable="twist_relay.py",
        name="twist_relay", output="screen")

    slam = Node(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox", output="screen",
        parameters=[os.path.join(pkg, "config", "sim_slam_mapping.yaml"),
                    {"use_sim_time": True}])

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["--ros-args", "-p", "use_sim_time:=true"])

    return LaunchDescription([
        gazebo,
        twist_relay,
        TimerAction(period=16.0, actions=[slam]),   # after the TF relay (14 s) is up
        TimerAction(period=18.0, actions=[rviz]),
    ])
