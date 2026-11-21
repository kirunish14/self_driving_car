import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # PACKAGE PATH
    # =========================================================

    pkg_path = get_package_share_directory(
        'self_driving_car'
    )

    xacro_file = os.path.join(
        pkg_path,
        'description',
        'robot.urdf.xacro'
    )

    # =========================================================
    # PROCESS XACRO
    # =========================================================

    robot_description_config = xacro.process_file(
        xacro_file
    )

    robot_description = {
        'robot_description':
        robot_description_config.toxml()
    }

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description
        ]
    )

    # =========================================================
    # JOINT STATE PUBLISHER
    # =========================================================

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[
            robot_description
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])