import os
import re

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # =========================================================
    # PACKAGE PATHS
    # =========================================================

    pkg_self_driving_car = get_package_share_directory(
        'self_driving_car'
    )

    gazebo_ros_path = get_package_share_directory(
        'gazebo_ros'
    )

    # =========================================================
    # WORLD
    # =========================================================

    world_file = os.path.join(
        pkg_self_driving_car,
        'worlds',
        'autonomous_track.world'
    )

    # =========================================================
    # ROBOT XACRO
    # =========================================================

    xacro_file = os.path.join(
        pkg_self_driving_car,
        'description',
        'robot.urdf.xacro'
    )

    robot_description_xml = xacro.process_file(
        xacro_file
    ).toxml()

    # Remove XML comments
    robot_description = re.sub(
        r'<!--.*?-->',
        '',
        robot_description_xml,
        flags=re.DOTALL
    )

    # =========================================================
    # GAZEBO
    #
    # IMPORTANT:
    #
    # DO NOT START PAUSED.
    #
    # ros2_control needs Gazebo's simulation/update loop
    # running when controllers are activated.
    # =========================================================

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_path,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file,
            'pause': 'false'
        }.items()
    )

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',

        output='screen',

        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    # =========================================================
    # SPAWN ROBOT
    # =========================================================

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',

        arguments=[
            '-entity',
            'self_driving_car',

            '-topic',
            'robot_description',

            '-x',
            '-8.0',

            '-y',
            '0.0',

            '-z',
            '0.15'
        ],

        output='screen'
    )

    # =========================================================
    # JOINT STATE BROADCASTER
    # =========================================================
    #
    # IMPORTANT:
    #
    # This is started AFTER the robot is spawned.
    #
    # It is NOT started at the same time as the Ackermann
    # controller.
    # =========================================================

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            'joint_state_broadcaster',

            '--controller-manager',
            '/controller_manager',

            '--controller-manager-timeout',
            '60'
        ],

        output='screen'
    )

    # =========================================================
    # ACKERMANN CONTROLLER
    # =========================================================
    #
    # Started AFTER joint_state_broadcaster.
    # =========================================================

    ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            'ackermann_controller',

            '--controller-manager',
            '/controller_manager',

            '--controller-manager-timeout',
            '60'
        ],

        output='screen'
    )

    # =========================================================
    # TF RELAY
    # =========================================================
    #
    # Ackermann controller publishes:
    #
    # /ackermann_controller/tf_odometry
    #
    # This relay republishes it to:
    #
    # /tf
    #
    # =========================================================

    tf_relay = Node(
        package='self_driving_car',
        executable='tf_relay.py',

        output='screen'
    )

    # =========================================================
    # STARTUP TIMING
    # =========================================================
    #
    # 0.0 sec
    #   Gazebo starts RUNNING
    #
    # 1.0 sec
    #   robot_state_publisher
    #
    # 3.0 sec
    #   robot spawned into Gazebo
    #
    # 8.0 sec
    #   joint_state_broadcaster
    #
    # 11.0 sec
    #   ackermann_controller
    #
    # 14.0 sec
    #   TF relay
    #
    # NO UNPAUSE COMMAND IS REQUIRED.
    #
    # =========================================================

    delayed_robot_state_publisher = TimerAction(
        period=1.0,

        actions=[
            robot_state_publisher
        ]
    )

    delayed_spawn_robot = TimerAction(
        period=3.0,

        actions=[
            spawn_robot
        ]
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=8.0,

        actions=[
            joint_state_broadcaster
        ]
    )

    delayed_ackermann_controller = TimerAction(
        period=11.0,

        actions=[
            ackermann_controller
        ]
    )

    delayed_tf_relay = TimerAction(
        period=14.0,

        actions=[
            tf_relay
        ]
    )

    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================

    return LaunchDescription([

        # -----------------------------------------------------
        # 1. Gazebo
        # -----------------------------------------------------

        gazebo_launch,

        # -----------------------------------------------------
        # 2. Robot State Publisher
        # -----------------------------------------------------

        delayed_robot_state_publisher,

        # -----------------------------------------------------
        # 3. Spawn Robot
        # -----------------------------------------------------

        delayed_spawn_robot,

        # -----------------------------------------------------
        # 4. Joint State Broadcaster
        # -----------------------------------------------------

        delayed_joint_state_broadcaster,

        # -----------------------------------------------------
        # 5. Ackermann Controller
        # -----------------------------------------------------

        delayed_ackermann_controller,

        # -----------------------------------------------------
        # 6. TF Relay
        # -----------------------------------------------------

        delayed_tf_relay,
    ])