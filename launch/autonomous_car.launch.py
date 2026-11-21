import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory("self_driving_car")

    launch_dir = os.path.join(package_dir, "launch")
    config_dir = os.path.join(package_dir, "config")
    nav2_dir = os.path.join(config_dir, "nav2")

    map_file = os.path.join(package_dir, "maps", "my_map.yaml")

    # ============================================================
    # 1. GAZEBO + ROBOT
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "launch_sim.launch.py"
            )
        )
    )

    # ============================================================
    # 2. TF RELAY
    # ============================================================

    tf_relay = Node(
        package="self_driving_car",
        executable="tf_relay.py",
        name="tf_relay",
        output="screen"
    )

    # ============================================================
    # 3. TWIST RELAY
    # ============================================================

    twist_relay = Node(
        package="self_driving_car",
        executable="twist_relay.py",
        name="twist_relay",
        output="screen"
    )

    # ============================================================
    # 4. MAP SERVER
    # ============================================================

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_file,
                "use_sim_time": True
            }
        ]
    )

    # ============================================================
    # 5. AMCL
    # ============================================================

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            os.path.join(
                config_dir,
                "amcl.yaml"
            ),
            {
                "use_sim_time": True
            }
        ]
    )

    # ============================================================
    # 6. LOCALIZATION LIFECYCLE MANAGER
    # ============================================================

    localization_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl"
                ]
            }
        ]
    )

    # ============================================================
    # 7. NAV2
    #
    # Delayed so Gazebo + TF + Map + AMCL have time to start.
    # ============================================================

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "nav2.launch.py"
            )
        )
    )

    delayed_nav2 = TimerAction(
        period=8.0,
        actions=[
            nav2
        ]
    )

    # ============================================================
    # 8. GOAL TRACKER
    # ============================================================

    goal_tracker = Node(
        package="self_driving_car",
        executable="goal_tracker.py",
        name="goal_tracker",
        output="screen"
    )

    # ============================================================
    # 9. RVIZ
    # ============================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "--ros-args",
            "-p",
            "use_sim_time:=true"
        ]
    )

    delayed_rviz = TimerAction(
        period=10.0,
        actions=[
            rviz
        ]
    )

    # ============================================================
    # START EVERYTHING
    # ============================================================

    return LaunchDescription([

        # Start simulation first
        gazebo,

        # Communication relays
        tf_relay,
        twist_relay,

        # Localization
        map_server,
        amcl,
        localization_manager,

        # Navigation after localization has had time to start
        delayed_nav2,

        # Monitoring
        goal_tracker,

        # RViz last
        delayed_rviz,
    ])
