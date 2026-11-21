import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


# False = Nav2 stock behaviour tree (path blocked -> clear costmaps, spin, wait, back up, then abort)
# True  = "go around the obstacle if there is free space, otherwise stop and WAIT, keep
#          re-planning every 2 s and continue as soon as a free path appears"
USE_WAIT_BT = False


def generate_launch_description():

    pkg_share = get_package_share_directory("self_driving_car")

    nav2_config = os.path.join(pkg_share, "config", "nav2")

    return LaunchDescription([

        # Planner Server
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[
                os.path.join(nav2_config, "planner.yaml"),
                os.path.join(nav2_config, "global_costmap.yaml"),
            ],
        ),

        # Controller Server
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[
                os.path.join(nav2_config, "controller.yaml"),
                os.path.join(nav2_config, "local_costmap.yaml"),
            ],
        ),
        
        Node(
	    package="nav2_behaviors",
	    executable="behavior_server",
	    name="behavior_server",
	    output="screen",
	    parameters=[
		os.path.join(nav2_config, "behavior_server.yaml"),
	    ],
),

        # Behavior Tree Navigator
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                os.path.join(nav2_config, "bt_navigator.yaml"),
            ] + ([{"default_nav_to_pose_bt_xml": os.path.join(
                    pkg_share, "behavior_trees", "navigate_wait.xml")}]
                 if USE_WAIT_BT else []),
        ),

        # Lifecycle Manager
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                os.path.join(nav2_config, "lifecycle.yaml"),
            ],
        ),
    ])
