"""STEP 2 - load the saved map, localise (AMCL) and navigate to a 2D Goal Pose with Nav2."""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# False = Nav2 stock behaviour tree.
# True  = go around obstacles if there is free space, otherwise STOP and WAIT and keep re-planning.
USE_WAIT_BT = False
# True: divert around obstacles, wait 3 s then reverse only when stuck, re-plan only when the path is blocked
# (behavior_trees/navigate_divert_reverse.xml). False: the stock Nav2 tree (spins, re-plans every second).
USE_DIVERT_BT = True


def has_bt_plugin(name):
    """True when this Nav2 installation has the behavior-tree plugin library <name>."""
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix('nav2_behavior_tree')
    except Exception:
        return False
    return os.path.exists(os.path.join(prefix, 'lib', 'lib' + name + '.so'))


def build_nav2_params(pkg):
    """Take config/nav2_params.yaml and fill in everything that depends on the real car."""
    car = yaml.safe_load(open(os.path.join(pkg, 'config', 'real', 'car.yaml')))['car']
    p = yaml.safe_load(open(os.path.join(pkg, 'config', 'real', 'nav2_params.yaml')))

    L = float(car['wheelbase'])
    r_min = L / math.tan(math.radians(float(car['max_steer_deg'])))
    vmax = float(car['max_speed'])
    acc = float(car['accel_limit'])
    front, rear, half_w = float(car['front_overhang']), float(car['rear_overhang']), float(car['width']) / 2.0
    footprint = str([[front, half_w], [front, -half_w], [-rear, -half_w], [-rear, half_w]])

    p['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = round(r_min * 1.15, 2)
    p['global_costmap']['global_costmap']['ros__parameters']['footprint'] = footprint
    p['local_costmap']['local_costmap']['ros__parameters']['footprint'] = footprint

    # Regulated Pure Pursuit (can reverse): speed limit and the radius below which it slows down
    c = p['controller_server']['ros__parameters']['FollowPath']
    c['desired_linear_vel'] = vmax
    c['regulated_linear_scaling_min_radius'] = round(r_min * 1.2, 2)

    if USE_WAIT_BT:
        p['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = os.path.join(
            pkg, 'behavior_trees', 'navigate_wait.xml')
    elif USE_DIVERT_BT:
        bt = p['bt_navigator']['ros__parameters']
        if has_bt_plugin('nav2_is_path_valid_condition_bt_node'):
            if 'nav2_is_path_valid_condition_bt_node' not in bt['plugin_lib_names']:
                bt['plugin_lib_names'].append('nav2_is_path_valid_condition_bt_node')
            xml = 'navigate_divert_reverse.xml'
        else:
            xml = 'navigate_divert_reverse_basic.xml'
            print('[real_navigation] this Nav2 has no IsPathValid node - using ' + xml + ' (re-plans once per second)')
        bt['default_nav_to_pose_bt_xml'] = os.path.join(pkg, 'behavior_trees', xml)

    out = '/tmp/self_driving_car_nav2_params.yaml'
    with open(out, 'w') as f:
        yaml.safe_dump(p, f)
    return out, r_min


def generate_launch_description():
    pkg = get_package_share_directory('self_driving_car')
    params, _ = build_nav2_params(pkg)

    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg, 'maps', 'real_map.yaml'),
        description='full path to the map yaml saved after mapping')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'real_bringup.launch.py')))

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': False}])

    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[params])

    loc_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True, 'bond_timeout': 10.0,
                     'node_names': ['map_server', 'amcl']}])

    planner = Node(package='nav2_planner', executable='planner_server',
                   name='planner_server', output='screen', parameters=[params])
    controller = Node(package='nav2_controller', executable='controller_server',
                      name='controller_server', output='screen', parameters=[params])
    behaviors = Node(package='nav2_behaviors', executable='behavior_server',
                     name='behavior_server', output='screen', parameters=[params])
    bt_nav = Node(package='nav2_bt_navigator', executable='bt_navigator',
                  name='bt_navigator', output='screen', parameters=[params])

    nav_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True, 'bond_timeout': 10.0,
                     'node_names': ['planner_server', 'controller_server',
                                    'behavior_server', 'bt_navigator']}])

    return LaunchDescription([
        map_arg, bringup, map_server, amcl, loc_manager,
        planner, controller, behaviors, bt_nav, nav_manager,
    ])
