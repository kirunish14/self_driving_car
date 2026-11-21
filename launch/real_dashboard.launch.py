"""STEP 3 - drive the real car from a browser (phone / tablet / laptop).

Starts the normal navigation stack (real_navigation.launch.py: lidar, car_bridge, map_server, AMCL, Nav2)
AND the web dashboard.  Nothing in the navigation is changed.

    ros2 launch self_driving_car real_dashboard.launch.py map:=$HOME/car_maps/my_map.yaml
    -> open  http://<car-ip>:8080  on any device on the same Wi-Fi

Options
    pin:=1234        ask for a PIN in the browser (recommended when other people share the Wi-Fi)
    port:=8080       web port
    lane:=true       also start the USB camera + indoor lane detection; its video appears on the dashboard
    camera:=0        which camera when lane:=true
    nav:=false       dashboard only (use when the navigation stack is already running in another terminal)
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue as PV


def generate_launch_description():
    pkg = get_package_share_directory('self_driving_car')
    car = yaml.safe_load(open(os.path.join(pkg, 'config', 'real', 'car.yaml')))['car']

    args = [
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/car_maps/my_map.yaml'),
                              description='map .yaml the navigation starts with'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('pin', default_value=''),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('lane', default_value='false'),
        DeclareLaunchArgument('camera', default_value='0'),
    ]

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'real_navigation.launch.py')),
        launch_arguments={'map': LC('map')}.items(), condition=IfCondition(LC('nav')))

    lane = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'indoor_lane.launch.py')),
        launch_arguments={'device': LC('camera')}.items(), condition=IfCondition(LC('lane')))

    dashboard = Node(
        package='self_driving_car', executable='dashboard_node.py', name='dashboard', output='screen',
        parameters=[{
            'port': PV(LC('port'), value_type=int),
            'pin': PV(LC('pin'), value_type=str),
            'initial_map': PV(LC('map'), value_type=str),
            'wheelbase': float(car['wheelbase']),
            'front_overhang': float(car['front_overhang']),
            'rear_overhang': float(car['rear_overhang']),
            'width': float(car['width']),
        }])

    return LaunchDescription(args + [navigation, lane, dashboard])
