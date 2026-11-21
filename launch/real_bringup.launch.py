"""Hardware bring-up: TF frames, RPLidar, ESP32 bridge (odometry + /cmd_vel -> car)."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('self_driving_car')
    car = yaml.safe_load(open(os.path.join(pkg, 'config', 'real', 'car.yaml')))['car']

    # ---- simple robot model (box + 4 wheels) so RViz "RobotModel" shows the car ----
    L = float(car['wheelbase'])
    front, rear = float(car['front_overhang']), float(car['rear_overhang'])
    width = float(car['width'])
    r = float(car.get('wheel_radius', 0.10))
    body_len = front + rear
    body_cx = (front - rear) / 2.0
    wy = width / 2.0 - 0.04

    def wheel(name, x, y):
        return f"""
  <link name="{name}">
    <visual><origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="{r}" length="0.08"/></geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1"/></material></visual>
  </link>
  <joint name="{name}_joint" type="fixed">
    <parent link="base_link"/><child link="{name}"/>
    <origin xyz="{x} {y} {r}" rpy="0 0 0"/></joint>"""

    urdf = f"""<?xml version="1.0"?>
<robot name="kids_car">
  <link name="base_link"/>
  <link name="body">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="{body_len} {width - 0.08} 0.20"/></geometry>
      <material name="blue"><color rgba="0.2 0.4 0.9 0.8"/></material></visual>
  </link>
  <joint name="body_joint" type="fixed">
    <parent link="base_link"/><child link="body"/>
    <origin xyz="{body_cx} 0 {r + 0.10}" rpy="0 0 0"/></joint>
  {wheel('rear_left_wheel', 0.0, wy)}
  {wheel('rear_right_wheel', 0.0, -wy)}
  {wheel('front_left_wheel', L, wy)}
  {wheel('front_right_wheel', L, -wy)}
</robot>"""

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': urdf}])

    static_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'])

    static_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=[str(car['lidar_x']), str(car['lidar_y']), str(car['lidar_z']),
                   str(car['lidar_yaw']), '0', '0', 'base_link', 'laser_frame'])

    lidar = Node(
        package='rplidar_ros', executable='rplidar_node', name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': car['lidar_port'],
            'serial_baudrate': int(car['lidar_baud']),
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }])

    bridge = Node(
        package='self_driving_car', executable='car_bridge.py', name='car_bridge',
        output='screen',
        parameters=[{
            'port': car['esp32_port'],
            'baud': int(car['esp32_baud']),
            'wheelbase': float(car['wheelbase']),
            'max_steer_deg': float(car['max_steer_deg']),
            'max_speed': float(car['max_speed']),
            'v_at_full_cmd': float(car['v_at_full_cmd']),
            'accel_limit': float(car['accel_limit']),
            'steer_rate_limit': float(car['steer_rate_limit']),
            'max_reverse_speed': float(car.get('max_reverse_speed', 0.20)),
            'steer_settle_deg': float(car.get('steer_settle_deg', 4.0)),
        }])

    return LaunchDescription([robot_state_publisher, static_base, static_laser, lidar, bridge])
