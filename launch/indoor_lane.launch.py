"""Live USB camera + indoor corridor / path detection (indoor_lane.py).  DETECTION ONLY - it does not drive the car.

    ros2 launch self_driving_car indoor_lane.launch.py                    # camera 0  (/dev/video0)
    ros2 launch self_driving_car indoor_lane.launch.py device:=1          # second camera
    ros2 launch self_driving_car indoor_lane.launch.py device:=/dev/v4l/by-id/usb-...-video-index0

The result video is published on /indoor_lane/image/compressed (the dashboard shows it) and the
steering suggestion on /indoor_lane/steer.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue as PV


def generate_launch_description():
    args = [
        DeclareLaunchArgument('device', default_value='0', description='camera number or /dev/video path'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('fourcc', default_value='MJPG', description='MJPG is much faster over USB; empty = camera default'),
        DeclareLaunchArgument('width', default_value='640', description='processing width (smaller = faster on a Raspberry Pi)'),
        DeclareLaunchArgument('publish_fps', default_value='15.0', description='max detection / video rate'),
        DeclareLaunchArgument('jpeg_quality', default_value='70'),
        DeclareLaunchArgument('hood', default_value='0.0', description='row (0-1) where the bonnet starts, 0 = not visible'),
        DeclareLaunchArgument('car_x', default_value='0.5', description='car centre in the picture (0-1)'),
    ]
    node = Node(
        package='self_driving_car', executable='indoor_lane_node.py', name='indoor_lane', output='screen',
        parameters=[{
            'device': PV(LC('device'), value_type=str),
            'camera_width': PV(LC('camera_width'), value_type=int),
            'camera_height': PV(LC('camera_height'), value_type=int),
            'camera_fps': PV(LC('camera_fps'), value_type=int),
            'fourcc': PV(LC('fourcc'), value_type=str),
            'width': PV(LC('width'), value_type=int),
            'publish_fps': PV(LC('publish_fps'), value_type=float),
            'jpeg_quality': PV(LC('jpeg_quality'), value_type=int),
            'hood': PV(LC('hood'), value_type=float),
            'car_x': PV(LC('car_x'), value_type=float),
        }])
    return LaunchDescription(args + [node])
