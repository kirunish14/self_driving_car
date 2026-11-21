#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TwistRelay(Node):
    """
    /cmd_vel (from Nav2 / DWB)  ->  /ackermann_controller/reference_unstamped

    DWB thinks the robot is differential, so it can ask for "turn on the spot" or very
    sharp turns at low speed. An Ackermann car cannot do that, so the angular velocity
    is limited to what the steering can physically achieve:

        |w| <= |v| * tan(max_steering) / wheelbase
    """

    def __init__(self):
        super().__init__('twist_relay')

        self.declare_parameter('wheelbase', 0.76)
        # URDF limit is 0.610865 rad (35 deg); keep a little margin
        self.declare_parameter('max_steering', 0.55)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering').value

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.publisher = self.create_publisher(
            Twist, '/ackermann_controller/reference_unstamped', 10)

        self.get_logger().info(
            'Twist relay started: /cmd_vel -> /ackermann_controller/reference_unstamped '
            f'(steering-limited, wheelbase={self.wheelbase}, max_steering={self.max_steering})')

    def cmd_vel_callback(self, msg):
        out = Twist()
        out.linear.x = msg.linear.x

        w_max = abs(msg.linear.x) * math.tan(self.max_steering) / self.wheelbase
        out.angular.z = max(-w_max, min(w_max, msg.angular.z))

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
