#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class GoalTracker(Node):

    def __init__(self):
        super().__init__('goal_tracker')

        self.goal = None
        self.amcl_pose = None

        # RViz Nav2 Goal
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # AMCL pose is in the MAP frame
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # Print every second
        self.timer = self.create_timer(
            1.0,
            self.print_status
        )

        self.get_logger().info('========================================')
        self.get_logger().info('GOAL TRACKER STARTED')
        self.get_logger().info('Goal topic : /goal_pose')
        self.get_logger().info('Car pose   : /amcl_pose')
        self.get_logger().info('Frame      : map')
        self.get_logger().info('========================================')

    def goal_callback(self, msg):

        self.goal = msg

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gz = msg.pose.position.z

        self.get_logger().info('')
        self.get_logger().info('*************** NEW GOAL ***************')
        self.get_logger().info(
            f'Goal X = {gx:.3f} m'
        )
        self.get_logger().info(
            f'Goal Y = {gy:.3f} m'
        )
        self.get_logger().info(
            f'Goal Z = {gz:.3f} m'
        )
        self.get_logger().info(
            f'Goal frame = {msg.header.frame_id}'
        )
        self.get_logger().info('***************************************')
        self.get_logger().info('')

    def pose_callback(self, msg):

        self.amcl_pose = msg

    def quaternion_to_yaw(self, q):

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def print_status(self):

        if self.amcl_pose is None:

            self.get_logger().warn(
                'Waiting for AMCL pose...'
            )

            return

        # ==========================================
        # CAR POSITION - MAP FRAME
        # ==========================================

        car_x = self.amcl_pose.pose.pose.position.x
        car_y = self.amcl_pose.pose.pose.position.y
        car_z = self.amcl_pose.pose.pose.position.z

        car_yaw = self.quaternion_to_yaw(
            self.amcl_pose.pose.pose.orientation
        )

        self.get_logger().info(
            f'CAR  | '
            f'X={car_x:.3f} m | '
            f'Y={car_y:.3f} m | '
            f'Z={car_z:.3f} m | '
            f'Yaw={math.degrees(car_yaw):.2f} deg'
        )

        # ==========================================
        # GOAL POSITION
        # ==========================================

        if self.goal is None:

            self.get_logger().info(
                'GOAL | No goal received'
            )

            return

        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y
        goal_z = self.goal.pose.position.z

        # ==========================================
        # DISTANCE TO GOAL
        # ==========================================

        distance = math.sqrt(
            (goal_x - car_x) ** 2 +
            (goal_y - car_y) ** 2
        )

        self.get_logger().info(
            f'GOAL | '
            f'X={goal_x:.3f} m | '
            f'Y={goal_y:.3f} m | '
            f'Z={goal_z:.3f} m | '
            f'Distance={distance:.3f} m'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GoalTracker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
