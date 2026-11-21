#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from tf2_msgs.msg import TFMessage


class TFOdometryRelay(Node):

    def __init__(self):

        super().__init__('tf_odometry_relay')

        # =====================================================
        # SUBSCRIBER
        # =====================================================

        self.subscription = self.create_subscription(
            TFMessage,
            '/ackermann_controller/tf_odometry',
            self.tf_callback,
            10
        )

        # =====================================================
        # PUBLISHER
        # =====================================================

        self.publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10
        )

        self.get_logger().info(
            'TF relay started: '
            '/ackermann_controller/tf_odometry -> /tf'
        )

    # =========================================================
    # CALLBACK
    # =========================================================

    def tf_callback(self, msg):

        # Do not modify the transform.
        #
        # The Ackermann controller should publish:
        #
        #     odom -> base_footprint
        #
        # We simply forward it to /tf.

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = TFOdometryRelay()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()