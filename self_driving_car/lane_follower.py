#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class LaneFollower(Node):

    def __init__(self):

        super().__init__('lane_follower')

        # =====================================================
        # CONTROL PARAMETERS
        # =====================================================

        # Start conservative.
        self.kp = 0.55

        # Maximum steering angle
        self.max_steering = 0.6109

        # Maximum wheel angular velocity
        self.max_speed = 0.50

        # Minimum speed while lane is confidently detected
        self.min_speed = 0.15

        # Current lane error
        self.lane_error = 0.0

        # Whether we have received lane information
        self.lane_received = False

        # =====================================================
        # SUBSCRIBER
        # =====================================================

        self.error_subscription = self.create_subscription(
            Float64,
            '/lane/error',
            self.error_callback,
            10
        )

        # =====================================================
        # STEERING PUBLISHER
        # =====================================================

        self.steering_publisher = self.create_publisher(
            Float64,
            '/vehicle/steering',
            10
        )

        # =====================================================
        # SPEED PUBLISHER
        # =====================================================

        self.speed_publisher = self.create_publisher(
            Float64,
            '/vehicle/speed',
            10
        )

        # =====================================================
        # CONTROL LOOP
        # =====================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Lane follower V2 started.'
        )

    # =========================================================
    # LANE ERROR CALLBACK
    # =========================================================

    def error_callback(self, msg):

        self.lane_error = msg.data

        self.lane_received = True

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):

        # -----------------------------------------------------
        # No lane information
        # -----------------------------------------------------

        if not self.lane_received:

            self.publish_stop()

            return

        # =====================================================
        # STEERING
        # =====================================================

        steering = (
            -self.kp *
            self.lane_error
        )

        # Limit steering

        steering = max(
            -self.max_steering,
            min(
                self.max_steering,
                steering
            )
        )

        # =====================================================
        # SPEED CONTROL
        # =====================================================

        # Large steering error -> slow down.
        steering_ratio = (
            abs(steering)
            /
            self.max_steering
        )

        speed = (
            self.max_speed
            -
            steering_ratio *
            (
                self.max_speed
                -
                self.min_speed
            )
        )

        speed = max(
            self.min_speed,
            min(
                self.max_speed,
                speed
            )
        )

        # =====================================================
        # PUBLISH STEERING
        # =====================================================

        steering_msg = Float64()

        steering_msg.data = steering

        self.steering_publisher.publish(
            steering_msg
        )

        # =====================================================
        # PUBLISH SPEED
        # =====================================================

        speed_msg = Float64()

        speed_msg.data = speed

        self.speed_publisher.publish(
            speed_msg
        )

        # =====================================================
        # DEBUG
        # =====================================================

        self.get_logger().info(
            f'error={self.lane_error:.3f} '
            f'steering={steering:.3f} '
            f'speed={speed:.3f}',
            throttle_duration_sec=0.5
        )

    # =========================================================
    # STOP
    # =========================================================

    def publish_stop(self):

        steering_msg = Float64()

        steering_msg.data = 0.0

        self.steering_publisher.publish(
            steering_msg
        )

        speed_msg = Float64()

        speed_msg.data = 0.0

        self.speed_publisher.publish(
            speed_msg
        )

    # =========================================================
    # SAFE SHUTDOWN
    # =========================================================

    def stop_vehicle(self):

        self.publish_stop()

        self.get_logger().info(
            'Vehicle stopped.'
        )


def main(args=None):

    rclpy.init(args=args)

    node = LaneFollower()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_vehicle()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()