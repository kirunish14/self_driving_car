#!/usr/bin/env python3
"""
ROS 2  <->  ESP32 bridge for the Ackermann kids ride-on car.

  /cmd_vel (Twist)  ->  steering angle + speed  ->  serial  "C,<speed>,<steer>\\n"  ->  ESP32
  dead-reckoned /odom + TF odom -> base_footprint (no wheel encoders needed)

All four motors always get the SAME command (no differential steering).
Steering is done only by the servo.
"""
import math
import time

import rclpy
import serial
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def approach(current, target, max_step):
    if target > current:
        return min(target, current + max_step)
    return max(target, current - max_step)


class CarBridge(Node):

    def __init__(self):
        super().__init__('car_bridge')

        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheelbase', 0.76)
        self.declare_parameter('max_steer_deg', 20.0)
        self.declare_parameter('max_speed', 0.40)
        self.declare_parameter('v_at_full_cmd', 0.80)
        self.declare_parameter('accel_limit', 0.50)
        self.declare_parameter('steer_rate_limit', 1.50)
        self.declare_parameter('max_reverse_speed', 0.20)    # m/s, slower going backwards (can not see behind)
        self.declare_parameter('steer_settle_deg', 4.0)      # steer first: wheels must be this close to the target
        self.declare_parameter('steer_settle_timeout', 1.5)  # ... but never wait longer than this (s)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        g = self.get_parameter
        self.port = g('port').value
        self.baud = g('baud').value
        self.L = g('wheelbase').value
        self.max_steer = math.radians(g('max_steer_deg').value)
        self.max_speed = g('max_speed').value
        self.v_full = g('v_at_full_cmd').value
        self.accel = g('accel_limit').value
        self.steer_rate = g('steer_rate_limit').value
        self.max_reverse = abs(g('max_reverse_speed').value)
        self.settle_tol = math.radians(g('steer_settle_deg').value)
        self.settle_timeout = g('steer_settle_timeout').value
        self.timeout = g('cmd_timeout').value
        self.rate = g('rate').value
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value

        self.target_v = 0.0
        self.target_delta = 0.0
        self.v = 0.0          # smoothed speed (m/s), also used for odometry
        self.delta = 0.0      # smoothed steering angle (rad, + = left)
        self.last_cmd_time = 0.0
        self.hold_since = None   # when the car started waiting for the steering (see loop)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.ser = None
        self.last_connect_try = 0.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br = TransformBroadcaster(self)

        self.dt = 1.0 / self.rate
        self.last_time = time.monotonic()
        self.create_timer(self.dt, self.loop)

        self.get_logger().info(
            f'car_bridge: L={self.L} m, max steer={math.degrees(self.max_steer):.1f} deg, '
            f'min turning radius={self.L / math.tan(self.max_steer):.2f} m, '
            f'max speed={self.max_speed} m/s, max reverse={self.max_reverse} m/s, port={self.port}')

    # ------------------------------------------------------------ serial
    def connect(self):
        now = time.monotonic()
        if now - self.last_connect_try < 2.0:
            return
        self.last_connect_try = now
        try:
            s = serial.Serial()
            s.port = self.port
            s.baudrate = self.baud
            s.timeout = 0
            s.dtr = False      # do not reset the ESP32 when the port opens
            s.rts = False
            s.open()
            self.ser = s
            self.get_logger().info(f'ESP32 connected on {self.port}')
        except Exception as e:  # noqa
            self.ser = None
            self.get_logger().warn(
                f'cannot open {self.port}: {e}', throttle_duration_sec=5.0)

    def send(self, speed_norm, steer_norm):
        if self.ser is None:
            self.connect()
            return
        try:
            self.ser.write(f'C,{speed_norm:.3f},{steer_norm:.3f}\n'.encode())
            self.ser.read(256)   # drain ESP32 status text so its TX buffer never fills
        except Exception as e:  # noqa
            self.get_logger().error(f'serial error: {e}')
            try:
                self.ser.close()
            except Exception:  # noqa
                pass
            self.ser = None

    # ------------------------------------------------------------ /cmd_vel
    def cmd_cb(self, msg: Twist):
        v_cmd = msg.linear.x
        v = clamp(v_cmd, -self.max_reverse, self.max_speed)   # reverse is slower than forward
        w = msg.angular.z

        # Ackermann: tan(delta) = L * w / v.  Use the speed Nav2 asked for (not the limited one) so the
        # curve stays the same when the speed is limited. v < 0 (reversing) gives the correct steering.
        # At (almost) standstill keep the wheels where they are.
        if abs(v_cmd) > 0.02:
            delta = math.atan(self.L * w / v_cmd)
            self.target_delta = clamp(delta, -self.max_steer, self.max_steer)
        self.target_v = v
        self.last_cmd_time = time.monotonic()

    # ------------------------------------------------------------ main loop
    def loop(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = self.dt

        # no command for a while -> stop (steering stays)
        if now - self.last_cmd_time > self.timeout:
            self.target_v = 0.0

        # STEER FIRST: before the car starts moving, or changes between forward and reverse, the servo
        # must have reached the commanded angle. The car waits (speed 0) until it has, max steer_settle_timeout.
        target_v = self.target_v
        want = (target_v > 0.02) - (target_v < -0.02)
        have = (self.v > 0.03) - (self.v < -0.03)
        if want != 0 and have != want and abs(self.target_delta - self.delta) > self.settle_tol:
            if self.hold_since is None:
                self.hold_since = now
            if now - self.hold_since < self.settle_timeout:
                target_v = 0.0
        else:
            self.hold_since = None

        # smooth speed and steering: this smoothed value is what the car really gets
        self.v = approach(self.v, target_v, self.accel * dt)
        self.delta = approach(self.delta, self.target_delta, self.steer_rate * dt)

        speed_norm = clamp(self.v / self.v_full, -1.0, 1.0)
        steer_norm = clamp(self.delta / self.max_steer, -1.0, 1.0)
        self.send(speed_norm, steer_norm)

        # dead-reckoning (bicycle model, reference point = rear axle)
        w = self.v * math.tan(self.delta) / self.L
        self.th += w * dt
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        self.publish_odom(w)

    def publish_odom(self, w):
        stamp = self.get_clock().now().to_msg()
        q = Quaternion()
        q.z = math.sin(self.th / 2.0)
        q.w = math.cos(self.th / 2.0)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_br.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation = q
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = w
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05
        cov[14] = cov[21] = cov[28] = 1e6
        cov[35] = 0.1
        o.pose.covariance = cov
        o.twist.covariance = list(cov)
        self.odom_pub.publish(o)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            time.sleep(0.02)
        if self.ser:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = CarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
