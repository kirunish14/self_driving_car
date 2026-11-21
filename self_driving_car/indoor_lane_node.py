#!/usr/bin/env python3
"""
indoor_lane_node.py - ROS 2 wrapper around indoor_lane.py (corridor / path detection).

Opens a USB camera, runs IndoorLane on every frame and publishes
    /indoor_lane/image/compressed   sensor_msgs/CompressedImage   the picture with the detection drawn on it
    /indoor_lane/steer              std_msgs/Float32              steering suggestion -1 (left) .. +1 (right)

DETECTION ONLY: nothing in here drives the car. The dashboard shows the video.
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))   # find indoor_lane.py next to this file

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

from indoor_lane import IndoorLane, clamp


class IndoorLaneNode(Node):
    def __init__(self):
        super().__init__('indoor_lane')
        self.declare_parameter('device', '0')                  # camera number (0 = /dev/video0) or a path / video file
        self.declare_parameter('camera_width', 640)            # what the camera is asked to deliver
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('fourcc', 'MJPG')               # MJPG = much faster over USB; '' = leave the camera default
        self.declare_parameter('width', 640)                   # processing width (smaller = faster)
        self.declare_parameter('hood', 0.0)                    # row (0-1) where the bonnet starts, 0 = not visible
        self.declare_parameter('car_x', 0.5)                   # car centre in the picture (0-1)
        self.declare_parameter('publish_fps', 15.0)            # max detection / video rate
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('image_topic', '/indoor_lane/image/compressed')
        self.declare_parameter('steer_topic', '/indoor_lane/steer')
        self.declare_parameter('frame_id', 'camera_link')

        g = lambda n: self.get_parameter(n).value
        self.device = str(g('device'))
        self.cam_w, self.cam_h, self.cam_fps = int(g('camera_width')), int(g('camera_height')), int(g('camera_fps'))
        self.fourcc = str(g('fourcc'))
        self.width = int(g('width'))
        self.min_period = 1.0 / max(1.0, float(g('publish_fps')))
        self.quality = int(clamp(int(g('jpeg_quality')), 10, 100))
        self.frame_id = str(g('frame_id'))
        self.det = IndoorLane(car_x=clamp(float(g('car_x')), 0.2, 0.8), hood=clamp(float(g('hood')), 0.0, 0.95))

        self.img_pub = self.create_publisher(CompressedImage, str(g('image_topic')), qos_profile_sensor_data)
        self.steer_pub = self.create_publisher(Float32, str(g('steer_topic')), 10)

        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ camera
    def _open(self):
        src = int(self.device) if self.device.isdigit() else self.device
        cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            cap.release()
            return None
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc[:4]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_h)
        cap.set(cv2.CAP_PROP_FPS, self.cam_fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)                  # always the newest frame, no growing delay
        return cap

    def _prep(self, frame):
        h0, w0 = frame.shape[:2]
        if w0 == self.width:
            return frame
        return cv2.resize(frame, (self.width, int(round(h0 * self.width / w0))))

    # -------------------------------------------------------------------- loop
    def _run(self):
        cap, last, warned = None, 0.0, False
        while self._running and rclpy.ok():
            if cap is None:
                cap = self._open()
                if cap is None:
                    if not warned:
                        self.get_logger().warn(f'cannot open camera "{self.device}" - retrying every 2 s '
                                               f'(check: ls /dev/video*, and that you are in the "video" group)')
                        warned = True
                    time.sleep(2.0)
                    continue
                warned = False
                self.get_logger().info(
                    f'camera "{self.device}" open: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
                    f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}')
            ok, frame = cap.read()
            if not ok or frame is None:
                self.get_logger().warn('camera stopped delivering frames - reopening')
                cap.release()
                cap = None
                time.sleep(1.0)
                continue
            now = time.monotonic()
            if now - last < self.min_period:                 # keep reading (drains the buffer) but do not process
                continue
            last = now
            try:
                out, info = self.det.process(self._prep(frame))
            except Exception as e:                           # a bad frame must never kill the node
                self.get_logger().warn(f'detection error: {e}')
                continue
            if not info['valid']:
                cv2.putText(out, info['text'], (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                self.steer_pub.publish(Float32(data=float(info['steer'])))
            ok, buf = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.img_pub.publish(msg)
        if cap is not None:
            cap.release()

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = IndoorLaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
