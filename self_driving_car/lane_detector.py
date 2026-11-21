#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class LaneDetector(Node):

    def __init__(self):
        super().__init__('lane_detector')

        self.bridge = CvBridge()

        # -----------------------------------------------------
        # Camera subscriber
        # -----------------------------------------------------

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/image_raw',
            self.image_callback,
            10
        )

        # -----------------------------------------------------
        # Lane error publisher
        # -----------------------------------------------------

        self.error_publisher = self.create_publisher(
            Float64,
            '/lane/error',
            10
        )

        # -----------------------------------------------------
        # Filtering
        # -----------------------------------------------------

        self.filtered_lane_center = None

        self.alpha = 0.25

        # Remember previous detections if one marking temporarily
        # disappears.
        self.last_left_boundary = None
        self.last_divider = None

        self.get_logger().info(
            'Lane detector V3 started - LEFT lane following.'
        )


    # =========================================================
    # IMAGE CALLBACK
    # =========================================================

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as error:
            self.get_logger().error(
                f'cv_bridge error: {error}'
            )
            return


        # -----------------------------------------------------
        # Standard image size
        # -----------------------------------------------------

        frame = cv2.resize(
            frame,
            (640, 480)
        )

        height, width = frame.shape[:2]

        image_center = width // 2


        # =====================================================
        # ROAD REGION
        # =====================================================

        roi_top = int(height * 0.52)

        roi = frame[
            roi_top:height,
            :
        ]

        roi_height = roi.shape[0]


        # =====================================================
        # WHITE MARKING MASK
        # =====================================================

        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        lower_white = np.array(
            [0, 0, 70],
            dtype=np.uint8
        )

        upper_white = np.array(
            [180, 120, 255],
            dtype=np.uint8
        )

        mask = cv2.inRange(
            hsv,
            lower_white,
            upper_white
        )


        # =====================================================
        # EDGE DETECTION
        # =====================================================

        edges = cv2.Canny(
            mask,
            50,
            150
        )


        # =====================================================
        # HOUGH LINE DETECTION
        # =====================================================

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=25,
            minLineLength=25,
            maxLineGap=25
        )


        left_lines = []
        divider_lines = []
        right_lines = []

        print(
            f"LEFT={len(left_lines)} "
            f"DIVIDER={len(divider_lines)} "
            f"RIGHT={len(right_lines)}"
        )


        if lines is not None:

            for line in lines:

                x1, y1, x2, y2 = line[0]

                # -------------------------------------------------
                # Determine TOP and BOTTOM endpoints
                # -------------------------------------------------

                if y1 < y2:

                    top_x = x1
                    top_y = y1

                    bottom_x = x2
                    bottom_y = y2

                else:

                    top_x = x2
                    top_y = y2

                    bottom_x = x1
                    bottom_y = y1


                # -------------------------------------------------
                # Horizontal movement from top to bottom
                # -------------------------------------------------

                x_delta = bottom_x - top_x


                # -------------------------------------------------
                # Average X position
                # -------------------------------------------------

                midpoint_x = (
                    top_x + bottom_x
                ) / 2.0


                # -------------------------------------------------
                # DEBUG
                # -------------------------------------------------

                print(
                    f"CLASSIFY: "
                    f"top=({top_x},{top_y}) "
                    f"bottom=({bottom_x},{bottom_y}) "
                    f"delta={x_delta:.1f} "
                    f"mid={midpoint_x:.1f}"
                )


                # =================================================
                # CENTER DIVIDER
                # =================================================

                if (
                    abs(x_delta) < 40
                    and
                    270 < midpoint_x < 370
                ):

                    divider_lines.append(
                        (x1, y1, x2, y2)
                    )


                # =================================================
                # LEFT ROAD BOUNDARY
                # =================================================

                elif (
                    x_delta < -40
                    and
                    midpoint_x < 300
                ):

                    left_lines.append(
                        (x1, y1, x2, y2)
                    )


                # =================================================
                # RIGHT ROAD BOUNDARY
                # =================================================

                elif (
                    x_delta > 40
                    and
                    midpoint_x > 340
                ):

                    right_lines.append(
                        (x1, y1, x2, y2)
                    )


        # =====================================================
        # LOOK-AHEAD POSITION
        # =====================================================

        # Measure lane position here.
        #
        # 0       = top of ROI
        # roi_h   = bottom of image

        lookahead_y = int(
            roi_height * 0.30
        )


        # =====================================================
        # FUNCTION: X POSITION OF LINE AT Y
        # =====================================================

        def x_at_y(line, target_y):

            x1, y1, x2, y2 = line

            if abs(y2 - y1) < 1:
                return None

            x = (
                x1
                +
                (target_y - y1)
                *
                (x2 - x1)
                /
                (y2 - y1)
            )

            return x

        # =====================================================
        # LEFT BOUNDARY POSITION
        # =====================================================

        left_boundary = None

        left_positions = []

        for line in left_lines:

            x = x_at_y(
                line,
                lookahead_y
            )

            if x is None:
                continue

            # Only accept points on the left side
            # of the camera image.

            if (
                0 <= x < image_center
            ):

                left_positions.append(
                    x
                )


        # -----------------------------------------------------
        # Use the median of all valid left-line positions
        # -----------------------------------------------------

        if len(left_positions) > 0:

            left_boundary = float(
                np.median(left_positions)
            )

            self.last_left_boundary = (
                left_boundary
            )


        # =====================================================
        # LEFT BOUNDARY LOSS RECOVERY
        # =====================================================

        if (
            left_boundary is None
            and
            self.last_left_boundary is not None
        ):

            left_boundary = (
                self.last_left_boundary
            )

        print(
            f"LEFT DEBUG | "
            f"lines={len(left_lines)} | "
            f"positions={left_positions} | "
            f"boundary={left_boundary}"
        )

        # =====================================================
        # DIVIDER POSITION
        # =====================================================

        divider_positions = []

        for line in divider_lines:

            x1, y1, x2, y2 = line

            # Center x-coordinate of each detected divider segment.
            # This works well for our approximately vertical
            # dashed center marking.
            segment_center_x = (
                x1 + x2
            ) / 2.0

            if (
                width * 0.35
                <
                segment_center_x
                <
                width * 0.65
            ):
                divider_positions.append(
                    segment_center_x
                )


        divider = None

        if divider_positions:

            divider = float(
                np.median(divider_positions)
            )

            self.last_divider = divider

        elif self.last_divider is not None:

            divider = self.last_divider


        # =====================================================
        # CALCULATE LEFT-LANE CENTER
        # =====================================================

        lane_center = None

        if (
            left_boundary is not None
            and
            divider is not None
        ):

            lane_center = (
                left_boundary
                +
                divider
            ) / 2.0


        # =====================================================
        # VISUALIZATION OF RAW HOUGH LINES
        # =====================================================

        display = frame.copy()


        # Left boundary Hough segments = RED
        for line in left_lines:

            x1, y1, x2, y2 = line

            cv2.line(
                display,
                (x1, y1 + roi_top),
                (x2, y2 + roi_top),
                (0, 0, 255),
                2
            )

        # Right boundary Hough segments = BLUE
        for line in right_lines:

            x1, y1, x2, y2 = line

            cv2.line(
                display,
                (x1, y1 + roi_top),
                (x2, y2 + roi_top),
                (255, 0, 0),
                3
            )


        # Divider Hough segments = YELLOW
        for line in divider_lines:

            x1, y1, x2, y2 = line

            cv2.line(
                display,
                (x1, y1 + roi_top),
                (x2, y2 + roi_top),
                (0, 255, 255),
                2
            )


        # =====================================================
        # LANE ERROR
        # =====================================================

        if lane_center is not None:

            # ---------------------------------------------
            # Smooth target
            # ---------------------------------------------

            if self.filtered_lane_center is None:

                self.filtered_lane_center = (
                    lane_center
                )

            else:

                self.filtered_lane_center = (
                    self.alpha
                    * lane_center
                    +
                    (1.0 - self.alpha)
                    * self.filtered_lane_center
                )


            target_x = int(
                self.filtered_lane_center
            )


            # ---------------------------------------------
            # Normalize error to approximately [-1, +1]
            # ---------------------------------------------

            # ---------------------------------------------
            # LATERAL ERROR
            # ---------------------------------------------

            error_pixels = (
                target_x
                -
                image_center
            )

            lateral_error = (
                error_pixels
                /
                (width / 2.0)
            )


            # =====================================================
            # HEADING ERROR
            # =====================================================

            heading_error = 0.0

            # We compare the lane center at two different
            # distances in front of the vehicle.

            near_y = int(roi_height * 0.35)
            far_y = int(roi_height * 0.80)

            near_left = []
            near_divider = []

            far_left = []
            far_divider = []


            # -----------------------------------------------------
            # LEFT BOUNDARY AT NEAR/FAR POSITIONS
            # -----------------------------------------------------

            for line in left_lines:

                near_x = x_at_y(
                    line,
                    near_y
                )

                far_x = x_at_y(
                    line,
                    far_y
                )

                if near_x is not None:
                    if 0 <= near_x < image_center:
                        near_left.append(near_x)

                if far_x is not None:
                    if 0 <= far_x < image_center:
                        far_left.append(far_x)


            # -----------------------------------------------------
            # DIVIDER AT NEAR/FAR POSITIONS
            # -----------------------------------------------------

            for line in divider_lines:

                near_x = x_at_y(
                    line,
                    near_y
                )

                far_x = x_at_y(
                    line,
                    far_y
                )

                if near_x is not None:
                    if width * 0.25 < near_x < width * 0.75:
                        near_divider.append(near_x)

                if far_x is not None:
                    if width * 0.25 < far_x < width * 0.75:
                        far_divider.append(far_x)


            # -----------------------------------------------------
            # CALCULATE NEAR/FAR LANE CENTERS
            # -----------------------------------------------------

            if (
                len(near_left) > 0
                and
                len(near_divider) > 0
                and
                len(far_left) > 0
                and
                len(far_divider) > 0
            ):

                near_lane_center = (
                    np.median(near_left)
                    +
                    np.median(near_divider)
                ) / 2.0

                far_lane_center = (
                    np.median(far_left)
                    +
                    np.median(far_divider)
                ) / 2.0


                # -------------------------------------------------
                # LANE DIRECTION
                # -------------------------------------------------

                lane_direction = (
                    far_lane_center
                    -
                    near_lane_center
                )


                # Normalize heading error
                heading_error = (
                    lane_direction
                    /
                    (width / 2.0)
                )


            # =====================================================
            # COMBINE LATERAL + HEADING ERROR
            # =====================================================

            normalized_error = (
                0.70 * lateral_error
                +
                0.30 * heading_error
            )


            # ---------------------------------------------
            # Publish
            # ---------------------------------------------

            error_msg = Float64()

            error_msg.data = float(
                normalized_error
            )

            self.error_publisher.publish(
                error_msg
            )


            # =============================================
            # DRAW DETECTIONS
            # =============================================

            global_y = (
                roi_top
                +
                lookahead_y
            )


            # Look-ahead horizontal line
            cv2.line(
                display,
                (0, global_y),
                (width, global_y),
                (255, 255, 0),
                1
            )


            # Camera center = BLUE
            cv2.line(
                display,
                (image_center, height),
                (image_center, roi_top),
                (255, 0, 0),
                2
            )


            # Left boundary point = RED
            cv2.circle(
                display,
                (
                    int(left_boundary),
                    global_y
                ),
                8,
                (0, 0, 255),
                -1
            )


            # Divider point = YELLOW
            cv2.circle(
                display,
                (
                    int(divider),
                    global_y
                ),
                8,
                (0, 255, 255),
                -1
            )


            # Desired lane center = GREEN
            cv2.circle(
                display,
                (
                    target_x,
                    global_y
                ),
                10,
                (0, 255, 0),
                -1
            )


            cv2.line(
                display,
                (
                    target_x,
                    height
                ),
                (
                    target_x,
                    global_y
                ),
                (0, 255, 0),
                3
            )


            cv2.putText(
                display,
                'LEFT LANE DETECTED',
                (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )


            cv2.putText(
                display,
                f'Lane error: {normalized_error:.3f}',
                (15, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.putText(
                display,
                f'Lat: {lateral_error:.3f}  '
                f'Head: {heading_error:.3f}',
                (15, 140),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )


            cv2.putText(
                display,
                (
                    f'L:{left_boundary:.0f}  '
                    f'C:{target_x}  '
                    f'D:{divider:.0f}'
                ),
                (15, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )


        else:

            cv2.putText(
                display,
                'LEFT LANE NOT DETECTED',
                (15, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )


        # =====================================================
        # ROI BOUNDARY
        # =====================================================

        cv2.line(
            display,
            (0, roi_top),
            (width, roi_top),
            (255, 0, 255),
            1
        )


        # =====================================================
        # WINDOWS
        # =====================================================

        cv2.imshow(
            'Autonomous Car - Lane Detection V3',
            display
        )

        cv2.imshow(
            'White Lane Mask',
            mask
        )

        cv2.imshow(
            'Lane Edges',
            edges
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = LaneDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()