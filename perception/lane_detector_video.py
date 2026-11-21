import cv2
import numpy as np
import sys
import time


# ============================================================
# ROAD BOUNDARY + FUTURE PATH DETECTOR
#
# BLUE   = LEFT ROAD BOUNDARY
# RED    = RIGHT ROAD BOUNDARY
# YELLOW = FUTURE VEHICLE PATH
# GREEN  = ESTIMATED DRIVABLE AREA
#
# IMPORTANT:
#   Left and right boundaries are ALWAYS straight.
#   Only the yellow center trajectory is curved.
# ============================================================


# ============================================================
# VIDEO PATH
# ============================================================

VIDEO_PATH = "test_videos/test4.mp4"

if len(sys.argv) > 1:
    VIDEO_PATH = sys.argv[1]


# ============================================================
# OPEN VIDEO
# ============================================================

cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():

    print("ERROR: Could not open video:")
    print(VIDEO_PATH)

    sys.exit(1)


# ============================================================
# VIDEO INFORMATION
# ============================================================

fps = cap.get(cv2.CAP_PROP_FPS)

if fps <= 0:
    fps = 30.0


width = int(
    cap.get(cv2.CAP_PROP_FRAME_WIDTH)
)

height = int(
    cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
)

total_frames = int(
    cap.get(cv2.CAP_PROP_FRAME_COUNT)
)


print()
print("============================================")
print(" ROAD BOUNDARY + FUTURE PATH DETECTOR")
print("============================================")
print(f"Video      : {VIDEO_PATH}")
print(f"Resolution : {width} x {height}")
print(f"FPS        : {fps:.2f}")
print(f"Frames     : {total_frames}")
print(f"Duration   : {total_frames / fps:.2f} seconds")
print("============================================")
print()


# ============================================================
# CALIBRATION
# ============================================================

# Ignore the upper part of the image.
#
# Change these if your camera/video has a different view.

ROI_TOP = 0.42


# Region used for fitting the road boundaries.

BOUNDARY_TOP = 0.48

BOUNDARY_BOTTOM = 0.97


# ============================================================
# BOUNDARY SMOOTHING
# ============================================================

# Smaller value:
#   smoother but slower reaction
#
# Larger value:
#   faster reaction but more jumping

SMOOTHING_ALPHA = 0.12


previous_left = None
previous_right = None


# ============================================================
# LOST BOUNDARY MEMORY
# ============================================================

left_lost_frames = 0
right_lost_frames = 0

MAX_LOST_FRAMES = 20


# ============================================================
# FUTURE PATH CALIBRATION
# ============================================================

# Number of points used to draw yellow path.

PATH_POINTS = 80


# Strength of the curvature.
#
# 0.0 = almost straight
# 1.0 = strong curvature
#
# Start around 0.45-0.65.

CURVE_STRENGTH = 0.55


# Minimum safe distance between yellow path
# and road boundaries.

PATH_MARGIN = 25


# ============================================================
# BRIGHT ROAD MARKING DETECTION
# ============================================================

LOWER_BRIGHT = np.array(
    [0, 0, 145],
    dtype=np.uint8
)

UPPER_BRIGHT = np.array(
    [180, 110, 255],
    dtype=np.uint8
)


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    frame_start_time = time.perf_counter()


    # ========================================================
    # READ FRAME
    # ========================================================

    ret, frame = cap.read()

    if not ret:
        break


    # ========================================================
    # ORIGINAL COLOR IMAGE
    # ========================================================

    output = frame.copy()


    # ========================================================
    # REGION OF INTEREST
    # ========================================================

    roi_start = int(
        height * ROI_TOP
    )

    roi = frame[
        roi_start:height,
        :
    ]


    # ========================================================
    # HSV
    # ========================================================

    hsv = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2HSV
    )


    # ========================================================
    # BRIGHT PIXELS
    # ========================================================

    mask = cv2.inRange(
        hsv,
        LOWER_BRIGHT,
        UPPER_BRIGHT
    )


    # ========================================================
    # MORPHOLOGICAL CLEANUP
    # ========================================================

    kernel = np.ones(
        (5, 5),
        np.uint8
    )


    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )


    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )


    # ========================================================
    # EDGE DETECTION
    # ========================================================

    edges = cv2.Canny(
        mask,
        50,
        150
    )


    # ========================================================
    # HOUGH LINE DETECTION
    # ========================================================

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=35,
        minLineLength=45,
        maxLineGap=100
    )


    left_lines = []
    right_lines = []


    # ========================================================
    # CLASSIFY DETECTED LINES
    # ========================================================

    if lines is not None:

        for line in lines:

            x1, y1, x2, y2 = line[0]


            # Convert ROI coordinates
            # to full image coordinates.

            y1 += roi_start
            y2 += roi_start


            dx = x2 - x1
            dy = y2 - y1


            # Ignore nearly vertical lines.

            if abs(dx) < 5:
                continue


            slope = dy / dx


            # Ignore nearly horizontal lines.

            if abs(slope) < 0.30:
                continue


            midpoint_x = (
                x1 + x2
            ) / 2.0


            # =================================================
            # LEFT SIDE
            # =================================================

            if (
                slope < 0
                and
                midpoint_x < width * 0.58
            ):

                left_lines.append(
                    (
                        x1,
                        y1,
                        x2,
                        y2
                    )
                )


            # =================================================
            # RIGHT SIDE
            # =================================================

            elif (
                slope > 0
                and
                midpoint_x > width * 0.42
            ):

                right_lines.append(
                    (
                        x1,
                        y1,
                        x2,
                        y2
                    )
                )


    # ========================================================
    # LEFT STRAIGHT LINE FIT
    # ========================================================

    left_fit = None


    if len(left_lines) >= 2:

        points = []


        for x1, y1, x2, y2 in left_lines:

            points.append(
                [x1, y1]
            )

            points.append(
                [x2, y2]
            )


        points = np.array(
            points,
            dtype=np.float32
        )


        try:

            # x = m*y + c
            #
            # Degree 1 = STRAIGHT LINE

            left_fit = np.polyfit(
                points[:, 1],
                points[:, 0],
                1
            )

        except Exception:

            left_fit = None


    # ========================================================
    # RIGHT STRAIGHT LINE FIT
    # ========================================================

    right_fit = None


    if len(right_lines) >= 2:

        points = []


        for x1, y1, x2, y2 in right_lines:

            points.append(
                [x1, y1]
            )

            points.append(
                [x2, y2]
            )


        points = np.array(
            points,
            dtype=np.float32
        )


        try:

            # x = m*y + c
            #
            # Degree 1 = STRAIGHT LINE

            right_fit = np.polyfit(
                points[:, 1],
                points[:, 0],
                1
            )

        except Exception:

            right_fit = None


    # ========================================================
    # SMOOTH LEFT BOUNDARY
    # ========================================================

    if left_fit is not None:

        if previous_left is None:

            previous_left = left_fit

        else:

            previous_left = (
                SMOOTHING_ALPHA * left_fit
                +
                (1.0 - SMOOTHING_ALPHA)
                *
                previous_left
            )


        left_lost_frames = 0


    else:

        left_lost_frames += 1


    # ========================================================
    # SMOOTH RIGHT BOUNDARY
    # ========================================================

    if right_fit is not None:

        if previous_right is None:

            previous_right = right_fit

        else:

            previous_right = (
                SMOOTHING_ALPHA * right_fit
                +
                (1.0 - SMOOTHING_ALPHA)
                *
                previous_right
            )


        right_lost_frames = 0


    else:

        right_lost_frames += 1


    # ========================================================
    # FORGET BADLY LOST BOUNDARIES
    # ========================================================

    if left_lost_frames > MAX_LOST_FRAMES:

        previous_left = None


    if right_lost_frames > MAX_LOST_FRAMES:

        previous_right = None


    # ========================================================
    # DRAW ROAD
    # ========================================================

    if (
        previous_left is not None
        and
        previous_right is not None
    ):


        # ====================================================
        # CALIBRATED TOP AND BOTTOM
        # ====================================================

        y_top = int(
            height * BOUNDARY_TOP
        )


        y_bottom = int(
            height * BOUNDARY_BOTTOM
        )


        # ====================================================
        # LEFT STRAIGHT BOUNDARY
        # ====================================================

        left_x_top = np.polyval(
            previous_left,
            y_top
        )


        left_x_bottom = np.polyval(
            previous_left,
            y_bottom
        )


        # ====================================================
        # RIGHT STRAIGHT BOUNDARY
        # ====================================================

        right_x_top = np.polyval(
            previous_right,
            y_top
        )


        right_x_bottom = np.polyval(
            previous_right,
            y_bottom
        )


        # ====================================================
        # SANITY CHECK
        # ====================================================

        valid_road = (

            right_x_bottom
            >
            left_x_bottom

            and

            right_x_top
            >
            left_x_top
        )


        if valid_road:


            # =================================================
            # BLUE LEFT BOUNDARY
            # =================================================

            cv2.line(
                output,

                (
                    int(left_x_top),
                    y_top
                ),

                (
                    int(left_x_bottom),
                    y_bottom
                ),

                (255, 0, 0),

                5
            )


            # =================================================
            # RED RIGHT BOUNDARY
            # =================================================

            cv2.line(
                output,

                (
                    int(right_x_top),
                    y_top
                ),

                (
                    int(right_x_bottom),
                    y_bottom
                ),

                (0, 0, 255),

                5
            )


            # =================================================
            # GREEN DRIVABLE AREA
            # =================================================

            road_polygon = np.array(
                [
                    [
                        int(left_x_top),
                        y_top
                    ],

                    [
                        int(right_x_top),
                        y_top
                    ],

                    [
                        int(right_x_bottom),
                        y_bottom
                    ],

                    [
                        int(left_x_bottom),
                        y_bottom
                    ]
                ],
                dtype=np.int32
            )


            overlay = output.copy()


            cv2.fillPoly(
                overlay,
                [road_polygon],
                (0, 180, 0)
            )


            output = cv2.addWeighted(
                overlay,
                0.22,
                output,
                0.78,
                0
            )


            # =================================================
            # ROAD CENTER AT VEHICLE
            # =================================================

            center_bottom = (
                left_x_bottom
                +
                right_x_bottom
            ) / 2.0


            # =================================================
            # ROAD CENTER AT HORIZON
            # =================================================

            center_top = (
                left_x_top
                +
                right_x_top
            ) / 2.0


            # =================================================
            # ROAD DIRECTION
            # =================================================

            road_direction = (
                center_top
                -
                center_bottom
            )


            # =================================================
            # GENERATE Y VALUES
            # =================================================

            path_y = np.linspace(
                y_bottom,
                y_top,
                PATH_POINTS
            )


            # =================================================
            # NORMALIZED DISTANCE
            #
            # 0 = vehicle
            # 1 = far ahead
            # =================================================

            path_t = (

                y_bottom
                -
                path_y

            ) / (

                y_bottom
                -
                y_top
            )


            # =================================================
            # STRAIGHT CENTER REFERENCE
            # =================================================

            base_center = (

                center_bottom

                +

                (
                    center_top
                    -
                    center_bottom
                )

                *

                path_t
            )


            # =================================================
            # CURVED FUTURE TRAJECTORY
            # =================================================
            #
            # IMPORTANT:
            #
            # This is the ONLY place where curvature
            # is introduced.
            #
            # Blue = straight
            # Red  = straight
            # Yellow = curved
            #
            # The curve starts at the vehicle and becomes
            # stronger farther ahead.
            # =================================================

            curve_offset = (

                road_direction

                *

                CURVE_STRENGTH

                *

                (
                    path_t ** 2
                )
            )


            path_x = (

                base_center
                +
                curve_offset
            )


            # =================================================
            # CALCULATE BOUNDARY POSITIONS ALONG PATH
            # =================================================

            left_boundary_x = (

                previous_left[0]
                *
                path_y
                +
                previous_left[1]
            )


            right_boundary_x = (

                previous_right[0]
                *
                path_y
                +
                previous_right[1]
            )


            # =================================================
            # KEEP YELLOW PATH INSIDE ROAD
            # =================================================

            path_x = np.maximum(

                path_x,

                left_boundary_x
                +
                PATH_MARGIN
            )


            path_x = np.minimum(

                path_x,

                right_boundary_x
                -
                PATH_MARGIN
            )


            # =================================================
            # CREATE OPENCV PATH
            # =================================================

            future_path = []


            for x, y in zip(
                path_x,
                path_y
            ):

                x = int(
                    np.clip(
                        x,
                        0,
                        width - 1
                    )
                )


                y = int(
                    np.clip(
                        y,
                        0,
                        height - 1
                    )
                )


                future_path.append(
                    (x, y)
                )


            future_path = np.array(
                future_path,
                dtype=np.int32
            )


            # =================================================
            # YELLOW FUTURE VEHICLE PATH
            # =================================================

            cv2.polylines(
                output,

                [
                    future_path
                ],

                False,

                (0, 255, 255),

                5,

                cv2.LINE_AA
            )


            # =================================================
            # CURRENT VEHICLE CENTER
            # =================================================

            cv2.circle(
                output,

                (
                    int(center_bottom),
                    int(y_bottom)
                ),

                8,

                (0, 255, 255),

                -1
            )


            # =================================================
            # FUTURE DIRECTION ARROW
            # =================================================

            if len(future_path) >= 25:

                arrow_start = tuple(
                    future_path[-30]
                )


                arrow_end = tuple(
                    future_path[-1]
                )


                cv2.arrowedLine(
                    output,

                    arrow_start,

                    arrow_end,

                    (0, 255, 255),

                    4,

                    cv2.LINE_AA,

                    tipLength=0.20
                )


    # ========================================================
    # INFORMATION
    # ========================================================

    cv2.putText(

        output,

        f"FPS: {fps:.1f}",

        (20, 35),

        cv2.FONT_HERSHEY_SIMPLEX,

        0.70,

        (255, 255, 255),

        2,

        cv2.LINE_AA
    )


    cv2.putText(

        output,

        "BLUE  = LEFT BOUNDARY",

        (20, 65),

        cv2.FONT_HERSHEY_SIMPLEX,

        0.55,

        (255, 255, 255),

        2,

        cv2.LINE_AA
    )


    cv2.putText(

        output,

        "RED   = RIGHT BOUNDARY",

        (20, 90),

        cv2.FONT_HERSHEY_SIMPLEX,

        0.55,

        (255, 255, 255),

        2,

        cv2.LINE_AA
    )


    cv2.putText(

        output,

        "YELLOW = FUTURE PATH",

        (20, 115),

        cv2.FONT_HERSHEY_SIMPLEX,

        0.55,

        (255, 255, 255),

        2,

        cv2.LINE_AA
    )


    # ========================================================
    # DISPLAY
    # ========================================================

    cv2.imshow(
        "Road Boundary + Future Vehicle Path",
        output
    )


    # ========================================================
    # MAINTAIN ORIGINAL VIDEO SPEED
    # ========================================================

    processing_time = (
        time.perf_counter()
        -
        frame_start_time
    )


    remaining_time = (
        (1.0 / fps)
        -
        processing_time
    )


    if remaining_time > 0:

        time.sleep(
            remaining_time
        )


    # ========================================================
    # KEYBOARD
    # ========================================================

    key = cv2.waitKey(1) & 0xFF


    if key == ord("q"):

        break


# ============================================================
# CLEANUP
# ============================================================

cap.release()

cv2.destroyAllWindows()


print()
print("Video processing finished.")