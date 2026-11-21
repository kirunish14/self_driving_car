#!/usr/bin/env python3
"""
indoor_lane.py - corridor path detection for the indoor videos (Indoor.mp4, Indoor1.mp4).
No painted lines needed.

For every frame it draws
    LEFT  boundary line   (red)     where the left wall meets the floor
    RIGHT boundary line   (blue)    where the right wall meets the floor
    yellow dot                      the far end of the corridor (vanishing point)
    green area                      the floor between the two lines (where the car can drive)
    blue ribbon                     the CENTRE PATH the car should follow
and prints a steering value (-1 = full left ... +1 = full right).

How it works
  1. Vanishing point: ceiling lines, door frames, wall panels and floor edges all converge at the far
     end of a corridor. Every long straight line in the picture "votes" for the point it passes
     through; the point with most votes is the end of the corridor.
  2. Wall/floor lines: from that point, rays are sent to the lower left and lower right of the picture.
     The ray along which the brightness step between the two sides is strongest and most consistent
     is the wall base (floor <-> wall).
  3. Both results are smoothed over time so the lines do not jump.
  4. The centre path runs between the two lines to a point ahead; steering = how far that point is
     from the car's centre.

Usage
    python3 indoor_lane.py --source Indoor.mp4
    python3 indoor_lane.py --source Indoor1.mp4 --save result.mp4 --no-window
    python3 indoor_lane.py --source 0                       # live camera
Keys (window): q quit | space pause | n next frame
"""
import argparse
import time

import cv2
import numpy as np


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class IndoorLane:
    def __init__(self, car_x=0.5, hood=0.0):
        self.car_x = car_x              # where the car's centre is in the picture (fraction of width)
        self.hood = hood                # row (fraction of height) where the car's bonnet starts, 0 = not visible
        self.lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
        self.reset()

    def reset(self):
        self.vp = None                  # smoothed vanishing point (x, y) in pixels
        self.ang = {"L": None, "R": None}   # smoothed wall-base angles (radians below the horizontal)
        self.last_steer = 0.0

    # ----------------------------------------------------------- vanishing point
    def _vanishing_point(self, gray):
        h, w = gray.shape
        segs = self.lsd.detect(gray)[0]
        if segs is None:
            return None
        s = segs.reshape(-1, 4)
        length = np.hypot(s[:, 2] - s[:, 0], s[:, 3] - s[:, 1])
        ang = np.degrees(np.arctan2(s[:, 3] - s[:, 1], s[:, 2] - s[:, 0])) % 180
        # long lines that are neither horizontal nor vertical
        ok = (length > 0.05 * w) & (np.minimum(ang, 180 - ang) > 8) & (np.abs(ang - 90) > 8)
        s, length = s[ok], length[ok]
        if len(s) < 6:
            return None
        gx = np.linspace(0.25 * w, 0.75 * w, 50)
        gy = np.linspace(0.20 * h, 0.60 * h, 40)
        GX, GY = np.meshgrid(gx, gy)
        votes = np.zeros_like(GX)
        for (x1, y1, x2, y2), ln in zip(s, length):
            dx, dy = x2 - x1, y2 - y1
            dist = np.abs((GX - x1) * dy - (GY - y1) * dx) / np.hypot(dx, dy)    # grid point to the line
            votes += ln * np.exp(-(dist / 6.0) ** 2)
        votes = cv2.GaussianBlur(votes.astype(np.float32), (5, 5), 0)
        iy, ix = np.unravel_index(np.argmax(votes), votes.shape)
        return np.array([gx[ix], gy[iy]])

    # --------------------------------------------------------------- wall bases
    @staticmethod
    def _ray_score(g, vx, vy, ex, ey):
        """How strong and how consistent the brightness step is along the ray (vx,vy) -> (ex,ey)."""
        h, w = g.shape
        n = np.hypot(ex - vx, ey - vy)
        t = np.linspace(0.12, 0.95, 40)
        px, py = vx + (ex - vx) * t, vy + (ey - vy) * t
        nx, ny = -(ey - vy) / n, (ex - vx) / n          # unit normal of the ray

        def sample(x, y):
            return g[np.clip(y, 0, h - 1).astype(int), np.clip(x, 0, w - 1).astype(int)].astype(float)

        side_a = np.mean([sample(px + nx * o, py + ny * o) for o in (4, 7, 10)], axis=0)
        side_b = np.mean([sample(px - nx * o, py - ny * o) for o in (4, 7, 10)], axis=0)
        d = side_a - side_b
        med = np.median(d)
        return abs(med) * np.mean(np.sign(d) == np.sign(med))     # a real wall/floor line: same sign all along

    def _wall_bases(self, gray, vx, vy):
        h, w = gray.shape
        g = cv2.GaussianBlur(gray, (5, 5), 0)
        found = {}
        for side in ("L", "R"):
            ends = []
            if side == "L":
                ends += [(0, y) for y in np.linspace(vy + 0.12 * h, h - 1, 45)]
                ends += [(x, h - 1) for x in np.linspace(0, vx - 0.10 * w, 45)]
            else:
                ends += [(w - 1, y) for y in np.linspace(vy + 0.12 * h, h - 1, 45)]
                ends += [(x, h - 1) for x in np.linspace(vx + 0.10 * w, w - 1, 45)]
            best, best_ang = -1.0, None
            for ex, ey in ends:
                a = np.degrees(np.arctan2(ey - vy, abs(ex - vx)))      # angle below the horizontal
                if not (6 <= a <= 50):                                 # corridor wall bases are not steep
                    continue
                sc = self._ray_score(g, vx, vy, ex, ey)
                if sc > best:
                    best, best_ang = sc, np.radians(a)
            found[side] = best_ang
        return found

    # ------------------------------------------------------------------ process
    def process(self, frame, draw=True):
        h, w = frame.shape[:2]
        self.s = w / 960.0
        car = self.car_x * w
        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (3, 3), 0)

        # 1. vanishing point (smoothed; a sensible start before the first one is found)
        vp = self._vanishing_point(gray)
        if self.vp is None:
            self.vp = vp if vp is not None else np.array([0.5 * w, 0.42 * h])
        elif vp is not None:
            jump = abs(vp[0] - self.vp[0]) / w
            a = 0.2 if jump < 0.15 else 0.08                 # a sudden big jump is probably a wrong vote
            self.vp = (1 - a) * self.vp + a * vp
        xv, yv = float(self.vp[0]), float(self.vp[1])

        # 2. left and right wall/floor lines
        found = self._wall_bases(gray, xv, yv)
        for side in ("L", "R"):
            if found[side] is None:
                continue
            cur = self.ang[side]
            self.ang[side] = found[side] if cur is None else 0.7 * cur + 0.3 * found[side]
        info = {"valid": False, "lines": 0, "steer": self.last_steer, "offset_near": 0.0, "offset_far": 0.0,
                "curve": 0.0, "lane_width": 0.0, "text": "looking for the corridor"}
        if self.ang["L"] is None or self.ang["R"] is None:
            return (frame.copy() if draw else None), info

        kL = -1.0 / np.tan(self.ang["L"])                  # x = xv + k (y - yv): left line goes left as y grows
        kR = 1.0 / np.tan(self.ang["R"])

        def xl(y):
            return xv + kL * (y - yv)

        def xr(y):
            return xv + kR * (y - yv)

        # 3. centre path and steering
        ybot = float(h - 1) if self.hood <= 0 else self.hood * h
        y_t = max(yv + 0.12 * h, 0.62 * h)                  # look-ahead row
        x_t = clamp(0.5 * (xl(y_t) + xr(y_t)), car - 0.25 * w, car + 0.25 * w)
        y_n = 0.92 * h
        half_n = 0.5 * (xr(y_n) - xl(y_n))
        e_near = (0.5 * (xl(y_n) + xr(y_n)) - car) / max(half_n, 0.25 * w)
        steer = float(clamp(2.0 * (x_t - car) / w, -1, 1))
        self.last_steer = steer
        action = "steer RIGHT" if steer > 0.08 else "steer LEFT" if steer < -0.08 else "keep"
        info.update(valid=True, lines=2, steer=steer, offset_near=float(clamp(e_near, -3, 3)),
                    offset_far=float((x_t - car) / (0.5 * w)), curve=float((xv - car) / (0.5 * w)),
                    lane_width=float((xr(y_n) - xl(y_n)) / w), text=f"{action} {steer:+.2f}")
        if not draw:
            return None, info
        return self._render(frame, xv, yv, xl, xr, car, ybot, x_t, y_t, info), info

    # --------------------------------------------------------------------- draw
    def _render(self, frame, xv, yv, xl, xr, car, ybot, x_t, y_t, info):
        h, w = frame.shape[:2]
        s = self.s
        out = frame.copy()
        th = max(2, int(round(4 * s)))
        # floor between the two lines
        poly = np.array([[clamp(xl(ybot), -2 * w, 3 * w), ybot], [xv, yv],
                         [clamp(xr(ybot), -2 * w, 3 * w), ybot]]).astype(np.int32)
        layer = np.zeros_like(out)
        cv2.fillPoly(layer, [poly], (0, 160, 0))
        out = cv2.addWeighted(out, 1.0, layer, 0.25, 0)
        # left (red) and right (blue) boundary lines
        for fx, col in ((xl, (0, 0, 255)), (xr, (255, 60, 0))):
            pts = [(int(fx(y)), int(y)) for y in np.linspace(yv, ybot, 60) if -50 < fx(y) < w + 50]
            if len(pts) > 1:
                cv2.polylines(out, [np.array(pts, np.int32)], False, col, th, cv2.LINE_AA)
        cv2.circle(out, (int(xv), int(yv)), max(4, int(6 * s)), (0, 255, 255), -1)
        # centre path: a smooth curve from the car, starting straight ahead, to the look-ahead point
        p0 = np.array([car, ybot])
        p1 = np.array([car, 0.5 * (ybot + y_t)])
        p2 = np.array([x_t, y_t])
        t = np.linspace(0, 1, 16)[:, None]
        pts = (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2
        xp, ys = pts[:, 0], pts[:, 1]
        half = 0.085 * w * (1 - 0.6 * t[:, 0])
        ribbon = np.zeros_like(out)
        pm = np.zeros(out.shape[:2], np.uint8)
        for i in range(len(ys) - 1):
            q = np.array([[xp[i] - half[i], ys[i]], [xp[i] + half[i], ys[i]],
                          [xp[i + 1] + half[i + 1], ys[i + 1]], [xp[i + 1] - half[i + 1], ys[i + 1]]]).astype(np.int32)
            shade = 1 - 0.5 * i / (len(ys) - 1)
            cv2.fillConvexPoly(ribbon, q, (int(255 * shade), int(180 * shade), 0))
            cv2.fillConvexPoly(pm, q, 255)
        blended = cv2.addWeighted(out, 0.35, ribbon, 0.65, 0)
        out[pm > 0] = blended[pm > 0]
        cv2.putText(out, info["text"], (int(20 * s), int(40 * s)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9 * s, (255, 255, 255), max(1, int(2 * s)), cv2.LINE_AA)
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0", help="video file or camera number")
    ap.add_argument("--save", default=None, help="write the result to this video file")
    ap.add_argument("--no-window", action="store_true", help="do not open a window (faster on the Pi)")
    ap.add_argument("--width", type=int, default=640, help="processing width (smaller = faster)")
    ap.add_argument("--hood", type=float, default=0.0, help="row (0-1) where the bonnet starts, 0 = not visible")
    ap.add_argument("--car-x", type=float, default=0.5, help="car centre in the picture (0-1)")
    ap.add_argument("--fps", type=float, default=None, help="output fps (default: same as the video)")
    ap.add_argument("--print", action="store_true", help="print the steering value of every frame")
    args = ap.parse_args()

    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {args.source}")
    fps = args.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0
    det = IndoorLane(car_x=clamp(args.car_x, 0.2, 0.8), hood=clamp(args.hood, 0.0, 0.95))
    writer = None
    n, t_sum, paused, out = 0, 0.0, False, None

    def prep(fr):
        h0, w0 = fr.shape[:2]
        return fr if w0 == args.width else cv2.resize(fr, (args.width, int(round(h0 * args.width / w0))))

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                break
            frame = prep(frame)
            t0 = time.time()
            out, info = det.process(frame)
            t_sum += time.time() - t0
            n += 1
            if args.print:
                print(f"frame {n}: steer {info['steer']:+.2f}  offset {info['offset_near']:+.2f}")
            if args.save:
                if writer is None:
                    writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), fps,
                                             (out.shape[1], out.shape[0]))
                writer.write(out)
        if not args.no_window and out is not None:
            cv2.imshow("indoor lane", out)
            k = cv2.waitKey(1 if not paused else 30) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" "):
                paused = not paused
            if k == ord("n"):
                paused = True
                ok, frame = cap.read()
                if ok:
                    out, info = det.process(prep(frame))
    if writer:
        writer.release()
    cap.release()
    cv2.destroyAllWindows()
    if n:
        print(f"{n} frames processed, {1000 * t_sum / n:.1f} ms per frame")


if __name__ == "__main__":
    main()
