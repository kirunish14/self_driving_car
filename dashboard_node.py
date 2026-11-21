#!/usr/bin/env python3
"""
dashboard_node.py - browser dashboard for the car (phone / tablet / laptop).

Runs on the car computer next to your existing stack and only TALKS to it, the same way RViz does:
    reads     TF map->base_footprint, /scan, /plan, /odom, navigate_to_pose status
    sends     /initialpose, NavigateToPose goals, cancel, /map_server/load_map
    shows     the map, lidar, car, path, goal and (optional) the indoor-lane camera video

No extra packages: a tiny web server (python standard library) serves the page, the map image,
live data (Server-Sent Events) and the video (MJPEG).  Open  http://<car-ip>:8080

It does not change any navigation / driving logic.
"""
import hashlib
import hmac
import json
import math
import mimetypes
import os
import queue
import re
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import yaml
import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from tf2_ros import Buffer, TransformListener


# ============================================================================ helpers
class ApiError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_pgm(path):
    """Binary PGM (P5, 8 bit) -> (width, height, bytes)."""
    with open(path, 'rb') as f:
        raw = f.read()
    pos, tokens = 0, []
    while len(tokens) < 4:
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':                       # comment line
            while pos < len(raw) and raw[pos:pos + 1] not in (b'\n', b'\r'):
                pos += 1
            continue
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        tokens.append(raw[start:pos])
    pos += 1                                               # the single whitespace after maxval
    if tokens[0] != b'P5' or int(tokens[3]) > 255:
        raise ValueError('only 8 bit binary PGM (P5) maps are supported')
    w, h = int(tokens[1]), int(tokens[2])
    data = raw[pos:pos + w * h]
    if len(data) < w * h:
        raise ValueError('PGM file is truncated')
    return w, h, data


def encode_png_gray(w, h, data):
    def chunk(tag, body):
        c = struct.pack('>I', len(body)) + tag + body
        return c + struct.pack('>I', zlib.crc32(tag + body) & 0xffffffff)
    rows = b''.join(b'\x00' + data[y * w:(y + 1) * w] for y in range(h))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(rows, 6)) + chunk(b'IEND', b''))


# ============================================================================ maps
class MapRegistry:
    """Finds the map .yaml files and turns the .pgm into a PNG the browser can draw."""

    def __init__(self, dirs, extra=()):
        self.dirs = [os.path.abspath(os.path.expanduser(d)) for d in dirs if d]
        self.extra = [os.path.abspath(os.path.expanduser(p)) for p in extra if p]
        self.entries = {}
        self._render = {}
        self._lock = threading.Lock()
        self.scan()

    @staticmethod
    def make_id(path):
        return hashlib.sha1(path.encode()).hexdigest()[:10]

    @staticmethod
    def _read_yaml(path):
        with open(path) as f:
            d = yaml.safe_load(f)
        if not isinstance(d, dict) or 'image' not in d or 'resolution' not in d:
            raise ValueError('not a map yaml')
        image = os.path.expanduser(str(d['image']))
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(path), image)
        origin = [float(v) for v in (d.get('origin') or [0, 0, 0])][:3]
        origin += [0.0] * (3 - len(origin))
        return {'id': MapRegistry.make_id(path), 'name': os.path.splitext(os.path.basename(path))[0],
                'path': path, 'image': image, 'resolution': float(d['resolution']), 'origin': origin,
                'negate': int(d.get('negate', 0)), 'occ': float(d.get('occupied_thresh', 0.65)),
                'free': float(d.get('free_thresh', 0.25))}

    def scan(self):
        found = {}
        candidates = []
        for d in self.dirs:
            if os.path.isdir(d):
                candidates += [os.path.join(d, n) for n in sorted(os.listdir(d)) if n.endswith('.yaml')]
        candidates += [p for p in self.extra if os.path.isfile(p)]
        for p in candidates:
            try:
                e = self._read_yaml(p)
            except Exception:
                continue
            found.setdefault(e['id'], e)
        self.entries = found

    def get(self, map_id):
        if map_id not in self.entries:
            self.scan()
        return self.entries.get(map_id)

    def render(self, map_id):
        e = self.get(map_id)
        if e is None:
            raise ApiError(404, 'unknown map')
        with self._lock:
            try:
                stamp = os.path.getmtime(e['image'])
            except OSError:
                raise ApiError(404, 'map image file not found: ' + e['image'])
            cached = self._render.get(map_id)
            if cached and cached['stamp'] == stamp:
                return cached
            try:
                if e['image'].lower().endswith('.pgm'):
                    w, h, data = read_pgm(e['image'])
                    lut = bytearray(256)
                    for p in range(256):
                        occ = p / 255.0 if e['negate'] else (255 - p) / 255.0
                        if occ > e['occ']:
                            lut[p] = 0                       # wall / obstacle
                        elif p == 205 and not e['negate']:
                            lut[p] = 205                     # the "unknown" value map_saver writes: draw it grey
                        else:
                            lut[p] = 254 if occ < e['free'] else 205
                    png = encode_png_gray(w, h, data.translate(bytes(lut)))
                elif e['image'].lower().endswith('.png'):
                    with open(e['image'], 'rb') as f:
                        png = f.read()
                    w, h = struct.unpack('>II', png[16:24])
                else:
                    raise ValueError('unsupported map image type')
            except ApiError:
                raise
            except Exception as ex:
                raise ApiError(500, f'cannot read map image: {ex}')
            cached = {'stamp': stamp, 'png': png, 'w': w, 'h': h}
            self._render[map_id] = cached
            return cached

    def meta(self, map_id):
        r = self.render(map_id)
        e = self.get(map_id)
        return {'id': e['id'], 'name': e['name'], 'width': r['w'], 'height': r['h'],
                'resolution': e['resolution'], 'origin': e['origin'],
                'image_url': f"/api/map/{e['id']}/image.png"}


# ============================================================================ live data
class EventHub:
    """Server-Sent Events: every browser gets its own queue; the newest message of each kind is replayed on connect."""

    def __init__(self):
        self._lock = threading.Lock()
        self._clients = set()
        self._last = {}

    def subscribe(self):
        q = queue.Queue(maxsize=200)
        with self._lock:
            self._clients.add(q)
            for msg in self._last.values():
                q.put_nowait(msg)
        return q

    def unsubscribe(self, q):
        with self._lock:
            self._clients.discard(q)

    def publish(self, event, data):
        msg = f'event: {event}\ndata: {json.dumps(data, separators=(",", ":"))}\n\n'
        with self._lock:
            self._last[event] = msg
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                try:
                    q.get_nowait()
                    q.put_nowait(msg)
                except Exception:
                    pass


class VideoHub:
    """Holds the newest camera JPEG and wakes every streaming browser when a new one arrives."""

    def __init__(self):
        self._cond = threading.Condition()
        self.frame = None
        self.seq = 0
        self.stamp = 0.0

    def update(self, jpeg):
        with self._cond:
            self.frame = jpeg
            self.seq += 1
            self.stamp = time.monotonic()
            self._cond.notify_all()

    def wait(self, last_seq, timeout):
        with self._cond:
            ready = lambda: self.frame is not None and self.seq != last_seq
            self._cond.wait_for(ready, timeout)
            return (self.seq, self.frame) if ready() else None

    def active(self):
        return self.frame is not None and (time.monotonic() - self.stamp) < 2.0


# ============================================================================ web server
def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'CarDashboard/1.0'
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            pass

        # ----- helpers
        def _query(self):
            return parse_qs(urlparse(self.path).query)

        def _authorized(self):
            if not node.pin:
                return True
            supplied = self.headers.get('X-PIN') or (self._query().get('pin') or [''])[0]
            return hmac.compare_digest(supplied.encode(), node.pin.encode())

        def _send(self, code, body, ctype, extra=None):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache')
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj).encode(), 'application/json')

        # ----- GET
        def do_GET(self):
            try:
                path = urlparse(self.path).path
                if path.startswith('/api/'):
                    self._api_get(path)
                else:
                    self._static(path)
            except ApiError as e:
                self._json(e.code, {'error': e.message})
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                self.close_connection = True
            except Exception as e:
                node.get_logger().error(f'dashboard GET {self.path}: {e}')
                try:
                    self._json(500, {'error': 'internal error'})
                except Exception:
                    pass

        def _api_get(self, path):
            if path == '/api/info':
                return self._json(200, node.api_info(self._authorized()))
            if not self._authorized():
                raise ApiError(401, 'PIN required')
            if path == '/api/maps':
                return self._json(200, node.api_maps())
            if path == '/api/events':
                return self._events()
            if path == '/api/video':
                return self._video()
            m = re.match(r'^/api/map/([0-9a-f]{10})/(meta|image\.png)$', path)
            if m:
                if m.group(2) == 'meta':
                    return self._json(200, node.maps.meta(m.group(1)))
                node.maps.get(m.group(1))
                return self._send(200, node.maps.render(m.group(1))['png'], 'image/png')
            raise ApiError(404, 'not found')

        def _static(self, path):
            rel = os.path.normpath(path.lstrip('/')) if path.strip('/') else 'index.html'
            root = os.path.realpath(node.web_dir)
            full = os.path.realpath(os.path.join(root, rel))
            if not full.startswith(root + os.sep) or not os.path.isfile(full):
                raise ApiError(404, 'not found')
            ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
            if ctype.startswith('text/') or ctype in ('application/javascript', 'application/json'):
                ctype += '; charset=utf-8'
            with open(full, 'rb') as f:
                self._send(200, f.read(), ctype)

        def _events(self):
            q = node.hub.subscribe()
            self.close_connection = True
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Connection', 'keep-alive')
                self.send_header('X-Accel-Buffering', 'no')
                self.end_headers()
                self.wfile.write(b'retry: 2000\n\n')
                self.wfile.flush()
                while True:
                    try:
                        msg = q.get(timeout=10)
                    except queue.Empty:
                        msg = ': keepalive\n\n'
                    self.wfile.write(msg.encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                pass
            finally:
                node.hub.unsubscribe(q)

        def _video(self):
            """MJPEG stream: an <img src="/api/video"> shows it live in every browser."""
            first = node.video.wait(-1, 3.0)
            if first is None:
                raise ApiError(503, 'no camera video')
            self.close_connection = True
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Connection', 'close')
                self.end_headers()
                seq, frame, idle = first[0], first[1], 0.0
                while True:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                    self.wfile.flush()
                    while True:
                        res = node.video.wait(seq, 2.0)
                        if res is not None:
                            seq, frame = res
                            idle = 0.0
                            break
                        idle += 2.0
                        if idle >= 10.0:                    # camera gone: end the stream, the page will retry
                            return
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                pass

        # ----- POST
        def do_POST(self):
            try:
                path = urlparse(self.path).path
                if not self._authorized():
                    raise ApiError(401, 'PIN required')
                length = int(self.headers.get('Content-Length') or 0)
                if length > 4096:
                    raise ApiError(413, 'request too large')
                try:
                    body = json.loads(self.rfile.read(length) or b'{}')
                    if not isinstance(body, dict):
                        raise ValueError
                except ValueError:
                    raise ApiError(400, 'invalid JSON')

                def num(key):
                    v = body.get(key)
                    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                        raise ApiError(400, f'"{key}" must be a number')
                    return float(v)

                if path == '/api/initialpose':
                    node.set_initial_pose(num('x'), num('y'), num('yaw'))
                    return self._json(200, {'ok': True})
                if path == '/api/goal':
                    node.send_goal(num('x'), num('y'), num('yaw'))
                    return self._json(200, {'ok': True})
                if path == '/api/cancel':
                    node.cancel_all()
                    return self._json(200, {'ok': True})
                if path == '/api/select_map':
                    node.select_map(str(body.get('id', '')))
                    return self._json(200, {'ok': True, 'map_id': node.current_map_id})
                raise ApiError(404, 'not found')
            except ApiError as e:
                self._json(e.code, {'error': e.message})
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                self.close_connection = True
            except Exception as e:
                node.get_logger().error(f'dashboard POST {self.path}: {e}')
                try:
                    self._json(500, {'error': 'internal error'})
                except Exception:
                    pass

    return Handler


# ============================================================================ ROS node
class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard')
        d = self.declare_parameter
        d('host', '0.0.0.0')
        d('port', 8080)
        d('pin', '')                                         # empty = no PIN. Set one when others share the Wi-Fi!
        d('web_dir', '')
        d('map_dirs', '')                                    # extra folders with map .yaml files, separated by ":"
        d('initial_map', '')                                 # the map yaml that map_server was started with
        d('map_frame', 'map')
        d('base_frame', 'base_footprint')
        d('scan_topic', '/scan')
        d('plan_topic', '/plan')
        d('odom_topic', '/odom')
        d('cmd_vel_topic', '/cmd_vel')
        d('initialpose_topic', '/initialpose')
        d('video_topic', '/indoor_lane/image/compressed')
        d('nav_action', 'navigate_to_pose')
        d('load_map_service', '/map_server/load_map')
        # car body, metres, measured from base_footprint (rear axle centre) - same values as config/real/car.yaml
        d('wheelbase', 0.76)
        d('front_overhang', 0.98)
        d('rear_overhang', 0.22)
        d('width', 0.64)
        p = lambda n: self.get_parameter(n).value

        self.pin = str(p('pin'))
        self.map_frame, self.base_frame = str(p('map_frame')), str(p('base_frame'))
        self.car = {'wheelbase': float(p('wheelbase')), 'front': float(p('front_overhang')),
                    'rear': float(p('rear_overhang')), 'width': float(p('width'))}

        try:
            share = get_package_share_directory('self_driving_car')
        except Exception:
            share = os.path.dirname(os.path.realpath(__file__))
        self.web_dir = str(p('web_dir')) or os.path.join(share, 'web')
        dirs = [os.path.expanduser('~/car_maps'), os.path.join(share, 'maps')]
        dirs += [x for x in str(p('map_dirs')).split(':') if x]
        initial = os.path.abspath(os.path.expanduser(str(p('initial_map')))) if str(p('initial_map')) else ''
        if initial:
            dirs.append(os.path.dirname(initial))
        self.maps = MapRegistry(dirs, extra=[initial] if initial else [])
        self.current_map_id = MapRegistry.make_id(initial) if initial else None

        self.hub, self.video = EventHub(), VideoHub()
        self._lock = threading.Lock()
        self._pose = None
        self._speed = 0.0
        self._status_active = False
        self._goal_seq = 0
        self._goal = None
        self._nav_state = 'idle'
        self._nav_dist = None
        self._nav_recov = 0
        self._last_scan = 0.0

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, str(p('scan_topic')), self._scan_cb, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(Path, str(p('plan_topic')), self._plan_cb, 10, callback_group=cb)
        self.create_subscription(Odometry, str(p('odom_topic')), self._odom_cb, 10, callback_group=cb)
        self.create_subscription(GoalStatusArray, f"/{str(p('nav_action')).strip('/')}/_action/status",
                                 self._status_cb, 10, callback_group=cb)
        self.create_subscription(CompressedImage, str(p('video_topic')), self._video_cb, qos_profile_sensor_data,
                                 callback_group=cb)
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, str(p('initialpose_topic')), 10)
        self.cmd_pub = self.create_publisher(Twist, str(p('cmd_vel_topic')), 10)
        self.nav_client = ActionClient(self, NavigateToPose, str(p('nav_action')), callback_group=cb)
        self.cancel_client = self.create_client(
            CancelGoal, f"/{str(p('nav_action')).strip('/')}/_action/cancel_goal", callback_group=cb)
        self.load_map_client = self.create_client(LoadMap, str(p('load_map_service')), callback_group=cb)
        self.create_timer(0.1, self._tick, callback_group=cb)

        self.server = ThreadingHTTPServer((str(p('host')), int(p('port'))), make_handler(self))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f"dashboard ready:  http://<car-ip>:{int(p('port'))}"
                               f"{'   (PIN required)' if self.pin else '   (no PIN set)'}")

    # ------------------------------------------------------------------ ROS inputs
    def _tick(self):
        pose = None
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            pose = (t.transform.translation.x, t.transform.translation.y, yaw_from_quat(t.transform.rotation))
        except Exception:
            pass
        with self._lock:
            self._pose = pose
        self.hub.publish('state', self._state())

    def _state(self):
        with self._lock:
            pose = self._pose
            return {
                'localized': pose is not None,
                'pose': None if pose is None else {'x': round(pose[0], 3), 'y': round(pose[1], 3), 'yaw': round(pose[2], 3)},
                'speed': round(self._speed, 3),
                'nav': {'state': self._nav_state if self._nav_state != 'idle' or not self._status_active else 'navigating',
                        'active': self._active_locked(), 'distance': self._nav_dist, 'goal': self._goal,
                        'recoveries': self._nav_recov},
                'map_id': self.current_map_id,
                'video': self.video.active(),
            }

    def _active_locked(self):
        return self._status_active or self._nav_state in ('sending', 'navigating')

    def _scan_cb(self, msg):
        now = time.monotonic()
        if now - self._last_scan < 0.2:                      # 5 Hz is plenty for the screen
            return
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        self._last_scan = now
        tx, ty = t.transform.translation.x, t.transform.translation.y
        yaw = yaw_from_quat(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        n = len(msg.ranges)
        stride = max(1, -(-n // 360))                        # at most ~360 points per scan
        pts = []
        for i in range(0, n, stride):
            r = msg.ranges[i]
            if not (msg.range_min <= r <= msg.range_max):    # also drops nan / inf
                continue
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            pts += [round(tx + c * lx - s * ly, 2), round(ty + s * lx + c * ly, 2)]
        self.hub.publish('scan', {'pts': pts})

    def _plan_cb(self, msg):
        poses = msg.poses
        stride = max(1, -(-len(poses) // 200))               # at most ~200 points per path
        pts = []
        for ps in poses[::stride]:
            pts += [round(ps.pose.position.x, 2), round(ps.pose.position.y, 2)]
        if poses and (len(poses) - 1) % stride:
            pts += [round(poses[-1].pose.position.x, 2), round(poses[-1].pose.position.y, 2)]
        self.hub.publish('plan', {'pts': pts})

    def _odom_cb(self, msg):
        self._speed = msg.twist.twist.linear.x

    def _status_cb(self, msg):
        active = any(s.status in (1, 2, 3) for s in msg.status_list)   # accepted / executing / cancelling
        with self._lock:
            self._status_active = active

    def _video_cb(self, msg):
        self.video.update(bytes(msg.data))

    # ------------------------------------------------------------------ commands from the page
    def api_info(self, authorized):
        return {'pin_required': bool(self.pin), 'authorized': authorized, 'car': self.car}

    def api_maps(self):
        self.maps.scan()
        return {'current': self.current_map_id,
                'maps': [{'id': e['id'], 'name': e['name'], 'path': e['path']} for e in self.maps.entries.values()]}

    @staticmethod
    def _pose_msg(x, y, yaw, stamp):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return m

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25                        # same numbers RViz uses for "2D Pose Estimate"
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        self.initpose_pub.publish(msg)
        self.get_logger().info(f'initial pose set from dashboard: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg')

    def send_goal(self, x, y, yaw):
        if not self.nav_client.server_is_ready():
            raise ApiError(503, 'Nav2 is not running (action navigate_to_pose not available)')
        with self._lock:
            localized = self._pose is not None
        if not localized:
            raise ApiError(409, 'Car is not localized yet - set the initial pose first')
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_msg(x, y, yaw, self.get_clock().now().to_msg())
        goal.pose.header.frame_id = self.map_frame
        with self._lock:
            self._goal_seq += 1
            seq = self._goal_seq
            self._goal = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
            self._nav_state, self._nav_dist, self._nav_recov = 'sending', None, 0
        fut = self.nav_client.send_goal_async(goal, feedback_callback=lambda fb, s=seq: self._feedback_cb(s, fb))
        fut.add_done_callback(lambda f, s=seq: self._goal_response_cb(s, f))
        self.get_logger().info(f'goal from dashboard: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg')

    def _feedback_cb(self, seq, fb):
        with self._lock:
            if seq == self._goal_seq:
                self._nav_dist = round(float(fb.feedback.distance_remaining), 2)
                self._nav_recov = int(fb.feedback.number_of_recoveries)

    def _goal_response_cb(self, seq, fut):
        try:
            handle = fut.result()
        except Exception as e:
            self._finish(seq, 'failed', f'goal request failed: {e}')
            return
        if not handle.accepted:
            self._finish(seq, 'rejected', 'goal rejected by Nav2')
            return
        with self._lock:
            if seq == self._goal_seq:
                self._nav_state = 'navigating'
        handle.get_result_async().add_done_callback(lambda f, s=seq: self._result_cb(s, f))

    def _result_cb(self, seq, fut):
        try:
            status = fut.result().status
        except Exception:
            status = 6
        self._finish(seq, {4: 'succeeded', 5: 'cancelled'}.get(status, 'failed'))

    def _finish(self, seq, state, log=None):
        with self._lock:
            if seq != self._goal_seq:                        # a newer goal replaced this one
                return
            self._nav_state, self._goal, self._nav_dist, self._nav_recov = state, None, None, 0
        if log:
            self.get_logger().warn(log)
        self.hub.publish('plan', {'pts': []})

    def cancel_all(self):
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())   # empty request = cancel every active goal
        for _ in range(3):
            self.cmd_pub.publish(Twist())                    # zero velocity
        self.get_logger().warn('STOP pressed on the dashboard - navigation cancelled')

    def select_map(self, map_id):
        entry = self.maps.get(map_id)
        if entry is None:
            raise ApiError(404, 'unknown map')
        with self._lock:
            busy = self._active_locked()
        if busy:
            raise ApiError(409, 'Car is navigating - press STOP and wait before changing the map')
        deadline = time.monotonic() + 2.0
        while not self.load_map_client.service_is_ready() and time.monotonic() < deadline:
            time.sleep(0.1)
        if not self.load_map_client.service_is_ready():
            raise ApiError(503, 'map_server is not running (service load_map not available)')
        req = LoadMap.Request()
        req.map_url = entry['path']
        done, box = threading.Event(), {}

        def cb(f):
            try:
                box['r'] = f.result()
            except Exception as e:
                box['e'] = str(e)
            done.set()

        self.load_map_client.call_async(req).add_done_callback(cb)
        if not done.wait(15.0) or 'r' not in box:
            raise ApiError(504, 'map_server did not answer: ' + box.get('e', 'timeout'))
        if box['r'].result != LoadMap.Response.RESULT_SUCCESS:
            reasons = {1: 'map file does not exist', 2: 'invalid map data', 3: 'invalid map metadata'}
            raise ApiError(500, 'map_server refused the map: ' + reasons.get(box['r'].result, 'unknown error'))
        with self._lock:
            self.current_map_id = entry['id']
            self._goal = None
        self.hub.publish('plan', {'pts': []})
        self.get_logger().info(f"map loaded from dashboard: {entry['path']}")

    def shutdown(self):
        try:
            self.server.shutdown()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
