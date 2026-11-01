"""
GUI/미션이 같이 쓰는 ROS 노드: TF(로봇 위치), /scan, /map, Nav2 경로, cmd_vel, Nav2 이동, 미션 스레드
"""
from collections import deque
import math
import threading
import time
import traceback

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

from pros2_robot.map_utils import (MapData, scan_points, set_quat_yaw, transform_pts, wrap,
                                   yaw_from_quat)

LASER_FRAME = 'laser_frame'
LASER_FALLBACK = (0.0625, 0.0, 0.0)   # URDF laser_joint (TF를 못 받을 때만 사용)
TRAIL_LEN = 2000                      # 지나온 길 점 개수
TRAIL_STEP = 0.02                     # m, 이만큼 움직일 때마다 기록

# 제자리 회전 (rotate)
ODOM_STALE = 1.0                      # s, odom TF가 이보다 오래되면 로봇 데이터가 끊긴 것으로 봄
ODOM_WAIT = 20.0                      # s, odom이 끊긴 채 이만큼 지나면 실패
ROTATE_STALL = 10.0                   # s, odom은 들어오는데 이 시간 동안 ROTATE_PROGRESS도 못 돌면 실패
ROTATE_PROGRESS = math.radians(5)

STATUS_TEXT = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}


class Cancelled(Exception):
    pass


class MissionError(Exception):
    pass


def make_pose_stamped(x, y, yaw, frame='map', stamp=None):
    p = PoseStamped()
    p.header.frame_id = frame
    if stamp is not None:
        p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    set_quat_yaw(p.pose.orientation, yaw)
    return p


def tf_to_pose(t):
    return (t.transform.translation.x, t.transform.translation.y,
            yaw_from_quat(t.transform.rotation))


class RobotInterface(Node):
    def __init__(self, name='pros2_mission'):
        super().__init__(name)
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.mission_thread = None
        self.mission_name = None
        self.goal_handle = None
        self.status = ''
        self.map = None
        self.map_version = 0
        self.scan = None
        self.scan_seq = 0
        self.scans = deque(maxlen=10)       # 최근 스캔 (seq, msg)
        self.plan = None                    # Nav2 전역 경로 (N, 2) map 좌표
        self.local_plan = None
        self.trail = deque(maxlen=TRAIL_LEN)
        self.nav_goal = None                # 마지막으로 보낸 Nav2 목표 pose
        self.viz = {}                       # 미션이 화면에 보여 줄 것 (parking.py 참고)
        self.amcl = None                    # (time, (x, y, yaw), std_xy, std_yaw)
        self.toy = None                     # (time, [found, cx, cy, w, h, conf])
        self.toy_seq = 0
        self.mission_step = None            # 실행 중인 미션 단계 번호 (mission_runner)
        self._laser_offset = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._map_cb, map_qos)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', lambda m: self._path_cb(m, 'plan'), 10)
        self.create_subscription(Path, 'local_plan', lambda m: self._path_cb(m, 'local_plan'), 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_cb, map_qos)
        self.create_subscription(Float32MultiArray, 'toy_detection', self._toy_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.follow_pub = self.create_publisher(Bool, 'toy_tracker/enable', 10)
        self.global_loc_client = self.create_client(Empty, 'reinitialize_global_localization')
        self.nomotion_client = self.create_client(Empty, 'request_nomotion_update')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(0.1, self._trail_tick)

    # ---------- 구독 ----------
    def set_map(self, m):
        with self.lock:
            self.map = m
            self.map_version += 1

    def _map_cb(self, msg):
        self.set_map(MapData.from_msg(msg))

    def _scan_cb(self, msg):
        with self.lock:
            self.scan = msg
            self.scan_seq += 1
            self.scans.append((self.scan_seq, msg))

    def _path_cb(self, msg, attr):
        pts = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        frame = msg.header.frame_id or 'map'
        if len(pts) and frame != 'map':
            tf = self.lookup_pose('map', frame)
            pts = transform_pts(tf, pts) if tf is not None else None
        with self.lock:
            setattr(self, attr, pts if pts is not None and len(pts) else None)

    def _amcl_cb(self, msg):
        p = msg.pose.pose
        c = msg.pose.covariance
        std_xy = math.sqrt(max(c[0], c[7], 0.0))
        std_yaw = math.sqrt(max(c[35], 0.0))
        with self.lock:
            self.amcl = (time.time(), (p.position.x, p.position.y, yaw_from_quat(p.orientation)),
                         std_xy, std_yaw)

    def _toy_cb(self, msg):
        if len(msg.data) < 6:
            return
        with self.lock:
            self.toy = (time.time(), list(msg.data[:6]))
            self.toy_seq += 1

    def _trail_tick(self):
        p = self.robot_pose()
        if p is None:
            return
        with self.lock:
            if not self.trail or math.hypot(p[0] - self.trail[-1][0],
                                            p[1] - self.trail[-1][1]) >= TRAIL_STEP:
                self.trail.append((p[0], p[1]))

    def clear_trail(self):
        with self.lock:
            self.trail.clear()

    # ---------- 위치 / 스캔 ----------
    def lookup_pose(self, target, source, stamp=None):
        try:
            t = self.tf_buffer.lookup_transform(target, source,
                                                stamp if stamp is not None else Time())
        except Exception:
            return None
        return tf_to_pose(t)

    def robot_pose(self):
        """map 기준 로봇 (x, y, yaw), AMCL(map->odom)이 없으면 None"""
        return self.lookup_pose('map', 'base_link')

    def laser_offset(self):
        if self._laser_offset is None:
            p = self.lookup_pose('base_link', LASER_FRAME)
            if p is None:
                return LASER_FALLBACK
            self._laser_offset = p
        return self._laser_offset

    def base_points(self, scan, max_range=None):
        """스캔 -> base_link 좌표 점"""
        return transform_pts(self.laser_offset(), scan_points(scan, max_range))

    def scan_in_frame(self, scan, frame='map', max_range=None):
        """스캔 -> frame 좌표 점 (스캔 시각의 TF, 없으면 최신 TF), TF가 없으면 None"""
        pose = self.lookup_pose(frame, scan.header.frame_id or LASER_FRAME,
                                Time.from_msg(scan.header.stamp))
        if pose is None:
            pose = self.lookup_pose(frame, scan.header.frame_id or LASER_FRAME)
        if pose is None:
            return None
        return transform_pts(pose, scan_points(scan, max_range))

    def latest_scan(self):
        with self.lock:
            return self.scan_seq, self.scan

    def wait_scans(self, n, timeout=5.0):
        """지금 이후로 새로 들어온 스캔 n장"""
        with self.lock:
            start = self.scan_seq
        self.wait_until(lambda: self.scan_seq >= start + n, timeout * n, '/scan')
        with self.lock:
            return [m for s, m in self.scans if s > start][-n:]

    # ---------- 명령 ----------
    def drive(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def odom_pose(self):
        """-> (odom->base_link 최신 자세, 그 TF의 나이 [s]), 없으면 (None, inf)"""
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
        except Exception:
            return None, float('inf')
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        return tf_to_pose(t), age

    def rotate(self, angle, w=0.5, until=None):
        """제자리 회전 angle [rad] (부호 = 방향, odom TF로 회전량 측정, 미션 스레드에서)
        until()이 True가 되면 바로 멈추고 True, 다 돌면 False
        odom이 끊기면(ODOM_STALE) 멈춰서 다시 들어올 때까지 최대 ODOM_WAIT 기다렸다가 이어서 돎
        odom은 들어오는데 ROTATE_STALL 동안 거의 못 돌면 실패"""
        start, _ = self.odom_pose()
        if start is None:
            raise MissionError('no odom->base_link TF')
        prev, turned = start[2], 0.0
        w = math.copysign(abs(w), angle)
        progress_t, progress_at = time.time(), 0.0
        stale_since = None
        try:
            while abs(turned) < abs(angle):
                self.check()
                if until is not None and until():
                    return True
                now = time.time()
                p, age = self.odom_pose()
                if p is None or age > ODOM_STALE:
                    if stale_since is None:
                        stale_since = now
                        self.get_logger().warn('odom stalled during rotation, waiting ...')
                    if now - stale_since > ODOM_WAIT:
                        raise MissionError(f'no odom for {ODOM_WAIT:.0f} s (robot data lost?)')
                    self.stop_robot()
                    time.sleep(0.05)
                    continue
                if stale_since is not None:
                    self.get_logger().info(
                        f'odom resumed after {now - stale_since:.1f} s, continue rotation')
                    stale_since = None
                    progress_t = now        # 끊겨 있던 시간은 정지 판정에서 뺌
                turned += wrap(p[2] - prev)
                prev = p[2]
                if abs(turned) - progress_at >= ROTATE_PROGRESS:
                    progress_t, progress_at = now, abs(turned)
                elif now - progress_t > ROTATE_STALL:
                    raise MissionError(f'rotation stalled: turned less than '
                                       f'{math.degrees(ROTATE_PROGRESS):.0f} deg in '
                                       f'{ROTATE_STALL:.0f} s (robot not turning?)')
                self.drive(0.0, w)
                time.sleep(0.05)
        finally:
            self.stop_robot()
        return False

    def publish_initial_pose(self, pose, sxy=0.1, syaw=0.1):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = make_pose_stamped(*pose).pose
        msg.pose.covariance[0] = sxy ** 2
        msg.pose.covariance[7] = sxy ** 2
        msg.pose.covariance[35] = syaw ** 2
        self.init_pub.publish(msg)
        self.set_status(f'Initial pose: x={pose[0]:.2f} y={pose[1]:.2f} '
                        f'yaw={math.degrees(pose[2]):.0f}')

    def set_follow(self, on):
        """toy_tracker 켜기/끄기"""
        self.follow_pub.publish(Bool(data=bool(on)))

    def call_service(self, client, what, timeout=5.0):
        """std_srvs/Empty 서비스 호출 (미션 스레드에서)"""
        if not client.wait_for_service(timeout_sec=timeout):
            raise MissionError(f'{what} service not available (is AMCL running?)')
        fut = client.call_async(Empty.Request())
        self.wait_until(fut.done, timeout, what)
        return fut.result()

    # ---------- 미션 실행 ----------
    def set_status(self, text):
        with self.lock:
            self.status = text
        self.get_logger().info(text)

    def start_mission(self, name, fn, *args):
        """fn(*args)를 별도 스레드에서 실행. 이미 실행 중이면 False"""
        with self.lock:
            if self.mission_thread is not None and self.mission_thread.is_alive():
                self.status = f'Busy ({self.mission_name}). Press Stop first.'
                return False
            self.cancel_event.clear()
            self.mission_name = name
            self.mission_thread = threading.Thread(target=self._run_mission,
                                                   args=(name, fn, args), daemon=True)
            self.mission_thread.start()
        return True

    def _run_mission(self, name, fn, args):
        try:
            fn(*args)
        except Cancelled:
            self.set_status(f'{name}: stopped')
        except MissionError as e:
            self.set_status(f'{name} failed: {e}')
        except Exception as e:
            self.get_logger().error(traceback.format_exc())
            self.set_status(f'{name} error: {e}')
        finally:
            self.stop_robot()
            with self.lock:
                self.mission_name = None
                self.mission_step = None

    def busy(self):
        with self.lock:
            return self.mission_thread is not None and self.mission_thread.is_alive()

    def stop(self):
        self.cancel_event.set()
        with self.lock:
            handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        self.set_follow(False)
        self.stop_robot()

    def check(self):
        if self.cancel_event.is_set() or not rclpy.ok():
            raise Cancelled()

    def sleep(self, sec):
        end = time.time() + sec
        while time.time() < end:
            self.check()
            time.sleep(0.05)

    def wait_until(self, cond, timeout, what):
        end = time.time() + timeout
        while not cond():
            self.check()
            if time.time() > end:
                raise MissionError(f'timeout waiting for {what}')
            time.sleep(0.1)

    # ---------- Nav2 ----------
    def navigate(self, pose, timeout=180.0):
        """Nav2로 pose까지 이동 (미션 스레드에서 호출). 성공하면 True, aborted 등이면 False"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            raise MissionError('Nav2 navigate_to_pose server not running')
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(*pose, stamp=self.get_clock().now().to_msg())
        with self.lock:
            self.nav_goal = pose
        send = self.nav_client.send_goal_async(goal)
        self.wait_until(send.done, 10.0, 'Nav2 to accept goal')
        handle = send.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            return False
        with self.lock:
            self.goal_handle = handle
        result = handle.get_result_async()
        end = time.time() + timeout
        try:
            while not result.done():
                if self.cancel_event.is_set() or time.time() > end:
                    handle.cancel_goal_async()
                    self.check()
                    raise MissionError('Nav2 timeout')
                time.sleep(0.1)
        finally:
            with self.lock:
                self.goal_handle = None
                self.plan = self.local_plan = None     # 끝난 목표의 경로는 지움
        status = result.result().status
        self.get_logger().info(f'Nav2 goal {STATUS_TEXT.get(status, status)}')
        if status == GoalStatus.STATUS_SUCCEEDED:
            with self.lock:
                self.nav_goal = None
        return status == GoalStatus.STATUS_SUCCEEDED

    def go_to(self, pose, label='Nav goal'):
        """미션용: 실패하면 MissionError"""
        self.set_status(f'{label}: x={pose[0]:.2f} y={pose[1]:.2f} '
                        f'yaw={math.degrees(pose[2]):.0f} (Nav2) ...')
        if not self.navigate(pose):
            raise MissionError('Nav2 could not reach the goal')
        self.set_status(f'{label}: reached')
