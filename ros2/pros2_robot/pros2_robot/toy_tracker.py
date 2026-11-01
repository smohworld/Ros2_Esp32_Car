#!/usr/bin/env python3
"""
장난감 따라가기 노드: /toy_detection(toy_detector)을 보고 장난감과 DIST(30cm) 거리를 유지하며 따라감 (카메라만 사용)

  구독 : /toy_detection [found, cx, cy, w, h, conf], /toy_tracker/enable (std_msgs/Bool)
  발행 : /cmd_vel (켜져 있을 때만, 꺼지면 정지 한 번 발행하고 손 뗌 -> Nav2와 안 싸움)

카메라가 로봇 정면에서 살짝 틀어져 있어서 화면 중앙(cx=0)이 로봇 정면이 아님
  -> 보정(--calibrate)으로 "로봇 정면 --dist 거리에 놓인 장난감"의 cx, 박스 높이 h를 재서 파일에 저장
     cx_offset : 정면일 때의 cx (이 값을 0으로 보고 조향)
     ref_h     : ref_dist 거리일 때 박스 높이
     ref_dist  : 보정할 때 장난감까지 거리 (--dist)
  방향 = cx - cx_offset, 거리 = ref_dist * ref_h / h

놓치면(TOY_STALE) 바로 놓친 쪽으로 찾을 때까지 계속 돎
  놓친 쪽: 화면 가장자리면 빠져나간 쪽 > 화면에서 움직이던 쪽 > 정면 기준 장난감이 있던 쪽
  회전 속도: --search-w에서 시작해 --search-acc로 --search-w-max까지 증가
따라갈 때 회전 최대 속도는 --turn-max
확신도: 추적 중에는 --keep-conf 이상이면 검출로 인정
        놓친 뒤 다시 잡을 때는 --conf 이상 한 번, 또는 --keep-conf 이상 ACQUIRE_N번 연속

  보정 (장난감을 로봇 정면 30cm에 놓고, toy_detector 켠 상태):
    ros2 run pros2_robot toy_tracker --calibrate                (--dist 0.25 처럼 놓은 거리 지정 가능)
  실행:
    ros2 run pros2_robot toy_tracker                    (바로 따라가기)
    ros2 run pros2_robot toy_tracker --start-disabled   (미션이 /toy_tracker/enable로 켜고 끔)
    ros2 run pros2_robot toy_tracker --cx-offset 0.12   (보정 파일 대신 직접 지정)
"""
import argparse
import math
import os
import time

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
import yaml

CALIB_FILE = os.path.expanduser('~/.ros/toy_tracker_calib.yaml')

DIST = 0.30                 
DIST_TOL = 0.02             
HFOV = math.radians(41)

KV = 1.2                    # 1/s, 거리 오차 -> 선속도
KW = 1.5                    # 1/s, 방향 오차 [rad] -> 각속도 (카메라 지연이 커서 크면 지나치게 돎)
V_MAX = 0.15                # m/s (nav2 max_vel_x와 같게)
V_BACK = 0.06               # m/s, 너무 가까울 때 후진 최대
W_MAX = 0.6                 # rad/s, 따라갈 때 최대 (--turn-max)
W_MIN = 0.3                 # rad/s, 제자리 회전 최소 (스키드 스티어는 이보다 작으면 안 돎)
CX_TOL = 0.06               # 이 안이면 정면으로 봄 (놓친 쪽 판단용)
EDGE_CX = 0.6               # |cx|가 이보다 크면 화면 가장자리 -> 놓치면 빠져나간 쪽으로 찾기
HEAD_TOL = math.radians(4)  # 방향 오차가 이 안이면 회전 안 함 (지연 때문에 흔들리지 않게)
HEAD_SLOW = math.radians(25)  # 방향 오차가 이만큼이면 전진 0 (먼저 돌기)
ACC_V = 0.4                 # m/s^2, 선속도 변화 제한
EMA = 0.5                   # 검출값 저역통과 (새 값 비중)

TOY_STALE = 0.6             # s, 이보다 오래된 검출은 무시 -> 놓친 것
# /toy_detection(검출 없음 포함)이 이만큼 안 오면 카메라가 끊긴 것
# -> 안 보이는 채로 돌지 않고 정지 (Wi-Fi가 약하면 몇 초씩 끊김)
FEED_STALE = 0.5            # s
LOST_SEARCH = 1.0           # s, 놓친 지 이 안이면 아직 추적 중으로 봄 (--keep-conf로 다시 잡음)
SEARCH_W = 0.4              # rad/s, 찾기 시작 회전 속도 (--search-w)
SEARCH_W_MAX = 0.8          # rad/s, 찾기 최대 회전 속도 (--search-w-max)
SEARCH_ACC = 0.1            # rad/s^2, 찾는 동안 회전 속도 증가율 (--search-acc)
TREND_MIN = 0.3             # 1/s, 화면에서 cx가 이보다 빠르게 움직이면 그 쪽으로 빠져나간 것으로 봄
RATE = 20.0                 # Hz

ACQUIRE_N = 3               # 놓친 뒤 --keep-conf 검출이 이만큼 연속이면 다시 따라가기
CALIB_SAMPLES = 30


def load_calib(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except OSError:
        return {}


class ToyTracker(Node):
    def __init__(self, args):
        super().__init__('toy_tracker')
        self.args = args
        calib = load_calib(args.calib)
        self.cx_offset = args.cx_offset if args.cx_offset is not None else calib.get('cx_offset', 0.0)
        self.ref_h = calib.get('ref_h')
        self.ref_dist = calib.get('ref_dist', args.dist)
        self.enabled = not args.start_disabled and not args.calibrate
        self.feed_t = 0.0               # 마지막 /toy_detection 받은 시각 (검출 없음 포함)
        self.det = None                 # (time, cx, h) 저역통과된 값
        self.last_seen = 0.0
        self.last_side = 1.0            # 놓치면 돌 방향 (+1 왼쪽 회전, -1 오른쪽)
        self.searching = False
        self.search_t = 0.0             # 찾기 시작 시각
        self.raw = None                 # (time, cx) 마지막 원본 검출 (화면에서 움직이는 방향 계산용)
        self.cx_vel = 0.0               # 화면에서 cx 변화 속도 [1/s], 저역통과
        self.v = 0.0
        self.acquire = 0                # 놓친 상태에서 --keep-conf 이상 연속 검출 수
        self.samples = []

        self.create_subscription(Float32MultiArray, 'toy_detection', self.det_cb, 10)
        self.create_subscription(Bool, 'toy_tracker/enable', self.enable_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(1.0 / RATE, self.calib_tick if args.calibrate else self.tick)

        if args.calibrate:
            self.get_logger().info(f'Calibrating: put the toy straight ahead, {args.dist * 100:.0f} cm '
                                   f'from the robot front. Collecting {CALIB_SAMPLES} detections ...')
        else:
            self.get_logger().info(
                f'cx_offset={self.cx_offset:+.3f}, '
                + (f'ref_h={self.ref_h:.3f} @ {self.ref_dist:.3f} m' if self.ref_h
                   else 'no ref_h (run --calibrate), NO DISTANCE: turn only')
                + f', {"enabled" if self.enabled else "disabled"}')

    # ---------- 구독 ----------
    def enable_cb(self, msg):
        if msg.data == self.enabled:
            return
        self.enabled = msg.data
        self.get_logger().info('follow ' + ('enabled' if msg.data else 'disabled'))
        self.v = 0.0
        self.last_seen = time.time() if msg.data else 0.0   # 켜자마자 찾기 회전 방지
        if not msg.data:
            self.stop()

    def det_cb(self, msg):
        d = msg.data
        now = time.time()
        self.feed_t = now
        tracking = now - self.last_seen < LOST_SEARCH
        min_conf = self.args.conf if self.args.calibrate else self.args.keep_conf
        if len(d) < 6 or d[0] < 0.5 or d[5] < min_conf:
            self.acquire = 0
            return
        if not tracking and not self.args.calibrate:
            self.acquire += 1
            if d[5] < self.args.conf and self.acquire < ACQUIRE_N:
                return
            self.get_logger().info(f'toy acquired (conf {d[5]:.2f})')
        self.acquire = 0
        cx, h = float(d[1]), float(d[4])
        if self.args.calibrate:
            self.samples.append((cx, h))
            return
        if self.raw is not None and 0.0 < now - self.raw[0] < TOY_STALE:
            vel = (cx - self.raw[1]) / (now - self.raw[0])
            self.cx_vel = EMA * vel + (1 - EMA) * self.cx_vel
        else:
            self.cx_vel = 0.0
        self.raw = (now, cx)
        # 놓치면 돌 방향 (화면 왼쪽(cx -) -> 왼쪽 회전(+))
        if abs(cx) > EDGE_CX:                       # 화면 가장자리 -> 빠져나간 쪽
            self.last_side = -math.copysign(1.0, cx)
        elif abs(self.cx_vel) > TREND_MIN:          # 화면에서 움직이던 쪽
            self.last_side = -math.copysign(1.0, self.cx_vel)
        elif abs(cx - self.cx_offset) > CX_TOL:     # 로봇 정면 기준 장난감 쪽
            self.last_side = -math.copysign(1.0, cx - self.cx_offset)
        if self.det is not None and now - self.det[0] < TOY_STALE:
            cx = EMA * cx + (1 - EMA) * self.det[1]
            h = EMA * h + (1 - EMA) * self.det[2]
        self.det = (now, cx, h)
        self.last_seen = now

    # ---------- 제어 ----------
    def stop(self):
        self.v = 0.0
        self.cmd_pub.publish(Twist())

    def drive(self, v, w):
        dv = ACC_V / RATE
        self.v = min(max(v, self.v - dv), self.v + dv)
        t = Twist()
        t.linear.x = float(self.v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def tick(self):
        if not self.enabled:
            return
        now = time.time()
        if self.det is None or now - self.det[0] > TOY_STALE:
            if now - self.feed_t > FEED_STALE:
                self.stop()                     # 카메라가 끊김 -> 안 보이는 채로 돌지 않음
                return
            if not self.searching:
                self.searching = True
                self.search_t = now
                self.get_logger().info('toy lost, searching '
                                       + ('left' if self.last_side > 0 else 'right'))
            w = min(self.args.search_w_max,
                    self.args.search_w + self.args.search_acc * (now - self.search_t))
            self.drive(0.0, self.last_side * w)
            return
        self.searching = False

        _, cx, h = self.det
        bearing = -math.atan((cx - self.cx_offset) * math.tan(HFOV / 2))   # + 왼쪽
        d = self.ref_dist * self.ref_h / h if self.ref_h and h > 1e-3 else None
        w_max = self.args.turn_max
        w = 0.0 if abs(bearing) < HEAD_TOL else max(-w_max, min(w_max, KW * bearing))

        v = 0.0
        if d is not None and abs(d - self.args.dist) > DIST_TOL:
            v = max(-V_BACK, min(V_MAX, KV * (d - self.args.dist)))
            v *= max(0.0, 1.0 - abs(bearing) / HEAD_SLOW)     # 많이 틀어져 있으면 먼저 돌기
        if abs(v) < 0.02 and w != 0.0:
            w = math.copysign(max(abs(w), W_MIN), w)  # 제자리 회전은 최소 각속도 이상
        self.drive(v, w)
        self.get_logger().debug(f'cx={cx:+.2f} d={d} bearing={math.degrees(bearing):+.1f}deg '
                                f'v={self.v:.3f} w={w:.2f}')

    def calib_tick(self):
        if len(self.samples) < CALIB_SAMPLES:
            return
        cx = float(np.median([x[0] for x in self.samples]))
        h = float(np.median([x[1] for x in self.samples]))
        data = {'cx_offset': round(cx, 4), 'ref_h': round(h, 4), 'ref_dist': round(self.args.dist, 4)}
        os.makedirs(os.path.dirname(self.args.calib), exist_ok=True)
        with open(self.args.calib, 'w') as f:
            yaml.safe_dump(data, f)
        self.get_logger().info(f'Saved {self.args.calib}: {data}')
        raise SystemExit


def main():
    ap = argparse.ArgumentParser(description='Follow the toy seen by toy_detector (camera only)')
    ap.add_argument('--conf', type=float, default=0.7, help='minimum confidence')
    ap.add_argument('--search-w', type=float, default=SEARCH_W,
                    help='rotation speed while searching for a lost toy [rad/s]')
    ap.add_argument('--search-w-max', type=float, default=SEARCH_W_MAX,
                    help='max rotation speed while searching [rad/s]')
    ap.add_argument('--search-acc', type=float, default=SEARCH_ACC,
                    help='search rotation speed increase [rad/s^2]')
    ap.add_argument('--turn-max', type=float, default=W_MAX,
                    help='max rotation speed while following [rad/s]')
    ap.add_argument('--keep-conf', type=float, default=0.5,
                    help='minimum confidence while already following')
    ap.add_argument('--dist', type=float, default=DIST,
                    help='distance to keep [m] (with --calibrate: distance of the toy)')
    ap.add_argument('--start-disabled', action='store_true',
                    help='wait for /toy_tracker/enable true')
    ap.add_argument('--calibrate', action='store_true',
                    help='measure cx_offset/ref_h with the toy straight ahead at --dist')
    ap.add_argument('--calib', default=CALIB_FILE, help='calibration file')
    ap.add_argument('--cx-offset', type=float, default=None,
                    help='cx when the toy is straight ahead (overrides calibration file)')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = ToyTracker(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        if rclpy.ok() and not args.calibrate:
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
