#!/usr/bin/env python3
"""
주차 로직: 현재 위치에서 뒤쪽(facing=back) 직선 벽을 라이다로 찾아서 벽 중심점에 벽을 등지고 주차
"""
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from pros2_robot.map_utils import (compose, default_mission_file, DEFAULT_GAP, find_spot,
                                   inverse, load_mission, MapData, rotate_vec, save_mission,
                                   scan_points, transform_pts, wrap)
from pros2_robot.robot_interface import (Cancelled, LASER_FRAME, MissionError,
                                         RobotInterface)

# 로봇 (nav2_params.yaml footprint 기준)
HALF_LEN = 0.12
HALF_WIDTH = 0.09

# 벽 검출
WALL_SCANS = 5
WALL_SEARCH_RANGE = 1.5     # m, 로봇에서 이 거리 안의 점만
LINE_TOL = 0.03             # m, RANSAC inlier 거리
RANSAC_ITERS = 200
MAX_LINES = 8
SEG_MIN_POINTS = 10
SEG_MIN_LEN = 0.25          # m
SEG_MAX_GAP = 0.15          # m, 이보다 끊기면 다른 벽 구간
WALL_PERP_TOL = math.radians(35)    # 벽 방향이 로봇 방향과 수직에서 이 이상 벗어나면 제외
RAY_MARGIN = 0.15           # m, 로봇 방향 직선이 벽 구간 끝에서 이만큼 벗어나도 그 벽으로 인정
LINE_BAND = 0.30            # m, 그린 선에서 이 거리 안의 점만 (직선 spot)
LINE_END_MARGIN = 0.10      # m, 그린 선 양 끝에서 이만큼 더
LINE_ANGLE_TOL = math.radians(25)   # 그린 선과 이 각도 안의 벽만

# 주차
PRE_PARK_EXTRA = 0.35       # m, pre-park 자세를 standoff보다 더 떨어뜨리는 거리
PRE_PARK_MAX_V = 0.10       # m/s, pre-park 자세까지 cmd_vel 이동 속도
PRE_PARK_POS_TOL = 0.03     # m
PRE_PARK_YAW_TOL = math.radians(3.0)
PRE_PARK_TIMEOUT = 40.0
MAX_V = 0.06
MIN_V = 0.015
MAX_W = 0.5
MIN_W = 0.08
K_V = 0.8
K_W = 1.5
K_W_ALIGN = 1.0
ALIGN_TOL = math.radians(2.0)
DIST_TOL = 0.01
LIVE_BAND = 0.08        # m, 측정해 둔 벽선 주변 이 거리 안의 점만 사용
SAFETY_GAP = 0.03       # m, 진행 방향 로봇 끝과 장애물 사이가 이보다 가까우면 멈춤
CONTROL_DT = 0.05
APPROACH_TIMEOUT = 40.0
FINE_TIMEOUT = 40.0


def clip_mag(v, lo, hi):
    """|v|를 [lo, hi]로 (0은 0 그대로) -> 너무 작은 속도는 모터가 안 움직여서 최소값 적용"""
    if v == 0.0:
        return 0.0
    return math.copysign(min(max(abs(v), lo), hi), v)


def search_dir(yaw, facing):
    """벽을 찾을 방향 단위벡터: back이면 로봇 뒤쪽, front면 앞쪽"""
    d = np.array([math.cos(yaw), math.sin(yaw)])
    return -d if facing == 'back' else d


# ---------------------------------------------------------------------------
# 벽(직선) 검출
# ---------------------------------------------------------------------------
def fit_line(pts):
    """PCA 직선 -> (중심, 방향 단위벡터, 법선 단위벡터)"""
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    u = vt[0]
    return c, u, np.array([-u[1], u[0]])


def extract_segments(pts, rng):
    """RANSAC으로 직선을 차례로 뽑고, 직선 위 점들을 끊긴 곳에서 나눠 벽 구간 리스트로"""
    segs = []
    rest = pts
    for _ in range(MAX_LINES):
        if len(rest) < SEG_MIN_POINTS:
            break
        best = None
        best_n = 0
        for _ in range(RANSAC_ITERS):
            i, j = rng.choice(len(rest), 2, replace=False)
            d = rest[j] - rest[i]
            length = math.hypot(d[0], d[1])
            if length < 0.05:
                continue
            n = np.array([-d[1], d[0]]) / length
            inl = np.abs((rest - rest[i]) @ n) < LINE_TOL
            cnt = int(inl.sum())
            if cnt > best_n:
                best, best_n = inl, cnt
        if best is None or best_n < SEG_MIN_POINTS:
            break
        c, u, n = fit_line(rest[best])
        inl = np.abs((rest - c) @ n) < LINE_TOL
        line_pts, rest = rest[inl], rest[~inl]
        t = (line_pts - c) @ u
        order = np.argsort(t)
        line_pts, t = line_pts[order], t[order]
        breaks = np.nonzero(np.diff(t) > SEG_MAX_GAP)[0] + 1
        for chunk in np.split(line_pts, breaks):
            if len(chunk) < SEG_MIN_POINTS:
                continue
            cc, uu, nn = fit_line(chunk)
            tt = (chunk - cc) @ uu
            if tt.max() - tt.min() < SEG_MIN_LEN:
                continue
            p1, p2 = cc + uu * tt.min(), cc + uu * tt.max()
            segs.append({'p1': p1, 'p2': p2, 'mid': (p1 + p2) / 2.0, 'dir': uu, 'n': nn,
                         'len': float(tt.max() - tt.min())})
    return segs


def merge_collinear(wall, segs):
    """wall과 같은 직선 위의 구간들을 합쳐서 끝점을 넓힘
    (라이다가 벽을 비스듬히 보면 점 간격이 벌어져서 한 벽이 여러 구간으로 끊김)"""
    p1 = wall['p1']
    t0, t1 = 0.0, wall['len']
    for s in segs:
        if (abs(float(s['dir'] @ wall['dir'])) > math.cos(math.radians(10))
                and abs(float((s['mid'] - p1) @ wall['n'])) < 2 * LINE_TOL):
            ts = sorted((float((s['p1'] - p1) @ wall['dir']), float((s['p2'] - p1) @ wall['dir'])))
            t0, t1 = min(t0, ts[0]), max(t1, ts[1])
    out = dict(wall)
    out['p1'], out['p2'] = p1 + wall['dir'] * t0, p1 + wall['dir'] * t1
    out['mid'] = (out['p1'] + out['p2']) / 2.0
    out['len'] = t1 - t0
    return out


def choose_wall(segs, pose, facing):
    """로봇 방향(back이면 뒤쪽)으로 뻗은 직선이 지나가는 벽 중 가장 가까운 것.
    벽은 로봇 방향과 거의 수직이어야 함. 법선은 로봇 쪽(벽 앞)을 향하게"""
    o = np.array(pose[:2])
    d = search_dir(pose[2], facing)
    best, best_t = None, math.inf
    for s in segs:
        if abs(float(s['dir'] @ d)) > math.sin(WALL_PERP_TOL):
            continue
        # 로봇에서 d 방향 직선과 벽 직선의 교점: o + t d = p1 + u dir
        den = float(d[0] * s['dir'][1] - d[1] * s['dir'][0])
        if abs(den) < 1e-6:
            continue
        r = s['p1'] - o
        t = float(r[0] * s['dir'][1] - r[1] * s['dir'][0]) / den
        u = float(r[0] * d[1] - r[1] * d[0]) / den
        if t <= 0 or not (-RAY_MARGIN <= u <= s['len'] + RAY_MARGIN):
            continue
        if t < best_t:
            best, best_t = s, t
    if best is None:
        return None
    wall = dict(best)
    if float(wall['n'] @ (o - wall['mid'])) < 0:
        wall['n'] = -wall['n']
    return wall


def line_wall(spot):
    """직선 spot의 그린 선 -> 벽 dict (n = 벽에서 로봇 쪽)"""
    p1, p2 = (np.asarray(q, dtype=float) for q in spot['line'])
    length = float(np.hypot(*(p2 - p1)))
    return {'p1': p1, 'p2': p2, 'mid': (p1 + p2) / 2.0, 'dir': (p2 - p1) / max(length, 1e-9),
            'n': np.asarray(spot['normal'], dtype=float), 'len': length}


def line_free_normal(p1, p2, m):
    """선의 양쪽 중 맵에서 빈 공간이 더 넓은 쪽 법선 (맵이 없거나 같으면 p1->p2의 왼쪽)"""
    p1, p2 = np.asarray(p1, dtype=float), np.asarray(p2, dtype=float)
    d = (p2 - p1) / max(float(np.hypot(*(p2 - p1))), 1e-9)
    n = np.array([-d[1], d[0]])
    if m is not None:
        mid = (p1 + p2) / 2.0
        left = m.clearance_at(*(mid + n * 0.25))
        right = m.clearance_at(*(mid - n * 0.25))
        if right > left:
            n = -n
    return n


def spot_approach(spot):
    """직선 spot의 접근 자세 (nav 단계 / Go to spot의 Nav2 목표): 선 중심점 앞, pre-park 첫 거리"""
    return wall_pose(line_wall(spot), HALF_LEN + float(spot.get('gap', DEFAULT_GAP))
                     + PRE_PARK_EXTRA, spot.get('facing', 'back'))


def wall_pose(wall, dist, facing):
    """벽 중심점에서 법선 방향 dist 떨어진 자세 (back: 벽을 등짐 = 벽 반대 방향, front: 벽을 바라봄)"""
    p = wall['mid'] + wall['n'] * dist
    n = wall['n']
    yaw = math.atan2(n[1], n[0]) if facing == 'back' else math.atan2(-n[1], -n[0])
    return (float(p[0]), float(p[1]), yaw)


# ---------------------------------------------------------------------------
# 주차 미션
# ---------------------------------------------------------------------------
class Parker:
    def __init__(self, robot):
        """robot: robot_interface.RobotInterface"""
        self.robot = robot
        self.rng = np.random.default_rng()

    def _viz(self, **kw):
        with self.robot.lock:
            for k, v in kw.items():
                if v is None:
                    self.robot.viz.pop(k, None)
                else:
                    self.robot.viz[k] = v

    def measure_wall(self, facing, prev=None, line=None):
        """스캔 여러 장을 map 좌표로 모아서 로봇 뒤쪽(front면 앞쪽) 벽 구간 검출 -> 주차할 벽 하나
        line(그린 선)이 있으면 그 선 근처 점만 쓰고 선과 평행한 벽 중 가장 긴 것
        prev가 있으면 처음 고른 벽과 같은 벽 (방향 비슷 + 이전 중심점이 그 직선 근처)"""
        pose = self.robot.robot_pose()
        if pose is None:
            raise MissionError('no map->base_link TF (set the initial pose first)')
        pts_list = []
        for scan in self.robot.wait_scans(WALL_SCANS):
            p = self.robot.scan_in_frame(scan, 'map', WALL_SEARCH_RANGE + 0.5)
            if p is not None:
                pts_list.append(p)
        if not pts_list:
            raise MissionError('no map->laser_frame TF (is AMCL running?)')
        pts = np.vstack(pts_list)
        rel = pts - np.array(pose[:2])
        if line is not None:
            lr = pts - line['p1']
            along = lr @ line['dir']
            keep = ((np.abs(lr @ line['n']) < LINE_BAND) & (along > -LINE_END_MARGIN)
                    & (along < line['len'] + LINE_END_MARGIN))
        elif prev is None:
            # 찾는 방향 반쪽 + 거리 제한 (다른 벽이 덜 섞이게)
            keep = ((rel @ search_dir(pose[2], facing) > 0)
                    & (np.hypot(rel[:, 0], rel[:, 1]) < WALL_SEARCH_RANGE))
        else:
            keep = np.hypot(rel[:, 0], rel[:, 1]) < WALL_SEARCH_RANGE
        pts = pts[keep]
        where = ('near the drawn line' if line is not None
                 else 'behind the robot' if facing == 'back' else 'in front of the robot')
        if len(pts) < SEG_MIN_POINTS:
            raise MissionError(f'lidar sees no wall points {where}')
        segs = extract_segments(pts, self.rng)
        self._viz(segments=segs)
        if prev is None and line is not None:
            cands = [s for s in segs
                     if abs(float(s['dir'] @ line['dir'])) > math.cos(LINE_ANGLE_TOL)]
            if not cands:
                raise MissionError(f'no straight wall found {where}')
            wall = dict(max(cands, key=lambda s: s['len']))
            if float(wall['n'] @ line['n']) < 0:
                wall['n'] = -wall['n']
        elif prev is None:
            wall = choose_wall(segs, pose, facing)
            if wall is None:
                raise MissionError(f'no straight wall found {where}')
        else:
            same = [s for s in segs
                    if abs(float(s['dir'] @ prev['dir'])) > math.cos(math.radians(20))
                    and abs(float((prev['mid'] - s['mid']) @ s['n'])) < 0.15]
            if not same:
                self.robot.get_logger().warn('Wall not seen again, using the first measurement')
                return prev
            wall = dict(max(same, key=lambda s: s['len']))
            if float(wall['n'] @ prev['n']) < 0:
                wall['n'] = -wall['n']
        wall = merge_collinear(wall, segs)
        self._viz(wall=wall)
        self.robot.get_logger().info(
            f'wall mid=({wall["mid"][0]:.2f}, {wall["mid"][1]:.2f}) len={wall["len"]:.2f} '
            f'n=({wall["n"][0]:.2f}, {wall["n"][1]:.2f})')
        return wall

    def standoff(self, spot):
        """base_link ~ 벽 거리. front는 벽이 라이다 range_min보다 멀어야 보임"""
        gap = float(spot.get('gap', DEFAULT_GAP))
        if spot.get('facing', 'back') == 'front':
            _, scan = self.robot.latest_scan()
            range_min = scan.range_min if scan is not None else 0.15
            lo = range_min + 0.02 + self.robot.laser_offset()[0] - HALF_LEN
        else:
            lo = 0.03
        if gap < lo:
            self.robot.get_logger().warn(f'gap {gap:.3f} too small, using {lo:.3f}')
            gap = lo
        return HALF_LEN + gap

    def motion_gap(self, sign):
        """진행 방향(sign=+1 앞, -1 뒤)으로 로봇 끝과 가장 가까운 장애물 사이 거리"""
        _, scan = self.robot.latest_scan()
        pts = self.robot.base_points(scan)
        x = pts[:, 0] * sign
        sel = (x > 0) & (np.abs(pts[:, 1]) < HALF_WIDTH + 0.02)
        if not sel.any():
            return float('inf')
        return float(x[sel].min()) - HALF_LEN

    @staticmethod
    def make_result(wall, target):
        return {'wall_mid': [round(float(v), 3) for v in wall['mid']],
                'wall_normal': [round(float(v), 4) for v in wall['n']],
                'wall_len': round(float(wall['len']), 3),
                'park_pose': [round(target[0], 3), round(target[1], 3), round(target[2], 4)],
                'time': time.strftime('%Y-%m-%d %H:%M:%S')}

    def go_to_spot(self, spot):
        self.robot.go_to((spot['x'], spot['y'], spot['yaw']), f'Go to {spot["name"]}')

    def detect(self, spot):
        """현재 위치에서 움직이지 않고 벽 검출 + 주차 자세 표시 (확인용) -> result"""
        self._viz(segments=None, wall=None, target=None)
        self.robot.set_status(f'Detect wall ({spot["name"]}): measuring ...')
        facing = spot.get('facing', 'back')
        wall = self.measure_wall(facing, line=line_wall(spot) if spot.get('line') else None)
        target = wall_pose(wall, self.standoff(spot), facing)
        self._viz(target=target)
        self.robot.set_status(
            f'Wall: len {wall["len"]:.2f} m, mid ({wall["mid"][0]:.2f}, {wall["mid"][1]:.2f}) '
            f'-> park x={target[0]:.2f} y={target[1]:.2f} yaw={math.degrees(target[2]):.0f}')
        return self.make_result(wall, target)

    def park(self, spot):
        """현재 위치(spot 근처)에서 벽 검출 -> 벽 중심점에 주차 -> result"""
        name = spot['name']
        facing = spot.get('facing', 'back')
        standoff = self.standoff(spot)
        line = line_wall(spot) if spot.get('line') else None
        self._viz(segments=None, wall=None, target=None)

        # 1. 벽 찾기
        self.robot.set_status(f'Park {name}: measuring the wall with lidar ...')
        wall = self.measure_wall(facing, line=line)
        target = wall_pose(wall, standoff, facing)
        self._viz(target=target)
        with self.robot.lock:
            m = self.robot.map
        if m is not None and m.clearance_at(target[0], target[1]) <= 0.0:
            self.robot.get_logger().warn('park pose is not free space on the map')

        # 2. 벽 중심점 앞 pre-park 자세로 (cmd_vel)
        self.robot.set_status(
            f'Park {name}: drive to pre-park pose ({standoff + PRE_PARK_EXTRA:.2f} m from wall) ...')
        self.drive_to_pose(wall_pose(wall, standoff + PRE_PARK_EXTRA, facing))
        self.robot.sleep(1.0)

        # 3. 가까이서 다시 측정 -> 직접 접근
        wall = self.measure_wall(facing, prev=wall, line=line)
        target = wall_pose(wall, standoff, facing)
        self._viz(target=target)
        self.robot.set_status(f'Park {name}: final approach ...')
        d, ang_err, offset = self.final_approach(wall, standoff, facing)
        self.robot.set_status(
            f'Parked at {name}: wall dist {d:.3f} m (target {standoff:.3f}), '
            f'angle {math.degrees(ang_err):+.1f} deg, offset from wall middle {offset:+.3f} m')
        return self.make_result(wall, target)

    def drive_to_pose(self, pose):
        """Nav2 없이 cmd_vel로 pose(map)까지: 목표점으로 직진(앞/뒤 중 가까운 쪽) -> 목표 방향으로 제자리 회전
        목표를 odom 좌표로 고정 (이동 중 AMCL 보정으로 map 좌표가 튀어도 영향 없게)"""
        r = self.robot
        map_odom = r.lookup_pose('map', 'odom')
        if map_odom is None:
            raise MissionError('no map->odom TF')
        target = compose(inverse(map_odom), pose)
        end = time.time() + PRE_PARK_TIMEOUT
        sign = None
        try:
            # (1) 목표점까지 이동 (목표가 뒤쪽이면 후진해서 회전을 줄임)
            while True:
                r.check()
                if time.time() > end:
                    raise MissionError('pre-park drive timeout')
                rb = r.lookup_pose('odom', 'base_link')
                if rb is None:
                    time.sleep(CONTROL_DT)
                    continue
                tx, ty, _ = compose(inverse(rb), target)
                if sign is None:
                    sign = 1.0 if tx >= 0.0 else -1.0
                along = sign * tx
                dist = math.hypot(tx, ty)
                if dist < PRE_PARK_POS_TOL or along < 0.01:
                    break
                if self.motion_gap(sign) < SAFETY_GAP:
                    r.get_logger().warn('obstacle too close, stop pre-park drive')
                    break
                alpha = math.atan2(sign * ty, sign * tx)
                if abs(alpha) > 0.35 and dist > 0.05:
                    r.drive(0.0, clip_mag(K_W * alpha, MIN_W, MAX_W))
                else:
                    w = clip_mag(K_W * alpha, 0.0, MAX_W) if dist > 0.05 else 0.0
                    r.drive(sign * clip_mag(K_V * along, MIN_V, PRE_PARK_MAX_V), w)
                time.sleep(CONTROL_DT)
            r.stop_robot()
            r.sleep(0.3)

            # (2) 목표 방향으로 회전
            while True:
                r.check()
                if time.time() > end:
                    raise MissionError('pre-park rotate timeout')
                rb = r.lookup_pose('odom', 'base_link')
                if rb is None:
                    time.sleep(CONTROL_DT)
                    continue
                err = wrap(target[2] - rb[2])
                if abs(err) < PRE_PARK_YAW_TOL:
                    break
                r.drive(0.0, clip_mag(K_W_ALIGN * err, MIN_W, MAX_W))
                time.sleep(CONTROL_DT)
        finally:
            r.stop_robot()

    def fit_wall_live(self, mid_o, dir_o, half_len, cache):
        """최신 스캔에서 측정해 둔 벽선 근처 점만 골라 odom 좌표로 직선 맞춤 -> (중심, 법선) 또는 None
        스캔 시각의 odom TF로 변환해서 라이다 지연 동안 움직인 만큼을 보정"""
        seq, scan = self.robot.latest_scan()
        if cache.get('seq') == seq:
            return cache.get('fit')
        lp = self.robot.lookup_pose('odom', LASER_FRAME, Time.from_msg(scan.header.stamp))
        if lp is None:
            lp = self.robot.lookup_pose('odom', LASER_FRAME)
        fit = None
        if lp is not None:
            pts = transform_pts(lp, scan_points(scan, 2.0))
            n_o = np.array([-dir_o[1], dir_o[0]])
            rel = pts - mid_o
            sel = (np.abs(rel @ n_o) < LIVE_BAND) & (np.abs(rel @ dir_o) < max(half_len, 0.15))
            if sel.sum() >= 6:
                c, _, n = fit_line(pts[sel])
                fit = (c, n)
        cache['seq'], cache['fit'] = seq, fit
        return fit

    def final_approach(self, wall, standoff, facing):
        r = self.robot
        sign = 1.0 if facing == 'front' else -1.0      # back이면 후진으로 벽에 붙음
        # 목표를 odom 좌표로 고정 (접근 중 AMCL 보정으로 map 좌표가 튀어도 영향 없게)
        map_odom = r.lookup_pose('map', 'odom')
        if map_odom is None:
            raise MissionError('no map->odom TF')
        o_from_m = inverse(map_odom)
        target = compose(o_from_m, wall_pose(wall, standoff, facing))
        mid_o = transform_pts(o_from_m, wall['mid'][None, :])[0]
        dir_o = rotate_vec(o_from_m[2], wall['dir'])
        n_ref = rotate_vec(o_from_m[2], wall['n'])

        d = ang_err = float('nan')
        try:
            # (1) 목표점까지 이동
            end = time.time() + APPROACH_TIMEOUT
            while True:
                r.check()
                rb = r.lookup_pose('odom', 'base_link')
                if rb is None:
                    time.sleep(CONTROL_DT)
                    continue
                tx, ty, _ = compose(inverse(rb), target)
                if time.time() > end:
                    raise MissionError(f'approach timeout (target in robot frame '
                                       f'x={tx:+.2f} y={ty:+.2f})')
                along = sign * tx
                dist = math.hypot(tx, ty)
                if along < 0.01:
                    break
                if self.motion_gap(sign) < SAFETY_GAP:
                    r.get_logger().warn('obstacle too close, stop approach')
                    break
                alpha = math.atan2(sign * ty, sign * tx)
                if abs(alpha) > 0.35 and dist > 0.05:
                    r.drive(0.0, clip_mag(K_W * alpha, MIN_W, MAX_W))
                else:
                    w = clip_mag(K_W * alpha, 0.0, MAX_W) if dist > 0.05 else 0.0
                    r.drive(sign * clip_mag(K_V * along, MIN_V, MAX_V), w)
                time.sleep(CONTROL_DT)
            r.stop_robot()
            r.sleep(0.5)

            # (2) 라이다로 벽과 직각 + 거리 맞추기
            cache = {}
            end = time.time() + FINE_TIMEOUT
            ok_count = 0
            misses = 0
            while True:
                r.check()
                if time.time() > end:
                    raise MissionError('fine alignment timeout')
                rb = r.lookup_pose('odom', 'base_link')
                fit = self.fit_wall_live(mid_o, dir_o, wall['len'] / 2.0, cache)
                if rb is None or fit is None:
                    misses += 1
                    if misses > 3.0 / CONTROL_DT:
                        raise MissionError('lost the wall in lidar')
                    r.stop_robot()
                    time.sleep(CONTROL_DT)
                    continue
                misses = 0
                c, n = fit
                if float(n @ n_ref) < 0:
                    n = -n
                d = float((np.array(rb[:2]) - c) @ n)            # base_link ~ 벽
                want = math.atan2(n[1], n[0]) if facing == 'back' else math.atan2(-n[1], -n[0])
                ang_err = wrap(want - rb[2])
                dist_err = d - standoff
                if abs(ang_err) > ALIGN_TOL:
                    ok_count = 0
                    r.drive(0.0, clip_mag(K_W_ALIGN * ang_err, MIN_W, MAX_W))
                elif abs(dist_err) > DIST_TOL:
                    ok_count = 0
                    # 벽 쪽으로 가는 방향이 sign, 멀면(dist_err > 0) 벽 쪽으로
                    v = sign * clip_mag(K_V * dist_err, MIN_V, MAX_V)
                    if v * sign > 0 and self.motion_gap(sign) < SAFETY_GAP:
                        v = 0.0
                    r.drive(v, K_W_ALIGN * ang_err)
                else:
                    ok_count += 1
                    r.stop_robot()
                    if ok_count >= 3:
                        break
                time.sleep(CONTROL_DT)
        finally:
            r.stop_robot()

        rb = r.lookup_pose('odom', 'base_link')
        offset = float((np.array(rb[:2]) - mid_o) @ dir_o) if rb is not None else float('nan')
        return d, ang_err, offset


def store_result(mission_file, name, result):
    """미션 파일의 spot에 결과 저장 (다른 내용은 그대로)"""
    doc = load_mission(mission_file)
    spot = find_spot(doc, name)
    if spot is not None:
        spot['result'] = result
        save_mission(mission_file, doc)


def main(args=None):
    """GUI 없이 미션 파일의 spot에 자동 주차"""
    # Ctrl+C를 KeyboardInterrupt로 받아서 정지 명령을 보낸 뒤 종료 (rclpy 기본 처리는 바로 컨텍스트를 내림)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotInterface('parking')
    share = get_package_share_directory('pros2_robot')
    map_yaml = node.declare_parameter('map_yaml', os.path.join(share, 'maps', 'map4.yaml')).value
    mission_file = (node.declare_parameter('mission_file', '').value
                    or default_mission_file(map_yaml))
    name = node.declare_parameter('spot', '').value

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    rc = 1
    try:
        doc = load_mission(mission_file)
        spots = doc['parking']
        spot = find_spot(doc, name) if name else (spots[0] if spots else None)
        if spot is None:
            raise MissionError(f'no parking spot "{name}" in {mission_file}')
        try:
            node.set_map(MapData.from_yaml(map_yaml))
        except (OSError, KeyError, ValueError) as e:
            node.get_logger().warn(f'Cannot read {map_yaml}: {e}')
        node.wait_until(lambda: node.robot_pose() is not None, 30.0,
                        'map->base_link TF (is navigation running?)')
        result = Parker(node).park(spot)
        store_result(mission_file, spot['name'], result)
        rc = 0
    except (MissionError, Cancelled) as e:
        node.get_logger().error(f'Parking failed: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        time.sleep(0.2)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
