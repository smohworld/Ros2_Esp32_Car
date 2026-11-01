"""
맵에서 로봇 위치 찾기 (AMCL 전역 위치 추정)
"""
import math

import numpy as np

from pros2_robot.robot_interface import MissionError

LOC_W = 0.5                     # rad/s, 회전 속도
LOC_STEP = math.radians(45)     # 한 번에 도는 각도
LOC_PAUSE = 0.6                 # s, 멈춰서 AMCL 갱신 기다리는 시간
MIN_TURNS = 1.0                 # 바퀴, 이만큼은 돌고 나서 수렴 판단
CONV_XY = 0.15                  # m, 위치 표준편차
CONV_YAW = math.radians(10)     # 방향 표준편차
MATCH_DIST = 0.10               # m, 라이다 점 ~ 맵 벽
MATCH_OK = 0.60                 # 일치율 기준
GUESS_SXY = 1.0                 # m, 대략 위치를 줬을 때 위치 불확실도


class Localizer:
    def __init__(self, robot):
        """robot: robot_interface.RobotInterface"""
        self.robot = robot

    def amcl_state(self):
        """-> (time, pose, std_xy, std_yaw) 또는 None"""
        with self.robot.lock:
            return self.robot.amcl

    def scan_match(self):
        """현재 추정 위치에서 최신 스캔이 맵의 벽과 겹치는 비율 (0~1), 모르면 None"""
        r = self.robot
        _, scan = r.latest_scan()
        with r.lock:
            m = r.map
        if scan is None or m is None:
            return None
        pts = r.scan_in_frame(scan, 'map')
        if pts is None or len(pts) < 10:
            return None
        return float(np.mean(m.clearance_pts(pts) < MATCH_DIST))

    def quality(self):
        """-> (std_xy, std_yaw, match) 모르는 값은 None (GUI 표시용)"""
        st = self.amcl_state()
        return (st[2] if st else None, st[3] if st else None, self.scan_match())

    @staticmethod
    def converged(std_xy, std_yaw, match):
        return (std_xy is not None and std_xy < CONV_XY and std_yaw < CONV_YAW
                and match is not None and match >= MATCH_OK)

    @staticmethod
    def describe(std_xy, std_yaw, match):
        s = (f'std {std_xy:.2f} m / {math.degrees(std_yaw):.0f} deg'
             if std_xy is not None else 'no /amcl_pose')
        return s + (f', scan match {match * 100:.0f}%' if match is not None else '')

    def localize(self, turns=2.0, guess=None):
        r = self.robot
        if guess is None:
            r.set_status('Localization: spreading particles over the whole map ...')
            r.call_service(r.global_loc_client, 'reinitialize_global_localization')
        else:
            r.publish_initial_pose(guess, sxy=GUESS_SXY, syaw=math.pi)
        r.sleep(0.5)

        total = max(float(turns), MIN_TURNS) * 2.0 * math.pi
        turned = 0.0
        q = (None, None, None)
        while turned < total - 1e-6:
            r.rotate(LOC_STEP, LOC_W)
            turned += LOC_STEP
            r.sleep(LOC_PAUSE)
            r.call_service(r.nomotion_client, 'request_nomotion_update')
            r.sleep(0.3)
            q = self.quality()
            r.set_status(f'Localization: turned {math.degrees(turned):.0f} deg, '
                         + self.describe(*q))
            if turned >= MIN_TURNS * 2.0 * math.pi - 1e-6 and self.converged(*q):
                break
        if not self.converged(*q):
            raise MissionError('not converged (' + self.describe(*q)
                               + '). Try again or give a rough pose')
        pose = self.amcl_state()[1]
        r.set_status(f'Localized: x={pose[0]:.2f} y={pose[1]:.2f} '
                     f'yaw={math.degrees(pose[2]):.0f}, ' + self.describe(*q))
        return pose
