#!/usr/bin/env python3
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from pros2_robot.localization import Localizer
from pros2_robot.map_utils import (default_mission_file, find_place, find_spot, load_mission,
                                   MapData, STEP_TYPES)
from pros2_robot.parking import Parker, store_result
from pros2_robot.robot_interface import Cancelled, MissionError, RobotInterface

TOY_CONF = 0.90                 # toy_tracker --conf와 같게 (이 이상이어야 따라가기 시작)
TOY_CONFIRM = 3                 # 연속 검출 수
TOY_STALE = 1.0                 # s, /toy_detection이 이보다 오래되면 검출기 꺼진 것
SEARCH_STEP = math.radians(30)  # 한 번에 도는 각도 (카메라 지연/흔들림 때문에 돌고 멈춰서 봄)
SEARCH_W = 0.4                  # rad/s (스키드 스티어는 0.3 이하로는 잘 안 돎)
SEARCH_PAUSE = 1.0              # s, 멈춰서 보는 시간
FOLLOW_SETTLE = 0.5             # s, 따라가기를 끈 뒤 정지 대기


class MissionRunner:
    def __init__(self, robot, parker=None, on_result=None):
        """robot: RobotInterface, on_result(spot_name, result): 주차 결과 저장 (없으면 저장 안 함)"""
        self.robot = robot
        self.parker = parker or Parker(robot)
        self.localizer = Localizer(robot)
        self.on_result = on_result

    # ---------- 대상 ----------
    @staticmethod
    def resolve(doc, step):
        """-> (place, spot) 둘 중 하나는 None, target이 없는 단계는 (None, None)"""
        kind = STEP_TYPES[step['type']][1]
        if kind is None:
            return None, None
        name = step.get('target')
        if not name:
            raise MissionError(f'{STEP_TYPES[step["type"]][0]}: no target set')
        place = find_place(doc, name) if kind in ('any', 'place') else None
        spot = find_spot(doc, name) if kind in ('any', 'parking') and place is None else None
        if place is None and spot is None:
            raise MissionError(f'{STEP_TYPES[step["type"]][0]}: "{name}" not found')
        return place, spot

    # ---------- 실행 ----------
    def run(self, doc, start=0, only=False):
        """doc['mission']의 start번째(0부터) 단계부터 끝까지 (only면 그 단계만)"""
        steps = doc.get('mission', [])
        if not 0 <= start < len(steps):
            raise MissionError('no mission steps (add them in the Mission tab)')
        todo = range(start, start + 1 if only else len(steps))
        targets = {i: self.resolve(doc, steps[i]) for i in todo}     # 시작 전에 전부 확인
        for i in todo:
            with self.robot.lock:
                self.robot.mission_step = i
            step = steps[i]
            label = f'Step {i + 1}/{len(steps)} {STEP_TYPES[step["type"]][0]}'
            self.robot.get_logger().info(f'{label} {step.get("target", "")}')
            try:
                self.run_step(step, *targets[i], label)
            except MissionError as e:
                raise MissionError(f'{label}: {e}') from e
        self.robot.set_status('Mission complete' if not only else f'Step {start + 1} done')

    def run_step(self, step, place, spot, label):
        t = step['type']
        if t == 'localize':
            self.localizer.localize(step['turns'])
        elif t == 'search_toy':
            self.search_toy(step['turns'], label)
        elif t == 'follow':
            self.follow(place, step['timeout'], label)
        elif t == 'park':
            self.park(spot)
            if step['wait'] > 0:
                self.robot.set_status(f'{label}: parked, waiting {step["wait"]:g} s')
                self.robot.sleep(step['wait'])
        elif t == 'final' and spot is not None:
            # 주차는 Nav2를 쓰지 않으므로 spot 근처까지 먼저 Nav2로 이동
            self.robot.go_to((spot['x'], spot['y'], spot['yaw']), f'{label} {spot["name"]}')
            self.park(spot)
        elif place is not None:
            self.robot.go_to((place['x'], place['y'], place['yaw']), f'{label} {place["name"]}')
        else:
            self.robot.go_to((spot['x'], spot['y'], spot['yaw']), f'{label} {spot["name"]}')

    def park(self, spot):
        result = self.parker.park(dict(spot))
        if self.on_result is not None:
            self.on_result(spot['name'], result)

    # ---------- 장난감 ----------
    def toy(self):
        with self.robot.lock:
            return self.robot.toy_seq, self.robot.toy

    def wait_detector(self):
        self.robot.wait_until(
            lambda: (self.toy()[1] is not None
                     and time.time() - self.toy()[1][0] < TOY_STALE),
            3.0, '/toy_detection (is toy_detector running?)')

    def toy_visible(self):
        _, toy = self.toy()
        return (toy is not None and time.time() - toy[0] < TOY_STALE
                and toy[1][0] > 0.5 and toy[1][5] >= TOY_CONF)

    def watch_toy(self, duration):
        """duration 동안 새 검출을 보며 TOY_CONFIRM번 연속 검출되면 True"""
        end = time.time() + duration
        last, count = None, 0
        while time.time() < end:
            self.robot.check()
            seq, toy = self.toy()
            if toy is not None and seq != last:
                last = seq
                d = toy[1]
                count = count + 1 if d[0] > 0.5 and d[5] >= TOY_CONF else 0
                if count >= TOY_CONFIRM:
                    return True
            time.sleep(0.03)
        return False

    def search_toy(self, turns, label):
        r = self.robot
        r.set_follow(False)
        self.wait_detector()
        total = max(float(turns), 0.0) * 2.0 * math.pi
        turned = 0.0
        while True:
            r.set_status(f'{label}: looking ... (turned {math.degrees(turned):.0f} deg)')
            if self.watch_toy(SEARCH_PAUSE):
                r.set_status(f'{label}: toy found')
                return
            if turned >= total - 1e-6:
                raise MissionError(f'toy not found after {math.degrees(turned):.0f} deg')
            r.rotate(SEARCH_STEP, SEARCH_W, until=self.toy_visible)
            turned += SEARCH_STEP

    def follow(self, place, timeout, label):
        r = self.robot
        if r.follow_pub.get_subscription_count() == 0:
            raise MissionError('toy_tracker not running '
                               '(ros2 run pros2_robot toy_tracker --start-disabled)')
        self.wait_detector()
        tx, ty, radius = place['x'], place['y'], place['radius']
        end = time.time() + timeout
        r.set_follow(True)
        try:
            while True:
                r.check()
                p = r.robot_pose()
                if p is not None:
                    d = math.hypot(p[0] - tx, p[1] - ty)
                    if d <= radius:
                        break
                    r.set_status(f'{label}: following, {d:.2f} m to {place["name"]} '
                                 f'(radius {radius:.2f})')
                if time.time() > end:
                    raise MissionError(f'did not reach {place["name"]} in {timeout:.0f} s')
                _, toy = self.toy()
                if toy is None or time.time() - toy[0] > 2.0:
                    raise MissionError('/toy_detection stopped (toy_detector?)')
                time.sleep(0.1)
        finally:
            r.set_follow(False)
            r.stop_robot()
        r.sleep(FOLLOW_SETTLE)
        r.set_status(f'{label}: reached {place["name"]}')


def main(args=None):
    """GUI 없이 미션 파일의 단계를 순서대로 실행"""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotInterface('mission')
    share = get_package_share_directory('pros2_robot')
    map_yaml = node.declare_parameter('map_yaml', os.path.join(share, 'maps', 'map4.yaml')).value
    mission_file = (node.declare_parameter('mission_file', '').value
                    or default_mission_file(map_yaml))
    start = int(node.declare_parameter('start', 1).value) - 1

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    rc = 1
    try:
        doc = load_mission(mission_file)
        try:
            node.set_map(MapData.from_yaml(map_yaml))
        except (OSError, KeyError, ValueError) as e:
            node.get_logger().warn(f'Cannot read {map_yaml}: {e}')
        steps = doc['mission']
        if not (0 <= start < len(steps) and steps[start]['type'] == 'localize'):
            node.wait_until(lambda: node.robot_pose() is not None, 30.0,
                            'map->base_link TF (is navigation running?)')
        MissionRunner(node, on_result=lambda n, res: store_result(mission_file, n, res)).run(
            doc, start)
        rc = 0
    except (MissionError, Cancelled) as e:
        node.get_logger().error(f'Mission failed: {e}')
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
