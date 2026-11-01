"""
맵(.yaml/.pgm) 읽기, 2D 기하 유틸, 미션 파일(<맵 이름>_mission.json) 읽기/쓰기

pose = (x, y, yaw), 좌표는 모두 map 프레임 [m]

미션 파일 (GUI가 저장, 주차 노드/미션이 자동으로 읽음)
{
  "map": "/.../map4.yaml",
  "parking": [
    {
      "name": "parking1",
      "x": 4.29, "y": -3.86, "yaw": -1.57,   Nav2로 먼저 갈 자세 (GUI에서 드래그)
      "facing": "back",                       back: 벽을 등지고 주차 (벽은 이 자세의 뒤쪽에서 찾음)
                                              front: 벽을 바라보고 주차 (벽은 앞쪽에서 찾음)
      "gap": 0.10,                            로봇 끝 ~ 벽 거리 [m]
      "line": [[x1, y1], [x2, y2]],           (선택) GUI에서 그린 주차 벽 직선. 있으면 x, y, yaw는
      "normal": [nx, ny],                     이 선에서 계산한 접근 자세이고, 라이다는 이 선 근처의
                                              벽만 찾음. normal = 벽에서 로봇 쪽(빈 공간) 방향
      "result": {                             마지막 주차/벽 검출 결과 (없을 수 있음)
        "wall_mid": [x, y], "wall_normal": [nx, ny], "wall_len": 0.8,
        "park_pose": [x, y, yaw], "time": "2026-10-25 15:00:00"
      }
    }
  ],
  "places": [                                 Nav2 이동 위치 / 따라가기 도착 영역
    {"name": "area1", "x": 1.0, "y": 2.0, "yaw": 0.0,
     "radius": 0.5}                           따라가기: 로봇이 이 반경 안에 들어오면 도착
  ],
  "mission": [                                순서대로 실행할 단계 (mission_runner.py)
    {"type": "nav", "target": "area1"},       target = places 또는 parking 이름
    {"type": "park", "target": "parking1", "wait": 3.0},  주차 후 wait초 대기
    {"type": "search_toy", "turns": 2.0},     제자리 회전하며 장난감 찾기 (최대 회전 수)
    {"type": "follow", "target": "area2", "timeout": 300},
    {"type": "final", "target": "parking1"}   parking이면 주차, place면 Nav2 이동
  ]
}
"""
import copy
import json
import math
import os

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

FACINGS = ('back', 'front')         # back: 벽을 등짐(벽 반대 방향을 바라봄), front: 벽을 바라봄
DEFAULT_GAP = 0.10                  # m, 로봇 끝 ~ 벽
DEFAULT_RADIUS = 0.5                # m, place 도착 반경 (따라가기)

# 미션 단계 종류: (type, GUI 표시 이름, target 종류, 기본 파라미터)
#   target 종류: 'any' = place 또는 parking, 'parking', 'place', None = target 없음
STEP_TYPES = {
    'localize':   ('Localization', None, {'turns': 2.0}),
    'nav':        ('Nav2 move', 'any', {}),
    'park':       ('Parking', 'parking', {'wait': 3.0}),     # wait: 주차 후 대기 (s)
    'search_toy': ('Search toy', None, {'turns': 2.0}),
    'follow':     ('Follow toy', 'place', {'timeout': 300.0}),
    'final':      ('Final position', 'any', {}),
}
# 시나리오 기본 순서: nav2 이동 -> 주차 -> 이동 -> 장난감 탐색 -> 따라가기 -> 최종 위치
DEFAULT_STEPS = ('nav', 'park', 'nav', 'search_toy', 'follow', 'final')
SRC_MAPS = os.path.expanduser('~/pros2_ws/src/pros2_robot/maps')


# ---------------------------------------------------------------------------
# 2D 기하
# ---------------------------------------------------------------------------
def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def transform_pts(pose, pts):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.column_stack((pose[0] + c * pts[:, 0] - s * pts[:, 1],
                            pose[1] + s * pts[:, 0] + c * pts[:, 1]))


def rotate_vec(yaw, v):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_quat_yaw(q, yaw):
    q.x, q.y = 0.0, 0.0
    q.z, q.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def scan_points(scan, max_range=None):
    """LaserScan -> 라이다 좌표 점 (N, 2). 0(측정 실패)/inf/범위 밖은 제외"""
    r = np.asarray(scan.ranges, dtype=np.float32)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    hi = scan.range_max if max_range is None else min(scan.range_max, max_range)
    ok = np.isfinite(r) & (r >= max(scan.range_min, 0.05)) & (r <= hi)
    r, a = r[ok], a[ok]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


# ---------------------------------------------------------------------------
# 맵
# ---------------------------------------------------------------------------
class MapData:
    """grid[h, w] int8 (map_server 규칙: 0 빈칸, 100 벽, -1 미탐색), 행 0 = 원점(y = oy) 쪽"""

    def __init__(self, grid, res, ox, oy, source=''):
        self.grid = grid
        self.res, self.ox, self.oy = float(res), float(ox), float(oy)
        self.h, self.w = grid.shape
        self.source = source
        # 빈 칸에서 가장 가까운 '빈 칸 아닌 칸'(벽/미탐색)까지 거리
        self.clearance = (distance_transform_edt(grid == 0) * self.res).astype(np.float32)

    @classmethod
    def from_yaml(cls, yaml_path):
        """map_server와 같은 규칙(trinary)으로 .yaml/.pgm 읽기"""
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        img = np.asarray(Image.open(img_path).convert('L'), dtype=np.float32)
        occ = img / 255.0 if meta.get('negate', 0) else (255.0 - img) / 255.0
        grid = np.full(img.shape, -1, dtype=np.int8)
        grid[occ > float(meta.get('occupied_thresh', 0.65))] = 100
        grid[occ < float(meta.get('free_thresh', 0.25))] = 0
        return cls(np.flipud(grid), meta['resolution'], meta['origin'][0], meta['origin'][1],
                   yaml_path)

    @classmethod
    def from_msg(cls, msg):
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        return cls(grid, info.resolution, info.origin.position.x, info.origin.position.y, '/map')

    @property
    def extent(self):
        """(x_min, y_min, x_max, y_max)"""
        return (self.ox, self.oy, self.ox + self.w * self.res, self.oy + self.h * self.res)

    def to_rgb(self):
        """화면 표시용 (h, w, 3) uint8, 첫 행 = 맵 위쪽(y 최대)"""
        rgb = np.empty(self.grid.shape + (3,), dtype=np.uint8)
        rgb[:] = (150, 150, 150)
        rgb[self.grid == 0] = (245, 245, 245)
        rgb[self.grid >= 65] = (25, 25, 25)
        return np.ascontiguousarray(np.flipud(rgb))

    def cell(self, x, y):
        return (np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int32),
                np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int32))

    def clearance_at(self, x, y):
        gx, gy = self.cell(x, y)
        if 0 <= gx < self.w and 0 <= gy < self.h:
            return float(self.clearance[gy, gx])
        return 0.0

    def clearance_pts(self, pts):
        """점들 (N, 2)의 clearance (맵 밖은 0) -> (N,)"""
        gx, gy = self.cell(pts[:, 0], pts[:, 1])
        inside = (gx >= 0) & (gx < self.w) & (gy >= 0) & (gy < self.h)
        out = np.zeros(len(pts), dtype=np.float32)
        out[inside] = self.clearance[gy[inside], gx[inside]]
        return out

    def best_free_in_rect(self, x0, y0, x1, y1):
        """사각형 안에서 벽/미탐색 칸과 가장 먼 빈 칸 -> (x, y, 여유 거리) 또는 None"""
        gx0, gy0 = self.cell(x0, y0)
        gx1, gy1 = self.cell(x1, y1)
        gx0, gy0 = max(int(gx0), 0), max(int(gy0), 0)
        gx1, gy1 = min(int(gx1), self.w - 1), min(int(gy1), self.h - 1)
        if gx1 < gx0 or gy1 < gy0:
            return None
        sub = self.clearance[gy0:gy1 + 1, gx0:gx1 + 1]
        iy, ix = np.unravel_index(int(np.argmax(sub)), sub.shape)
        return (self.ox + (gx0 + ix + 0.5) * self.res, self.oy + (gy0 + iy + 0.5) * self.res,
                float(sub[iy, ix]))


# ---------------------------------------------------------------------------
# 미션 파일
# ---------------------------------------------------------------------------
def default_mission_file(map_yaml):
    """설치된(share) 맵이어도 소스 maps/에 같은 맵이 있으면 소스 쪽에 저장 (다시 빌드해도 유지)"""
    name = os.path.splitext(os.path.basename(map_yaml))[0] + '_mission.json'
    if os.path.exists(os.path.join(SRC_MAPS, os.path.basename(map_yaml))):
        return os.path.join(SRC_MAPS, name)
    return os.path.splitext(map_yaml)[0] + '_mission.json'


def normalize_spot(p):
    p = dict(p)
    p['name'] = str(p.get('name', 'parking'))
    p['x'], p['y'] = round(float(p['x']), 3), round(float(p['y']), 3)
    p['yaw'] = round(wrap(float(p.get('yaw', 0.0))), 4)
    p['facing'] = p.get('facing') if p.get('facing') in FACINGS else 'back'
    p['gap'] = round(float(p.get('gap', DEFAULT_GAP)), 3)
    if p.get('line'):
        p['line'] = [[round(float(v), 3) for v in q] for q in p['line']]
        n = np.asarray(p.get('normal', (0.0, 1.0)), dtype=float)
        p['normal'] = [round(float(v), 4) for v in n / max(float(np.hypot(*n)), 1e-9)]
    else:
        p.pop('line', None)
        p.pop('normal', None)
    return p


def normalize_place(p):
    p = dict(p)
    p['name'] = str(p.get('name', 'place'))
    p['x'], p['y'] = round(float(p['x']), 3), round(float(p['y']), 3)
    p['yaw'] = round(wrap(float(p.get('yaw', 0.0))), 4)
    p['radius'] = round(max(float(p.get('radius', DEFAULT_RADIUS)), 0.05), 3)
    return p


def normalize_step(s):
    s = dict(s)
    if s.get('type') not in STEP_TYPES:
        raise ValueError(f'unknown mission step type: {s.get("type")}')
    _, target_kind, defaults = STEP_TYPES[s['type']]
    out = {'type': s['type']}
    if target_kind is not None:
        out['target'] = str(s.get('target') or '')
    for k, v in defaults.items():
        out[k] = round(float(s.get(k, v)), 3)
    return out


def load_mission(path):
    """-> 문서 dict {'parking': [...], 'places': [...], 'mission': [...]}
    (파일에 있던 다른 키도 그대로 유지)"""
    doc = {}
    if os.path.exists(path):
        with open(path) as f:
            doc = json.load(f)
    doc['parking'] = [normalize_spot(p) for p in doc.get('parking', [])]
    doc['places'] = [normalize_place(p) for p in doc.get('places', [])]
    doc['mission'] = [normalize_step(s) for s in doc.get('mission', [])]
    return doc


def save_mission(path, doc):
    doc = copy.deepcopy(doc)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(doc, f, indent=2)
    os.replace(tmp, path)


def find_spot(doc, name):
    return next((p for p in doc.get('parking', []) if p['name'] == name), None)


def find_place(doc, name):
    return next((p for p in doc.get('places', []) if p['name'] == name), None)
