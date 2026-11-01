#!/usr/bin/env python3
"""
미션 설정/실행 + 로봇 모니터링 GUI (PyQt5)
"""
import copy
import math
import os
import signal
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import (QBrush, QColor, QFont, QImage, QKeySequence, QPainter, QPainterPath,
                         QPen, QPixmap, QPolygonF)
from PyQt5.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox,
                             QFormLayout, QGraphicsItem, QGraphicsScene, QGraphicsView,
                             QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget,
                             QMainWindow, QMessageBox, QPushButton, QRadioButton, QScrollArea,
                             QShortcut, QTabWidget, QVBoxLayout, QWidget)
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

from pros2_robot.localization import Localizer
from pros2_robot.map_utils import (default_mission_file, DEFAULT_GAP, DEFAULT_RADIUS,
                                   DEFAULT_STEPS, FACINGS, find_place, find_spot, load_mission,
                                   MapData, normalize_place, normalize_spot, normalize_step,
                                   save_mission, STEP_TYPES)
from pros2_robot.mission_runner import MissionRunner
from pros2_robot.parking import (HALF_LEN, HALF_WIDTH, line_free_normal, Parker, search_dir,
                                 spot_approach, WALL_SEARCH_RANGE)
from pros2_robot.robot_interface import RobotInterface

GUI_PERIOD_MS = 100
CAMERA_WIDTH = 240                  # 카메라 화면 폭 (mjpeg_camera_publisher 출력 240x320)
CAMERA_STALE = 2.0                  # s, 이보다 오래된 영상은 끊긴 것으로 표시
CAMERAS = (('toy_detection/image', 'YOLO detection'),)
CANVAS_BG = '#3a3a3a'
SPOT_COLOR = QColor(60, 140, 255)
PLACE_COLOR = QColor(255, 96, 192)
MODES = {'view': 'View', 'nav': 'Nav goal',
         'spot': 'Add parking (drag = heading)', 'line': 'Draw wall line',
         'place': 'Add place (drag = heading)',
         'pose': 'Set pose (exact)', 'guess': 'Rough pose + search'}
MODE_COLORS = {'spot': SPOT_COLOR.name(), 'line': '#ff9a30', 'place': PLACE_COLOR.name(),
               'pose': '#20ff60', 'guess': '#a0ffc0', 'nav': '#ffff00'}
MIN_DRAG = 0.05     # m, 이보다 짧은 드래그는 방향 없음
TAB_LOCALIZATION = 3


def cpen(color, width=2.0, style=Qt.SolidLine):
    """확대해도 굵기가 같은 펜"""
    p = QPen(QColor(color), width, style)
    p.setCosmetic(True)
    return p


def sp(x, y):
    """map 좌표 -> scene 좌표 (scene은 y가 아래로 커져서 뒤집음)"""
    return QPointF(x, -y)


class MapView(QGraphicsView):
    poseDragged = pyqtSignal(float, float, float, bool)     # x, y, yaw, 방향 있음
    lineDrawn = pyqtSignal(float, float, float, float)      # x1, y1, x2, y2
    mouseMoved = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setScene(QGraphicsScene(self))
        self.setRenderHint(QPainter.Antialiasing)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QColor(CANVAS_BG))
        self.setMouseTracking(True)
        self.mode = 'view'
        self.map_item = None
        self.overlay = []
        self.rubber = None
        self._press = None
        self._pan = None

    def world(self, pos):
        s = self.mapToScene(pos)
        return s.x(), -s.y()

    def set_map(self, m):
        rgb = m.to_rgb()
        img = QImage(rgb.data, m.w, m.h, 3 * m.w, QImage.Format_RGB888).copy()
        first = self.map_item is None
        if self.map_item is not None:
            self.scene().removeItem(self.map_item)
        self.map_item = self.scene().addPixmap(QPixmap.fromImage(img))
        self.map_item.setScale(m.res)
        self.map_item.setPos(m.ox, -(m.oy + m.h * m.res))
        self.map_item.setZValue(-10)
        x0, y0, x1, y1 = m.extent
        margin = 5.0
        self.scene().setSceneRect(QRectF(x0 - margin, -y1 - margin,
                                         x1 - x0 + 2 * margin, y1 - y0 + 2 * margin))
        if first:
            self.fit()

    def fit(self):
        if self.map_item is not None:
            self.fitInView(self.map_item.sceneBoundingRect(), Qt.KeepAspectRatio)

    # ---------- 오버레이 ----------
    def clear_overlay(self):
        for it in self.overlay:
            self.scene().removeItem(it)
        self.overlay = []

    def _add(self, item, z=1):
        item.setZValue(z)
        self.overlay.append(item)
        return item

    def add_polyline(self, pts, color, width=2.0, style=Qt.SolidLine, z=2):
        if pts is None or len(pts) < 2:
            return None
        pts = np.asarray(pts, dtype=float).tolist()
        path = QPainterPath(sp(*pts[0]))
        for p in pts[1:]:
            path.lineTo(sp(*p))
        return self._add(self.scene().addPath(path, cpen(color, width, style)), z)

    def add_points(self, pts, color, size=0.03, z=3):
        if pts is None or len(pts) == 0:
            return None
        path = QPainterPath()
        h = size / 2.0
        for x, y in np.asarray(pts, dtype=float).tolist():
            path.addRect(x - h, -y - h, size, size)
        return self._add(self.scene().addPath(path, QPen(Qt.NoPen), QBrush(QColor(color))), z)

    def add_arrow(self, pose, length, color, width=2.5, z=4):
        x, y, yaw = pose
        tip = (x + length * math.cos(yaw), y + length * math.sin(yaw))
        path = QPainterPath(sp(x, y))
        path.lineTo(sp(*tip))
        for d in (2.6, -2.6):
            path.moveTo(sp(*tip))
            path.lineTo(sp(tip[0] + 0.3 * length * math.cos(yaw + d),
                           tip[1] + 0.3 * length * math.sin(yaw + d)))
        return self._add(self.scene().addPath(path, cpen(color, width)), z)

    def add_robot(self, pose, color, z=5, fill_alpha=90, style=Qt.SolidLine):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(HALF_LEN, HALF_WIDTH), (HALF_LEN, -HALF_WIDTH),
                   (-HALF_LEN, -HALF_WIDTH), (-HALF_LEN, HALF_WIDTH)]
        poly = QPolygonF([sp(x + c * a - s * b, y + s * a + c * b) for a, b in corners])
        fill = QColor(color)
        fill.setAlpha(fill_alpha)
        self._add(self.scene().addPolygon(poly, cpen(color, 2.0, style), QBrush(fill)), z)
        self.add_arrow(pose, HALF_LEN + 0.12, color, 2.5, z)

    def add_circle(self, x, y, r, color, z=2, fill_alpha=30, style=Qt.DashLine):
        fill = QColor(color)
        fill.setAlpha(fill_alpha)
        return self._add(self.scene().addEllipse(QRectF(x - r, -y - r, 2 * r, 2 * r),
                                                 cpen(color, 1.5, style), QBrush(fill)), z)

    def add_text(self, x, y, text, color, z=6):
        it = self.scene().addSimpleText(text)
        it.setBrush(QBrush(QColor(color)))
        it.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        it.setPos(sp(x, y))
        return self._add(it, z)

    # ---------- 마우스 ----------
    def wheelEvent(self, e):
        f = 1.25 ** (e.angleDelta().y() / 120.0)
        self.scale(f, f)

    def mousePressEvent(self, e):
        if e.button() in (Qt.RightButton, Qt.MiddleButton):
            self._pan = e.pos()
            self.setCursor(Qt.ClosedHandCursor)
            return
        if e.button() == Qt.LeftButton and self.mode in MODE_COLORS:
            self._press = self.world(e.pos())
            self.rubber = self.scene().addLine(0, 0, 0, 0, cpen(MODE_COLORS[self.mode], 2))
            self.rubber.setZValue(10)
            return
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        x, y = self.world(e.pos())
        self.mouseMoved.emit(x, y)
        if self._pan is not None:
            d = e.pos() - self._pan
            self._pan = e.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - d.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - d.y())
            return
        if self._press is not None and self.rubber is not None:
            px, py = self._press
            self.rubber.setLine(px, -py, x, -y)

    def mouseReleaseEvent(self, e):
        if self._pan is not None and e.button() in (Qt.RightButton, Qt.MiddleButton):
            self._pan = None
            self.setCursor(Qt.ArrowCursor)
            return
        if e.button() != Qt.LeftButton or self._press is None:
            return super().mouseReleaseEvent(e)
        px, py = self._press
        x, y = self.world(e.pos())
        self._press = None
        if self.rubber is not None:
            self.scene().removeItem(self.rubber)
            self.rubber = None
        moved = math.hypot(x - px, y - py) >= MIN_DRAG
        if self.mode == 'line':
            if moved:
                self.lineDrawn.emit(px, py, x, y)
            return
        self.poseDragged.emit(px, py, math.atan2(y - py, x - px) if moved else 0.0, moved)


def image_to_qimage(msg):
    """sensor_msgs/Image (bgr8, rgb8, mono8) -> QImage, 모르는 인코딩이면 None"""
    fmt = {'bgr8': QImage.Format_BGR888, 'rgb8': QImage.Format_RGB888,
           'mono8': QImage.Format_Grayscale8}.get(msg.encoding)
    if fmt is None:
        return None
    data = bytes(msg.data)
    return QImage(data, msg.width, msg.height, msg.step, fmt).copy()


class CameraView(QGroupBox):
    """이미지 토픽 하나를 표시 (구독 콜백은 저장만, 변환은 GUI 타이머에서)"""

    def __init__(self, node, topic, title):
        super().__init__(f'{title}  (/{topic})')
        self.lock = threading.Lock()
        self.msg = None
        self.stamp = 0.0
        self.shown = None
        self.label = QLabel('no image')
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFixedSize(CAMERA_WIDTH, CAMERA_WIDTH * 4 // 3)
        self.label.setStyleSheet(f'background: {CANVAS_BG}; color: #c0c0c0')
        QVBoxLayout(self).addWidget(self.label)
        node.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        with self.lock:
            self.msg = msg
            self.stamp = time.time()

    def refresh(self):
        with self.lock:
            msg, stamp = self.msg, self.stamp
        if msg is None or time.time() - stamp > CAMERA_STALE:
            if self.shown is not None or msg is not None:
                self.label.setText('no image' if msg is None else 'no image (stale)')
            self.shown = None
            return
        if msg is self.shown:
            return
        self.shown = msg
        img = image_to_qimage(msg)
        if img is None:
            self.label.setText(f'unsupported encoding {msg.encoding}')
            return
        self.label.setPixmap(QPixmap.fromImage(img).scaled(
            self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


class MainWindow(QMainWindow):
    def __init__(self, node, map_yaml, mission_file):
        super().__init__()
        self.node = node
        self.parker = Parker(node)
        self.localizer = Localizer(node)
        self.runner = MissionRunner(node, self.parker, self._store_result)
        self.map_yaml = map_yaml
        self.mission_file = mission_file
        try:
            self.doc = load_mission(mission_file)
        except (OSError, ValueError, KeyError) as e:
            QMessageBox.warning(self, 'Mission file',
                                f'Cannot read {mission_file}:\n{e}\nStarting empty.')
            self.doc = {'parking': [], 'places': [], 'mission': []}
        self.doc['map'] = map_yaml
        self.selected = None            # 선택한 parking 이름
        self.selected_place = None
        self.running_step = None        # 실행 중인 미션 단계 (목록 표시용)
        self._filling = False           # 단계 입력 칸을 코드로 채우는 중 (편집 이벤트 무시)
        self.map_version = -1
        self.result_lock = threading.Lock()
        self.pending_results = []       # 미션 스레드가 넣고 GUI가 저장 (name, result)

        self.setWindowTitle(f'pros2 mission - {os.path.basename(map_yaml)}')
        self.view = MapView()
        self.view.poseDragged.connect(self.on_pose_drag)
        self.view.lineDrawn.connect(self.on_line)
        self.view.mouseMoved.connect(
            lambda x, y: self.statusBar().showMessage(f'x={x:.2f}  y={y:.2f}'))

        central = QWidget()
        lay = QHBoxLayout(central)
        lay.addWidget(self._build_panel())
        lay.addWidget(self.view, 1)
        cams = QVBoxLayout()
        self.cameras = [CameraView(node, topic, title) for topic, title in CAMERAS]
        for c in self.cameras:
            cams.addWidget(c)
        cams.addStretch(1)
        lay.addLayout(cams)
        self.setCentralWidget(central)
        self.resize(1550, 850)

        QShortcut(QKeySequence('Esc'), self, activated=self.node.stop)
        QShortcut(QKeySequence('F'), self, activated=self.view.fit)
        self.refresh_all()
        self.timer = QTimer(self, timeout=self.tick)
        self.timer.start(GUI_PERIOD_MS)

    # ---------- 패널 ----------
    def _build_panel(self):
        self.mode_group = QButtonGroup(self)
        self.mode_group.buttonClicked.connect(
            lambda b: setattr(self.view, 'mode', b.property('mode')))
        self.mode_buttons = {}

        panel = QWidget()
        v = QVBoxLayout(panel)
        v.setContentsMargins(0, 0, 0, 0)
        self.tabs = QTabWidget()
        for build, name in ((self._build_mission_tab, 'Mission'),
                            (self._build_parking_tab, 'Parking'),
                            (self._build_places_tab, 'Places'),
                            (self._build_localization_tab, 'Localization')):
            self.tabs.addTab(self._scroll(build()), name)
        self.tabs.currentChanged.connect(lambda _: self.set_mode('view'))
        v.addWidget(self.tabs, 1)
        v.addWidget(self._build_common())
        panel.setFixedWidth(380)
        self.set_mode('view')
        return panel

    @staticmethod
    def _scroll(w):
        # 창이 낮아도 입력 칸이 찌그러지지 않게 스크롤
        scroll = QScrollArea()
        scroll.setWidget(w)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        return scroll

    def _mode_button(self, key):
        rb = QRadioButton(MODES[key])
        rb.setProperty('mode', key)
        if key in MODE_COLORS:
            rb.setStyleSheet(f'color: {MODE_COLORS[key]}; font-weight: bold')
        self.mode_group.addButton(rb)
        self.mode_buttons[key] = rb
        return rb

    def set_mode(self, key):
        self.mode_buttons[key].setChecked(True)
        self.view.mode = key

    @staticmethod
    def _buttons(layout, items):
        row = QHBoxLayout()
        for text, fn in items:
            b = QPushButton(text)
            b.clicked.connect(fn)
            row.addWidget(b)
        layout.addLayout(row)

    @staticmethod
    def _hint(text):
        lb = QLabel(text)
        lb.setWordWrap(True)
        lb.setStyleSheet('color: gray')
        return lb

    def _build_common(self):
        box = QWidget()
        v = QVBoxLayout(box)
        v.setContentsMargins(0, 0, 0, 0)
        row = QHBoxLayout()
        row.addWidget(self._mode_button('view'))
        row.addWidget(self._mode_button('nav'))
        v.addLayout(row)
        b = QPushButton('STOP (Esc)')
        b.setStyleSheet('background: #c0392b; color: white; font-weight: bold; padding: 6px')
        b.clicked.connect(self.node.stop)
        v.addWidget(b)
        row = QHBoxLayout()
        self.c_scan = QCheckBox('Scan', checked=True)
        self.c_path = QCheckBox('Path', checked=True)
        self.c_trail = QCheckBox('Trail', checked=True)
        for c in (self.c_scan, self.c_path, self.c_trail):
            row.addWidget(c)
        v.addLayout(row)
        self._buttons(v, (('Fit view (F)', self.view.fit), ('Clear trail', self.node.clear_trail)))
        self.l_pose = QLabel('Robot: -')
        self.l_status = QLabel('')
        self.l_status.setWordWrap(True)
        self.l_status.setMinimumHeight(40)
        v.addWidget(self.l_pose)
        v.addWidget(self.l_status)
        v.addWidget(self._hint(f'Mission file: {self.mission_file}'))
        return box

    # ---------- Mission 탭 ----------
    def _build_mission_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.addWidget(self._hint('Steps run in order. Targets are set in the Parking / Places '
                               'tabs. Follow toy: toy_tracker runs until the robot is inside '
                               'the place radius.'))
        self.step_list = QListWidget()
        self.step_list.setMinimumHeight(160)
        self.step_list.currentRowChanged.connect(self.on_step_select)
        v.addWidget(self.step_list)
        form = QFormLayout()
        self.s_type = QComboBox()
        for key, (label, _, _) in STEP_TYPES.items():
            self.s_type.addItem(label, key)
        self.s_target = QComboBox()
        self.s_turns = QDoubleSpinBox()
        self.s_turns.setRange(0.5, 5.0)
        self.s_turns.setSingleStep(0.5)
        self.s_turns.setDecimals(1)
        self.s_turns.setSuffix(' turns')
        self.s_timeout = QDoubleSpinBox()
        self.s_timeout.setRange(10.0, 1800.0)
        self.s_timeout.setSingleStep(30.0)
        self.s_timeout.setDecimals(0)
        self.s_timeout.setSuffix(' s')
        self.s_wait = QDoubleSpinBox()
        self.s_wait.setRange(0.0, 600.0)
        self.s_wait.setSingleStep(1.0)
        self.s_wait.setDecimals(1)
        self.s_wait.setSuffix(' s')
        form.addRow('Type', self.s_type)
        form.addRow('Target', self.s_target)
        form.addRow('Max turns', self.s_turns)
        form.addRow('Timeout', self.s_timeout)
        form.addRow('Wait after', self.s_wait)
        v.addLayout(form)
        self.s_type.currentIndexChanged.connect(self.on_step_edit)
        self.s_target.currentIndexChanged.connect(self.on_step_edit)
        self.s_turns.valueChanged.connect(self.on_step_edit)
        self.s_timeout.valueChanged.connect(self.on_step_edit)
        self.s_wait.valueChanged.connect(self.on_step_edit)
        self._buttons(v, (('Add', self.on_step_add), ('Delete', self.on_step_delete),
                          ('Up', lambda: self.on_step_move(-1)),
                          ('Down', lambda: self.on_step_move(1))))
        self._buttons(v, (('Default scenario', self.on_default_scenario),))

        box = QGroupBox('Run')
        bl = QVBoxLayout(box)
        self._buttons(bl, (('Run all', lambda: self.on_run(0)),
                           ('From selected', lambda: self.on_run(None)),
                           ('Selected only', lambda: self.on_run(None, True))))
        v.addWidget(box)
        v.addStretch(1)
        return w

    # ---------- Parking 탭 ----------
    def _build_parking_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox('Click mode')
        bl = QVBoxLayout(box)
        bl.addWidget(self._mode_button('spot'))
        bl.addWidget(self._mode_button('line'))
        v.addWidget(box)

        box = QGroupBox('Parking spots')
        bl = QVBoxLayout(box)
        self.spot_list = QListWidget()
        self.spot_list.setMaximumHeight(110)
        self.spot_list.currentRowChanged.connect(self.on_select)
        bl.addWidget(self.spot_list)
        form = QFormLayout()
        self.f_name = QLineEdit()
        self.f_facing = QComboBox()
        self.f_facing.addItems(FACINGS)
        self.f_gap = QDoubleSpinBox()
        self.f_gap.setRange(0.0, 1.0)
        self.f_gap.setSingleStep(0.01)
        self.f_gap.setDecimals(3)
        self.f_gap.setSuffix(' m')
        self.l_spot = QLabel('')
        form.addRow('Name', self.f_name)
        form.addRow('Facing', self.f_facing)
        form.addRow('Gap', self.f_gap)
        form.addRow('Pose', self.l_spot)
        bl.addLayout(form)
        self._buttons(bl, (('Apply', self.on_apply), ('Move here', self.on_move_here),
                           ('Delete', self.on_delete)))
        v.addWidget(box)

        box = QGroupBox('Actions')
        bl = QVBoxLayout(box)
        self._buttons(bl, (('Go to spot', self.on_go), ('Detect wall', self.on_detect),
                           ('Park', self.on_park)))
        v.addWidget(box)
        v.addStretch(1)
        return w

    # ---------- Places 탭 ----------
    def _build_places_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.addWidget(self._hint('Nav2 goals and follow-toy arrival areas '
                               '(the robot is there when inside the radius).'))
        box = QGroupBox('Click mode')
        bl = QVBoxLayout(box)
        bl.addWidget(self._mode_button('place'))
        v.addWidget(box)

        box = QGroupBox('Places')
        bl = QVBoxLayout(box)
        self.place_list = QListWidget()
        self.place_list.setMaximumHeight(130)
        self.place_list.currentRowChanged.connect(self.on_place_select)
        bl.addWidget(self.place_list)
        form = QFormLayout()
        self.p_name = QLineEdit()
        self.p_radius = QDoubleSpinBox()
        self.p_radius.setRange(0.05, 5.0)
        self.p_radius.setSingleStep(0.05)
        self.p_radius.setDecimals(2)
        self.p_radius.setSuffix(' m')
        self.l_place = QLabel('')
        form.addRow('Name', self.p_name)
        form.addRow('Radius', self.p_radius)
        form.addRow('Pose', self.l_place)
        bl.addLayout(form)
        self._buttons(bl, (('Apply', self.on_place_apply), ('Move here', self.on_place_move_here),
                           ('Delete', self.on_place_delete)))
        v.addWidget(box)
        self._buttons(v, (('Go to place', self.on_place_go),))
        v.addStretch(1)
        return w

    # ---------- Localization 탭 ----------
    def _build_localization_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        box = QGroupBox('Find the robot on the map')
        bl = QVBoxLayout(box)
        bl.addWidget(self._hint('Spreads AMCL particles over the whole map, then turns in '
                                'place until AMCL converges and the lidar matches the map walls. '
                                'Keep the area around the robot clear.'))
        form = QFormLayout()
        self.loc_turns = QDoubleSpinBox()
        self.loc_turns.setRange(1.0, 5.0)
        self.loc_turns.setSingleStep(0.5)
        self.loc_turns.setDecimals(1)
        self.loc_turns.setValue(2.0)
        self.loc_turns.setSuffix(' turns')
        form.addRow('Max turns', self.loc_turns)
        bl.addLayout(form)
        self._buttons(bl, (('Localize (whole map)', lambda: self.on_localize(None)),))
        v.addWidget(box)

        box = QGroupBox('Click mode')
        bl = QVBoxLayout(box)
        bl.addWidget(self._mode_button('guess'))
        bl.addWidget(self._hint('Drag roughly where the robot is -> search only near there'))
        bl.addWidget(self._mode_button('pose'))
        bl.addWidget(self._hint('Drag the exact pose -> /initialpose, no motion'))
        v.addWidget(box)

        box = QGroupBox('AMCL')
        bl = QVBoxLayout(box)
        self.l_loc = QLabel('-')
        self.l_loc.setWordWrap(True)
        bl.addWidget(self.l_loc)
        v.addWidget(box)
        v.addStretch(1)
        return w

    # ---------- 공통 ----------
    def save(self):
        try:
            save_mission(self.mission_file, self.doc)
        except OSError as e:
            QMessageBox.critical(self, 'Save', f'Cannot save {self.mission_file}:\n{e}')

    def refresh_all(self):
        self.refresh_list()
        self.refresh_places()
        self.refresh_steps()

    def name_taken(self, name, me=None):
        """parking과 place 이름은 서로 겹치면 안 됨 (미션 target이 이름으로 찾음)"""
        other = self.spot(name) or self.place(name)
        return other is not None and other is not me

    def free_name(self, prefix):
        n = 1
        while self.name_taken(f'{prefix}{n}'):
            n += 1
        return f'{prefix}{n}'

    def rename_targets(self, old, new):
        for s in self.steps:
            if s.get('target') == old:
                s['target'] = new

    # ---------- 주차 지점 ----------
    @property
    def spots(self):
        return self.doc['parking']

    def spot(self, name):
        return find_spot(self.doc, name)

    def refresh_list(self):
        self.spot_list.blockSignals(True)
        self.spot_list.clear()
        row = -1
        for i, p in enumerate(self.spots):
            self.spot_list.addItem(f'{p["name"]}  ({"line, " if p.get("line") else ""}'
                                   f'{p["facing"]}, gap {p["gap"]:.2f})'
                                   + ('  *' if p.get('result') else ''))
            if p['name'] == self.selected:
                row = i
        self.spot_list.setCurrentRow(row)
        self.spot_list.blockSignals(False)
        self.on_select(row)

    def on_select(self, row):
        p = self.spots[row] if 0 <= row < len(self.spots) else None
        self.selected = p['name'] if p else None
        if p is None:
            self.f_name.clear()
            self.l_spot.setText('')
            return
        self.f_name.setText(p['name'])
        self.f_facing.setCurrentText(p['facing'])
        self.f_gap.setValue(p['gap'])
        self.l_spot.setText(f'x={p["x"]:.2f} y={p["y"]:.2f} yaw={math.degrees(p["yaw"]):.0f}')

    def add_spot(self, x, y, yaw, **extra):
        p = normalize_spot({'name': self.free_name('parking'), 'x': x, 'y': y, 'yaw': yaw,
                            'facing': 'back', 'gap': DEFAULT_GAP, **extra})
        if p.get('line'):
            p.update(zip(('x', 'y', 'yaw'), spot_approach(p)))
            p = normalize_spot(p)
        self.spots.append(p)
        self.selected = p['name']
        self.save()
        self.refresh_list()
        self.refresh_steps()

    def on_line(self, x1, y1, x2, y2):
        with self.node.lock:
            m = self.node.map
        n = line_free_normal((x1, y1), (x2, y2), m)
        self.add_spot(0.0, 0.0, 0.0, line=[[x1, y1], [x2, y2]], normal=list(n))

    def on_apply(self):
        p = self.spot(self.selected)
        if p is None:
            return
        name = self.f_name.text().strip()
        if not name:
            return
        if self.name_taken(name, p):
            QMessageBox.warning(self, 'Parking', f'"{name}" already exists')
            return
        self.rename_targets(p['name'], name)
        p.update(name=name, facing=self.f_facing.currentText(), gap=self.f_gap.value())
        if p.get('line'):
            p.update(zip(('x', 'y', 'yaw'), spot_approach(p)))     # 접근 자세 다시 계산
        self.spots[self.spots.index(p)] = normalize_spot(p)
        self.selected = name
        self.save()
        self.refresh_list()
        self.refresh_steps()

    def on_move_here(self):
        p = self.spot(self.selected)
        robot = self.node.robot_pose()
        if p is None or robot is None:
            self.node.set_status('Move here: select a spot (and the robot pose must be known)')
            return
        if p.get('line'):
            self.node.set_status('Move here: line spot, draw a new wall line instead')
            return
        p.update(x=robot[0], y=robot[1], yaw=robot[2])
        p.pop('result', None)
        self.spots[self.spots.index(p)] = normalize_spot(p)
        self.save()
        self.refresh_list()

    def on_delete(self):
        p = self.spot(self.selected)
        if p is None:
            return
        self.spots.remove(p)
        self.selected = None
        self.save()
        self.refresh_list()
        self.refresh_steps()

    def selected_spot(self):
        p = self.spot(self.selected)
        if p is None:
            self.node.set_status('Select a parking spot first (or add one: Add parking + drag)')
        return dict(p) if p else None

    # ---------- 이동 위치 ----------
    @property
    def places(self):
        return self.doc['places']

    def place(self, name):
        return find_place(self.doc, name)

    def refresh_places(self):
        self.place_list.blockSignals(True)
        self.place_list.clear()
        row = -1
        for i, p in enumerate(self.places):
            self.place_list.addItem(f'{p["name"]}  (r {p["radius"]:.2f} m)')
            if p['name'] == self.selected_place:
                row = i
        self.place_list.setCurrentRow(row)
        self.place_list.blockSignals(False)
        self.on_place_select(row)

    def on_place_select(self, row):
        p = self.places[row] if 0 <= row < len(self.places) else None
        self.selected_place = p['name'] if p else None
        if p is None:
            self.p_name.clear()
            self.l_place.setText('')
            return
        self.p_name.setText(p['name'])
        self.p_radius.setValue(p['radius'])
        self.l_place.setText(f'x={p["x"]:.2f} y={p["y"]:.2f} yaw={math.degrees(p["yaw"]):.0f}')

    def add_place(self, x, y, yaw):
        p = normalize_place({'name': self.free_name('place'), 'x': x, 'y': y, 'yaw': yaw,
                             'radius': DEFAULT_RADIUS})
        self.places.append(p)
        self.selected_place = p['name']
        self.save()
        self.refresh_places()
        self.refresh_steps()

    def on_place_apply(self):
        p = self.place(self.selected_place)
        name = self.p_name.text().strip()
        if p is None or not name:
            return
        if self.name_taken(name, p):
            QMessageBox.warning(self, 'Places', f'"{name}" already exists')
            return
        self.rename_targets(p['name'], name)
        p.update(name=name, radius=self.p_radius.value())
        self.places[self.places.index(p)] = normalize_place(p)
        self.selected_place = name
        self.save()
        self.refresh_places()
        self.refresh_steps()

    def on_place_move_here(self):
        p = self.place(self.selected_place)
        robot = self.node.robot_pose()
        if p is None or robot is None:
            self.node.set_status('Move here: select a place (and the robot pose must be known)')
            return
        p.update(x=robot[0], y=robot[1], yaw=robot[2])
        self.places[self.places.index(p)] = normalize_place(p)
        self.save()
        self.refresh_places()

    def on_place_delete(self):
        p = self.place(self.selected_place)
        if p is None:
            return
        self.places.remove(p)
        self.selected_place = None
        self.save()
        self.refresh_places()
        self.refresh_steps()

    def on_place_go(self):
        p = self.place(self.selected_place)
        if p is None:
            self.node.set_status('Select a place first (or add one: Add place + drag)')
            return
        self.node.start_mission(f'Go to {p["name"]}', self.node.go_to,
                                (p['x'], p['y'], p['yaw']), f'Go to {p["name"]}')

    # ---------- 미션 단계 ----------
    @property
    def steps(self):
        return self.doc['mission']

    def target_options(self, kind):
        opts = []
        if kind in ('any', 'place'):
            opts += [(p['name'], f'{p["name"]} (place)') for p in self.places]
        if kind in ('any', 'parking'):
            opts += [(p['name'], f'{p["name"]} (parking)') for p in self.spots]
        return opts

    def step_text(self, i, s):
        label, kind, _ = STEP_TYPES[s['type']]
        text = f'{i + 1}. {label}'
        if kind is not None:
            name = s.get('target')
            missing = name and self.spot(name) is None and self.place(name) is None
            text += f'  ->  {name or "?"}' + ('  (missing!)' if missing else '')
        if 'turns' in s:
            text += f'  ({s["turns"]:g} turns)'
        if 'timeout' in s:
            text += f'  ({s["timeout"]:.0f} s)'
        if 'wait' in s:
            text += f'  (wait {s["wait"]:g} s)'
        return text

    def refresh_steps(self, row=None):
        if row is None:
            row = self.step_list.currentRow()
        self.step_list.blockSignals(True)
        self.step_list.clear()
        for i, s in enumerate(self.steps):
            self.step_list.addItem(self.step_text(i, s))
        row = min(row, len(self.steps) - 1)
        self.step_list.setCurrentRow(row)
        self.step_list.blockSignals(False)
        self.mark_running_step()
        self.on_step_select(row)

    def mark_running_step(self):
        bold = QFont()
        bold.setBold(True)
        for i in range(self.step_list.count()):
            it = self.step_list.item(i)
            running = i == self.running_step
            it.setFont(bold if running else QFont())
            it.setData(Qt.ForegroundRole, QBrush(QColor('#20a050')) if running else None)
            it.setText(('> ' if running else '') + self.step_text(i, self.steps[i]))

    def on_step_select(self, row):
        s = self.steps[row] if 0 <= row < len(self.steps) else None
        self._filling = True
        try:
            for w in (self.s_type, self.s_target, self.s_turns, self.s_timeout, self.s_wait):
                w.setEnabled(s is not None)
            if s is None:
                self.s_target.clear()
                return
            _, kind, _ = STEP_TYPES[s['type']]
            self.s_type.setCurrentIndex(self.s_type.findData(s['type']))
            self.s_target.clear()
            self.s_target.setEnabled(kind is not None)
            if kind is not None:
                self.s_target.addItem('(select)', '')
                for name, label in self.target_options(kind):
                    self.s_target.addItem(label, name)
                target = s.get('target', '')
                if target and self.s_target.findData(target) < 0:
                    self.s_target.addItem(f'{target} (missing)', target)
                self.s_target.setCurrentIndex(max(self.s_target.findData(target), 0))
            self.s_turns.setEnabled('turns' in s)
            self.s_turns.setValue(s.get('turns', 2.0))
            self.s_timeout.setEnabled('timeout' in s)
            self.s_timeout.setValue(s.get('timeout', 300.0))
            self.s_wait.setEnabled('wait' in s)
            self.s_wait.setValue(s.get('wait', 3.0))
        finally:
            self._filling = False

    def on_step_edit(self):
        row = self.step_list.currentRow()
        if self._filling or not 0 <= row < len(self.steps):
            return
        old = self.steps[row]
        t = self.s_type.currentData()
        kind = STEP_TYPES[t][1]
        target = ((self.s_target.currentData() or '') if t == old['type']
                  else old.get('target', ''))
        if kind is not None and target not in dict(self.target_options(kind)):
            target = ''     # 종류가 바뀌어서 맞지 않는 대상
        self.steps[row] = normalize_step({**old, 'type': t, 'target': target,
                                          'turns': self.s_turns.value(),
                                          'timeout': self.s_timeout.value(),
                                          'wait': self.s_wait.value()}
                                         if t == old['type'] else {'type': t, 'target': target})
        self.save()
        self.refresh_steps(row)

    def on_step_add(self):
        row = self.step_list.currentRow()
        row = len(self.steps) if row < 0 else row + 1
        self.steps.insert(row, normalize_step({'type': 'nav'}))
        self.save()
        self.refresh_steps(row)

    def on_step_delete(self):
        row = self.step_list.currentRow()
        if 0 <= row < len(self.steps):
            del self.steps[row]
            self.save()
            self.refresh_steps(row)

    def on_step_move(self, d):
        row = self.step_list.currentRow()
        new = row + d
        if 0 <= row < len(self.steps) and 0 <= new < len(self.steps):
            self.steps[row], self.steps[new] = self.steps[new], self.steps[row]
            self.save()
            self.refresh_steps(new)

    def on_default_scenario(self):
        if self.steps and QMessageBox.question(
                self, 'Mission', 'Replace the current steps with the default scenario?') \
                != QMessageBox.Yes:
            return
        park = self.spots[0]['name'] if self.spots else ''
        places = [p['name'] for p in self.places]
        targets = {0: park, 1: park, 2: places[0] if places else '',
                   4: places[1] if len(places) > 1 else (places[0] if places else ''), 5: park}
        self.doc['mission'] = [normalize_step({'type': t, 'target': targets.get(i, '')})
                               for i, t in enumerate(DEFAULT_STEPS)]
        self.save()
        self.refresh_steps(0)

    def on_run(self, start, only=False):
        if start is None:
            start = self.step_list.currentRow()
            if start < 0:
                self.node.set_status('Select a mission step first')
                return
        # 실행 중에 편집해도 영향 없게 복사본으로 실행
        doc = copy.deepcopy(self.doc)
        name = f'Mission step {start + 1}' if only else 'Mission'
        self.node.start_mission(name, self.runner.run, doc, start, only)

    # ---------- 위치 찾기 ----------
    def on_localize(self, guess):
        self.node.start_mission('Localization', self.localizer.localize,
                                self.loc_turns.value(), guess)

    # ---------- 동작 ----------
    def on_pose_drag(self, x, y, yaw, moved):
        mode = self.view.mode
        robot = self.node.robot_pose()
        if not moved:
            # 방향을 안 주면: 초기 위치는 현재 방향 유지, Nav 목표는 로봇에서 그 점을 향하는 방향
            if mode == 'nav' and robot:
                yaw = math.atan2(y - robot[1], x - robot[0])
            elif robot:
                yaw = robot[2]
        if mode == 'spot':
            self.add_spot(x, y, yaw)
        elif mode == 'place':
            self.add_place(x, y, yaw)
        elif mode == 'pose':
            self.node.publish_initial_pose((x, y, yaw))
        elif mode == 'guess':
            self.on_localize((x, y, yaw))       # 방향은 모르는 것으로 (360도 탐색)
        elif mode == 'nav':
            self.node.start_mission('Nav goal', self.node.go_to, (x, y, yaw))

    def _store_result(self, name, result):
        """미션 스레드: 결과를 GUI 쪽에 넘겨서 미션 파일에 저장"""
        with self.result_lock:
            self.pending_results.append((name, result))

    def _run_and_store(self, fn, spot, *args):
        self._store_result(spot['name'], fn(spot, *args))

    def on_go(self):
        p = self.selected_spot()
        if p is not None:
            self.node.start_mission(f'Go to {p["name"]}', self.parker.go_to_spot, p)

    def on_detect(self):
        p = self.selected_spot()
        if p is not None:
            self.node.start_mission(f'Detect wall ({p["name"]})', self._run_and_store,
                                    self.parker.detect, p)

    def on_park(self):
        p = self.selected_spot()
        if p is not None:
            self.node.start_mission(f'Park {p["name"]}', self._run_and_store,
                                    self.parker.park, p)

    # ---------- 화면 갱신 ----------
    def _store_pending(self):
        with self.result_lock:
            pending, self.pending_results = self.pending_results, []
        for name, result in pending:
            p = self.spot(name)
            if p is not None:
                p['result'] = result
        if pending:
            self.save()
            self.refresh_list()

    def tick(self):
        self._store_pending()
        for c in self.cameras:
            c.refresh()
        node = self.node
        with node.lock:
            m, ver = node.map, node.map_version
            plan = node.plan
            local_plan = node.local_plan
            trail = list(node.trail)
            goal = node.nav_goal
            viz = dict(node.viz)
            status = node.status
            mission = node.mission_name
            scan = node.scan
            step = node.mission_step
        if m is not None and ver != self.map_version:
            self.map_version = ver
            self.view.set_map(m)
        if step != self.running_step:
            self.running_step = step
            self.mark_running_step()

        v = self.view
        v.clear_overlay()
        for p in self.places:
            sel = p['name'] == self.selected_place
            col = PLACE_COLOR.lighter(130) if sel else PLACE_COLOR
            v.add_circle(p['x'], p['y'], p['radius'], col, z=2, fill_alpha=50 if sel else 20)
            v.add_arrow((p['x'], p['y'], p['yaw']), 0.25, col, 3.0 if sel else 2.0, z=3)
            v.add_text(p['x'], p['y'], ' ' + p['name'], col)
        for p in self.spots:
            sel = p['name'] == self.selected
            col = SPOT_COLOR.lighter(140) if sel else SPOT_COLOR
            pose = (p['x'], p['y'], p['yaw'])
            v.add_robot(pose, col, z=3, fill_alpha=60 if sel else 25, style=Qt.DashLine)
            v.add_text(p['x'], p['y'], ' ' + p['name'], col)
            if p.get('line'):
                (x1, y1), (x2, y2) = p['line']
                v.add_polyline(p['line'], MODE_COLORS['line'], 5.0 if sel else 3.0, z=3)
                mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                v.add_arrow((mx, my, math.atan2(p['normal'][1], p['normal'][0])), 0.15,
                            MODE_COLORS['line'], 2.0, z=3)
            elif sel:
                # 벽을 찾을 방향 (back이면 뒤쪽)
                d = search_dir(p['yaw'], p['facing'])
                v.add_polyline([(p['x'], p['y']), (p['x'] + d[0] * WALL_SEARCH_RANGE,
                                                   p['y'] + d[1] * WALL_SEARCH_RANGE)],
                               col, 1.5, Qt.DashLine, z=3)
            res = p.get('result')
            if res:
                v.add_points([res['wall_mid']], '#ff6060', 0.05, 3)
                v.add_robot(tuple(res['park_pose']), '#00a0a0', z=2, fill_alpha=30,
                            style=Qt.DotLine)

        if self.c_trail.isChecked() and len(trail) >= 2:
            v.add_polyline(trail, '#b070ff', 1.5, Qt.DotLine)
        if self.c_path.isChecked():
            v.add_polyline(plan, '#00c8ff', 2.0)
            v.add_polyline(local_plan, '#ff8c00', 2.5, z=3)
        if goal is not None:
            v.add_arrow(goal, 0.25, '#ffff00', 3.0)

        for s in viz.get('segments', []):
            v.add_polyline([s['p1'], s['p2']], '#ff40ff', 2.0, z=4)
        wall = viz.get('wall')
        if wall is not None:
            v.add_polyline([wall['p1'], wall['p2']], '#ff2020', 4.0, z=4)
            v.add_points([wall['mid']], '#ff2020', 0.06, 5)
        if viz.get('target') is not None:
            v.add_robot(viz['target'], '#00e0e0', z=4)

        robot = node.robot_pose()
        if self.c_scan.isChecked() and scan is not None:
            pts = node.scan_in_frame(scan, 'map')
            if pts is not None:
                v.add_points(pts, '#ff3030', 0.035)
        if robot is not None:
            v.add_robot(robot, '#20ff60')
            self.l_pose.setText(f'Robot: x={robot[0]:.2f}  y={robot[1]:.2f}  '
                                f'yaw={math.degrees(robot[2]):.0f} deg')
        else:
            self.l_pose.setText('Robot: no map->base_link TF\n(AMCL running? Localization tab)')
        self.l_status.setText((f'[{mission}]\n' if mission else '') + status)
        if self.tabs.currentIndex() == TAB_LOCALIZATION:
            q = self.localizer.quality()
            ok = self.localizer.converged(*q)
            self.l_loc.setText(self.localizer.describe(*q)
                               + ('\nlocalized' if ok else '\nnot localized yet'))
            self.l_loc.setStyleSheet(f'color: {"#20a050" if ok else "#d08000"}')

    def closeEvent(self, e):
        if self.node.busy():
            self.node.stop()
        super().closeEvent(e)


def main(args=None):
    # Ctrl+C는 아래 Qt 쪽 핸들러로 받아서 창을 닫고 정지 명령을 보낸 뒤 종료
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotInterface('mission_gui')
    share = get_package_share_directory('pros2_robot')
    map_yaml = node.declare_parameter('map_yaml', os.path.join(share, 'maps', 'map4.yaml')).value
    mission_file = (node.declare_parameter('mission_file', '').value
                    or default_mission_file(map_yaml))
    try:
        node.set_map(MapData.from_yaml(map_yaml))      # map_server의 /map이 오면 그걸로 교체
    except (OSError, KeyError, ValueError) as e:
        node.get_logger().error(f'Cannot read {map_yaml}: {e}')

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    app = QApplication(sys.argv[:1])
    win = MainWindow(node, map_yaml, mission_file)
    win.show()
    # Ctrl+C로 종료 (Qt 이벤트 루프 중에도 파이썬 시그널 처리가 돌도록 타이머)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    wake = QTimer(timeout=lambda: None)
    wake.start(200)
    try:
        app.exec_()
    finally:
        node.stop()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
