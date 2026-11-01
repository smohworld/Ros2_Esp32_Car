import math
import struct

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int64MultiArray, UInt8MultiArray, Float32, Int32
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

# 라이다 파라미터
SCAN_BUFFER_SIZE = 720
SEGMENTS = 4
SEGMENT_SIZE = SCAN_BUFFER_SIZE // SEGMENTS
SENSOR_HZ = 6  # Hz

WHEEL_JOINTS = [
    'front_left_wheel_joint',
    'rear_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_right_wheel_joint',
]


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class RobotDataPublisher(Node):
    """ESP32(micro-ROS) 원시 데이터 -> odom / tf / joint_states / scan / battery / rssi."""

    def __init__(self):
        super().__init__('robot_data_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ticks_per_rev', 7 * 298 * 4),   # 엔코더 해상도
                ('wheel_radius', 0.065 / 2),      # 바퀴 반지름 (m)
                ('encoder_idle_threshold', 3.0),  # 정지 판정 임계값 (자이로 드리프트 제거)
                ('gyro_lsb_per_dps', 131.0),      # MPU6050 ±250 deg/s
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
                ('laser_frame', 'laser_frame'),
                ('publish_tf', True),
            ])
        self.TICKS_PER_REV = self.get_parameter('ticks_per_rev').value
        self.WHEEL_RADIUS = self.get_parameter('wheel_radius').value
        self.IDLE_THRESHOLD = self.get_parameter('encoder_idle_threshold').value
        self.GYRO_SCALE = self.get_parameter('gyro_lsb_per_dps').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # micro-ROS 구독
        # 엔코더/라이다를 콜백 그룹으로 나눠서 MultiThreadedExecutor에서 서로 기다리지 않게 함
        # (두 콜백이 쓰는 상태 변수는 겹치지 않으므로 락 불필요)
        self.encoder_group = MutuallyExclusiveCallbackGroup()
        self.lidar_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Int64MultiArray, 'encoder_deltas_gyroz_bat_rssi', self.encoder_callback, qos,
            callback_group=self.encoder_group)
        self.create_subscription(
            UInt8MultiArray, 'range_segment', self.segment_callback, qos,
            callback_group=self.lidar_group)

        # 발행
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.bat_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.rssi_pub = self.create_publisher(Int32, 'rssi_strength', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 오도메트리 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_sec = None
        self.prev_nanosec = None
        self.wheel_pos = [0.0] * 4  # FL, RL, FR, RR 누적 회전각 (rad)

        # 라이다 상태
        self.ranges = np.zeros(SCAN_BUFFER_SIZE, dtype=np.float32)
        self.received = [False] * SEGMENTS
        self.scan = self.make_scan_template(self.get_parameter('laser_frame').value)

        self.get_logger().info('robot_data_publisher started')

    # ------------------------------------------------------------------
    # 엔코더 / 자이로 / 배터리 / RSSI
    # ------------------------------------------------------------------
    def encoder_callback(self, msg):
        if len(msg.data) != 9:
            self.get_logger().warn(
                'expected 9 values (sec, nanosec, FL, RL, FR, RR, GZ, BAT, RSSI)')
            return

        sec, nanosec, fl, rl, fr, rr, gz, batv, rssi = msg.data
        ticks = [fl, rl, fr, rr]

        self.bat_pub.publish(Float32(data=float(batv) / 1000.0))
        self.rssi_pub.publish(Int32(data=int(rssi)))

        # 바퀴가 멈춰 있으면 자이로 값 무시
        encoder_activity = math.sqrt(sum(t * t for t in ticks) / 4.0)
        if encoder_activity < self.IDLE_THRESHOLD:
            gz = 0.0

        # dt는 ESP32 타임스탬프 기준
        if self.prev_sec is not None:
            dt = (sec - self.prev_sec) + (nanosec - self.prev_nanosec) / 1e9
        else:
            dt = 0.0
        self.prev_sec = sec
        self.prev_nanosec = nanosec

        if dt <= 0:
            self.get_logger().warn('Invalid dt or first callback, skipping update')
            return

        stamp = self.get_clock().now().to_msg()

        # tick -> 바퀴 이동 거리(m)
        wheel_circumference = 2 * math.pi * self.WHEEL_RADIUS
        fl_d, rl_d, fr_d, rr_d = [
            (t / self.TICKS_PER_REV) * wheel_circumference for t in ticks]
        dist = ((fl_d + rl_d) / 2.0 + (fr_d + rr_d) / 2.0) / 2.0

        gyro_rad_s = math.radians(gz / self.GYRO_SCALE)
        delta_yaw = gyro_rad_s * dt

        # 위치 갱신
        self.x += dist * math.cos(self.yaw + delta_yaw / 2.0)
        self.y += dist * math.sin(self.yaw + delta_yaw / 2.0)
        self.yaw += delta_yaw

        # JointState
        for i, t in enumerate(ticks):
            self.wheel_pos[i] += (t / self.TICKS_PER_REV) * 2 * math.pi

        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        joint_msg.name = WHEEL_JOINTS
        joint_msg.position = list(self.wheel_pos)
        self.joint_pub.publish(joint_msg)

        # Odometry
        q = yaw_to_quaternion(self.yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = q
        odom_msg.twist.twist.linear.x = dist / dt
        odom_msg.twist.twist.angular.z = gyro_rad_s
        self.odom_pub.publish(odom_msg)

        # TF odom -> base_link
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf_msg)

    # ------------------------------------------------------------------
    # 라이다
    # ------------------------------------------------------------------
    @staticmethod
    def make_scan_template(frame_id):
        scan = LaserScan()
        scan.header.frame_id = frame_id
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / SCAN_BUFFER_SIZE
        scan.angle_max = scan.angle_min + scan.angle_increment * (SCAN_BUFFER_SIZE - 1)
        scan.scan_time = 1.0 / SENSOR_HZ
        scan.time_increment = scan.scan_time / SCAN_BUFFER_SIZE
        scan.range_min = 0.15
        scan.range_max = 8.0
        return scan

    def segment_callback(self, msg):
        data = msg.data
        expected_len = 4 + 4 + 1 + SEGMENT_SIZE * 2  # sec(4B)+nanosec(4B)+index(1B)+distance
        if len(data) != expected_len:
            self.get_logger().warn(f'Invalid segment length: {len(data)}')
            return

        seg_idx = data[8]
        if seg_idx >= SEGMENTS:
            self.get_logger().warn(f'Invalid segment index: {seg_idx}')
            return

        segment = struct.unpack('<' + 'H' * SEGMENT_SIZE, bytes(data[9:]))
        self.ranges[seg_idx * SEGMENT_SIZE:(seg_idx + 1) * SEGMENT_SIZE] = \
            np.array(segment, dtype=np.float32) / 1000.0
        self.received[seg_idx] = True

        # 모든 segment 수신 완료 시 LaserScan 발행
        if all(self.received):
            self.scan.header.stamp = self.get_clock().now().to_msg()
            self.scan.ranges = self.ranges.tolist()
            self.scan_pub.publish(self.scan)
            self.received = [False] * SEGMENTS
            self.get_logger().debug('Published complete scan')


def main(args=None):
    rclpy.init(args=args)
    node = RobotDataPublisher()
    # 엔코더 그룹 + 라이다 그룹 + 기본 그룹(파라미터 서비스)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
