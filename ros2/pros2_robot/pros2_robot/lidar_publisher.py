import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import numpy as np
import math
import struct
from builtin_interfaces.msg import Time

SCAN_BUFFER_SIZE = 720
SEGMENTS = 4
SEGMENT_SIZE = SCAN_BUFFER_SIZE // SEGMENTS
SENSOR_HZ = 6 # Hz

class LidarReceiver(Node):
    def __init__(self):
        super().__init__('lidar_receiver')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.ranges = np.zeros(SCAN_BUFFER_SIZE, dtype=np.float32)
        self.received = [False] * SEGMENTS
        self.sec = 0
        self.nanosec = 0

        self.sub = self.create_subscription(
            UInt8MultiArray,
            'range_segment',
            self.segment_callback,
            qos
        )

        self.pub = self.create_publisher(
            LaserScan,
            'scan',
            10
        )

        self.scan = LaserScan()
        self.initialize_scan()

    def segment_callback(self, msg: UInt8MultiArray):
        data = msg.data
        expected_len = 4 + 4 + 1 + SEGMENT_SIZE*2  # sec(4B)+nanosec(4B)+index(1B)+distance
        if len(data) != expected_len:
            self.get_logger().warn(f"Invalid segment length: {len(data)}")
            return

        seg_idx = data[8]
        if seg_idx >= SEGMENTS:
            self.get_logger().warn(f"Invalid segment index: {seg_idx}")
            return
        
        if seg_idx == 0:        
            self.sec = struct.unpack_from('<i', bytes(data[0:4]))[0]
            self.nanosec = struct.unpack_from('<I', bytes(data[4:8]))[0]

        # 거리 데이터
        segment_raw = data[9:]
        segment_data = struct.unpack('<' + 'H'*SEGMENT_SIZE, bytes(segment_raw))
        float_values = np.array(segment_data, dtype=np.float32) / 1000.0
        self.ranges[seg_idx * SEGMENT_SIZE:(seg_idx + 1) * SEGMENT_SIZE] = float_values
        self.received[seg_idx] = True

        # 모든 segment 수신 완료 시 LaserScan 발행
        if all(self.received):
            self.publish_scan()
            self.received = [False] * SEGMENTS

    def initialize_scan(self):
        # scan initialization
        scan = self.scan
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = -math.pi
        scan.angle_increment = 2*math.pi/SCAN_BUFFER_SIZE
        scan.angle_max = scan.angle_min + scan.angle_increment*(SCAN_BUFFER_SIZE-1) 
        scan.scan_time = 1.0 / SENSOR_HZ         # 센서 주파수에 맞춘 스캔 시간
        scan.time_increment = scan.scan_time / SCAN_BUFFER_SIZE
        
        scan.range_min = 0.15
        scan.range_max = 8.0

    def publish_scan(self):
        scan = self.scan

        stamp = self.get_clock().now().to_msg()
        scan.header.stamp = stamp
        scan.ranges = self.ranges.tolist()
        scan.intensities = []

        self.pub.publish(scan)
        self.get_logger().info('Published complete scan')

def main(args=None):
    rclpy.init(args=args)
    node = LidarReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
