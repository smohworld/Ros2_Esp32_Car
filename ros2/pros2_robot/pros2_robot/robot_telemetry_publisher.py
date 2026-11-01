import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray, Float32, Int32
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import tf_transformations
from builtin_interfaces.msg import Time


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('encoder_to_odom_node')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub = self.create_subscription(
            Int64MultiArray,
            'encoder_deltas_gyroz_bat_rssi',
            self.encoder_callback,
            qos
        )

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)

        # 추가된 publisher들
        self.bat_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.rssi_pub = self.create_publisher(Int32, 'rssi_strength', 10)

        # 로봇 파라미터
        self.TICKS_PER_REV = 7*298*4      # 엔코더 해상도
        self.WHEEL_RADIUS = 0.065/2       # 바퀴 반지름 (m)

        # 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.time_log_count = 0

        # 이전 타임스탬프 저장용
        self.prev_sec = None
        self.prev_nanosec = None

        # 각 휠 회전 각도 누적
        self.front_left_pos = 0.0
        self.rear_left_pos = 0.0
        self.front_right_pos = 0.0
        self.rear_right_pos = 0.0

    def encoder_callback(self, msg):
        if len(msg.data) != 9:
            self.get_logger().warn("expected 9 values (sec, nanosec, FL, RL, FR, RR, GZ, BAT, RSSI)")
            return

        # 인코더 tick 차이값
        sec, nanosec, fl, rl, fr, rr, gz, batv, rssi = msg.data
        #self.get_logger().info(f'Encoder deltas and gyroz: {msg.data}')

        # 자이로 값 보정
        encoder_activity = math.sqrt((fl**2 + rl**2 + fr**2 + rr**2) / 4.0)
        ENCODER_IDLE_THRESHOLD = 3

        if encoder_activity < ENCODER_IDLE_THRESHOLD:
            gz *= 0.0 # 0.2  # 감쇠 계수 (0~1)

        # 배터리 전압 및 RSSI 발행
        bat_msg = Float32()
        bat_msg.data = float(batv)/1000
        self.bat_pub.publish(bat_msg)

        rssi_msg = Int32()
        rssi_msg.data = int(rssi)
        self.rssi_pub.publish(rssi_msg)

        stamp = self.get_clock().now().to_msg()

        # 시간 차 계산
        if self.prev_sec is not None:
            dt = (sec - self.prev_sec) + (nanosec - self.prev_nanosec) / 1e9
        else:
            dt = 0.0

        self.prev_sec = sec
        self.prev_nanosec = nanosec

        if dt <= 0:
            self.get_logger().warn("Invalid dt or first callback, skipping update")
            return

        # tick → 바퀴 회전 거리(m)
        wheel_circumference = 2 * math.pi * self.WHEEL_RADIUS
        fl_dist = (fl / self.TICKS_PER_REV) * wheel_circumference
        rl_dist = (rl / self.TICKS_PER_REV) * wheel_circumference
        fr_dist = (fr / self.TICKS_PER_REV) * wheel_circumference
        rr_dist = (rr / self.TICKS_PER_REV) * wheel_circumference

        # 좌우 평균
        left_dist = (fl_dist + rl_dist) / 2.0
        right_dist = (fr_dist + rr_dist) / 2.0
        dist = (left_dist + right_dist) / 2.0

        gyro_deg_s = gz / 131.0 # deg/s
        DEG_TO_RAD = math.pi / 180.0 # rad/s
        gyro_rad_s = gyro_deg_s * DEG_TO_RAD # rad
        delta_yaw = gyro_rad_s * dt

        # 위치 갱신
        self.x += dist * math.cos(self.yaw + delta_yaw / 2.0)
        self.y += dist * math.sin(self.yaw + delta_yaw / 2.0)
        self.yaw += delta_yaw
        
        # JointState 갱신
        self.front_left_pos  += (fl / self.TICKS_PER_REV) * 2 * math.pi
        self.rear_left_pos   += (rl / self.TICKS_PER_REV) * 2 * math.pi
        self.front_right_pos += (fr / self.TICKS_PER_REV) * 2 * math.pi
        self.rear_right_pos  += (rr / self.TICKS_PER_REV) * 2 * math.pi

        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        joint_msg.name = [
            'front_left_wheel_joint',
            'rear_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_right_wheel_joint'
        ]
        joint_msg.position = [
            self.front_left_pos,
            self.rear_left_pos,
            self.front_right_pos,
            self.rear_right_pos
        ]
        self.joint_pub.publish(joint_msg)

        # Odometry 메시지
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        odom_msg.twist.twist.linear.x = dist / dt
        odom_msg.twist.twist.angular.z = gyro_rad_s 
        self.odom_pub.publish(odom_msg)

        # TF 메시지 발행
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation

        tfm = TFMessage(transforms=[tf_msg])
        self.tf_pub.publish(tfm)


def main():
    rclpy.init()
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()