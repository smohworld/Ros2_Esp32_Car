import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import requests
import time

# --- 설정 변수 ---
STREAM_URL = "http://192.168.137.100/stream"
IMAGE_TOPIC = "/camera/image_raw"
INFO_TOPIC = "/camera/camera_info"
FRAME_ID = "camera_link"
VGA_WIDTH = 640
VGA_HEIGHT = 480
QVGA_WIDTH = 320
QVGA_HEIGHT = 240
FINAL_WIDTH = 240
FINAL_HEIGHT = 320
MIRROR_FIX = True
# ------------------

class MJPEGToImageRaw(Node):
    def __init__(self):
        super().__init__('mjpeg_to_image_raw_publisher')

        self.bridge = CvBridge()
        self.frame_id = FRAME_ID

        # Publishers
        self.image_publisher = self.create_publisher(Image, IMAGE_TOPIC, 50)
        self.info_publisher = self.create_publisher(CameraInfo, INFO_TOPIC, 50)
        self.stream_url = self.declare_parameter('stream_url', STREAM_URL).value
        self.bytes_data = b""
        self.latest_jpg = None
        self.frame_seq = 0          # 새 프레임이 들어올 때마다 증가
        self.published_seq = 0      # 마지막으로 발행한 프레임

        self.camera_info_msg = self._create_transformed_camera_info(
            VGA_WIDTH, VGA_HEIGHT, FINAL_WIDTH, FINAL_HEIGHT)

        self.get_logger().info(f"Connecting to MJPEG stream: {self.stream_url}")

        self.stream_thread = threading.Thread(target=self._stream_reader, daemon=True)
        self.stream_thread.start()

        # ROS publish 타이머 (~30 FPS, 새 프레임이 있을 때만 발행)
        self.timer = self.create_timer(0.03, self._publish_data)

    def _create_transformed_camera_info(self, orig_w, orig_h, new_w, new_h):
        """
        VGA (640x480) 기본 K 행렬을 가정하고, QVGA 리사이즈, 270도(반시계 90도) 회전, 좌우 반전 보정을 적용하여
        최종 240x320 해상도에 맞는 CameraInfo를 계산하여 생성합니다.
        """
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width = new_w
        msg.height = new_h

        # 1. VGA 원본 해상도 가정 (더미 값)
        # 비율: 500은 일반적인 웹캠의 가상 초점 거리
        fx_orig = 500.0
        fy_orig = 500.0
        cx_orig = orig_w / 2.0 # 320.0
        cy_orig = orig_h / 2.0 # 240.0

        # 2. QVGA 리사이즈 (배율 0.5) 적용
        scale = float(QVGA_WIDTH) / orig_w # 0.5
        fx_res = fx_orig * scale # 250.0
        fy_res = fy_orig * scale # 250.0
        cx_res = cx_orig * scale # 160.0
        cy_res = cy_orig * scale # 120.0

        # 3. 반시계 90도 회전 적용: (u, v) -> (u', v') = (v, W - u),  W = QVGA 폭(320)
        # K_rot = [ fy_res, 0,      cy_res     ]
        #         [ 0,      fx_res, W - cx_res ]
        #         [ 0,      0,      1          ]

        fx_rot = fy_res
        fy_rot = fx_res
        cx_rot = cy_res # 120.0 (새로운 중심점 X)
        cy_rot = QVGA_WIDTH - cx_res # 320 - 160 = 160.0 (새로운 중심점 Y)

        # 4. 좌우 반전 바로잡기: u'' = new_w - u'  ->  cx'' = new_w - cx'
        if MIRROR_FIX:
            cx_rot = new_w - cx_rot # 240 - 120 = 120.0

        # K 행렬 (내부 매개변수) 설정
        msg.k = [fx_rot, 0.0, cx_rot,
                 0.0, fy_rot, cy_rot,
                 0.0, 0.0, 1.0]

        # P 행렬 (투영 행렬) 설정
        msg.p = [fx_rot, 0.0, cx_rot, 0.0,
                 0.0, fy_rot, cy_rot, 0.0,
                 0.0, 0.0, 1.0, 0.0]

        return msg

    def _stream_reader(self):
        """ 별도 쓰레드: MJPEG 스트림에서 JPEG 프레임을 추출 """
        while True:
            try:
                stream = requests.get(self.stream_url, stream=True, timeout=5)

                if stream.status_code != 200:
                    self.get_logger().error(f"HTTP error: {stream.status_code}")
                    time.sleep(1)
                    continue

                self.bytes_data = b""   # 재접속 시 이전 연결의 남은 조각 버림
                for chunk in stream.iter_content(chunk_size=4096):
                    if not chunk: break
                    self.bytes_data += chunk
                    start = self.bytes_data.find(b'\xff\xd8')
                    # 끝 표시는 시작 이후에서 찾아야 함 (앞에 남은 이전 프레임 꼬리와 혼동 방지)
                    end = self.bytes_data.find(b'\xff\xd9', start + 2) if start != -1 else -1
                    if start != -1 and end != -1:
                        jpg = self.bytes_data[start:end+2]
                        self.bytes_data = self.bytes_data[end+2:]
                        if len(jpg) < 100: continue
                        self.latest_jpg = jpg
                        self.frame_seq += 1

            except Exception as e:
                self.get_logger().error(f"Stream error: {e}")
                time.sleep(1)
                continue

    def _publish_data(self):
        """ ROS2 Image와 CameraInfo를 동기화하여 publish """
        # 카메라는 약 10fps라서 같은 프레임을 반복 발행하지 않도록 새 프레임일 때만
        seq = self.frame_seq
        jpg = self.latest_jpg
        if jpg is None or seq == self.published_seq:
            return
        self.published_seq = seq

        try:
            # 1. JPEG 디코드 (Numpy -> OpenCV)
            np_arr = np.frombuffer(jpg, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn("Could not decode JPEG image.")
                return

            # 2. VGA 640x480 -> QVGA 320x240 리사이즈 (이미 QVGA면 그대로)
            if cv_image.shape[1] != QVGA_WIDTH or cv_image.shape[0] != QVGA_HEIGHT:
                cv_image = cv2.resize(cv_image, (QVGA_WIDTH, QVGA_HEIGHT), interpolation=cv2.INTER_LINEAR)

            # 3. 270도 회전 = 반시계 90도 (최종 240x320)
            rotated_img = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # 3-1. 좌우 반전 바로잡기 (로봇 오른쪽에 있는 물체가 화면 오른쪽에 보이도록)
            if MIRROR_FIX:
                rotated_img = cv2.flip(rotated_img, 1)

            # 4. OpenCV -> ROS Image 메시지 변환
            img_msg = self.bridge.cv2_to_imgmsg(rotated_img, encoding="bgr8")
            current_time = self.get_clock().now().to_msg()
            img_msg.header.stamp = current_time
            img_msg.header.frame_id = self.frame_id

            # 5. Image 발행
            self.image_publisher.publish(img_msg)

            # 6. CameraInfo 발행 (동기화)
            info_msg = self.camera_info_msg
            info_msg.header.stamp = current_time # 이미지와 동일한 타임스탬프
            self.info_publisher.publish(info_msg)

        except Exception as e:
            self.get_logger().error(f"Processing/Publishing error: {e}")
            return


def main(args=None):
    rclpy.init(args=args)
    node = MJPEGToImageRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
