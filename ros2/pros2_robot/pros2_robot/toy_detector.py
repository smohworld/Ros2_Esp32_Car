#!/usr/bin/env python3
import argparse
import glob
import os
import sys
import threading
import time

YOLO_VENV = os.environ.get(
    'PROS2_YOLO_VENV', os.path.expanduser('~/pros2_ws/src/pros2_robot/yolo/venv'))
for _site in glob.glob(os.path.join(YOLO_VENV, 'lib', 'python3*', 'site-packages')):
    if _site not in sys.path:
        sys.path.insert(0, _site)

import cv2  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Float32MultiArray  # noqa: E402

DEFAULT_MODEL = os.path.join(get_package_share_directory('pros2_robot'), 'models', 'toy_best.pt')
WINDOW = 'YOLO detection (q: quit)'


class Detector(Node):
    def __init__(self, topic):
        super().__init__('toy_detector')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.create_subscription(Image, topic, self.image_cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, 'toy_detection', 10)
        self.image_pub = self.create_publisher(Image, 'toy_detection/image', 2)

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        with self.lock:
            self.frame = img
            self.seq += 1


def draw_boxes(frame, boxes, names):
    view = frame.copy()
    if boxes is not None and len(boxes) > 0:
        best = int(boxes.conf.argmax())
        for i in range(len(boxes)):
            x0, y0, x1, y1 = map(int, boxes.xyxy[i].tolist())
            cls = int(boxes.cls[i])
            color = (0, 255, 0) if i == best else (0, 160, 255)
            cv2.rectangle(view, (x0, y0), (x1, y1), color, 2)
            cv2.putText(view, f'{names[cls]} {float(boxes.conf[i]):.2f}', (x0, max(12, y0 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    return view


def status_text(fps, data):
    found = data[0] > 0
    text = f'{fps:.1f} fps  ' + (f'cx={data[1]:+.2f} h={data[4]:.2f}' if found else 'no toy')
    return text, (0, 255, 0) if found else (0, 0, 255)


def spin_quietly(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass        # 종료 시 컨텍스트가 먼저 내려가면 발생 (정상)


def main():
    ap = argparse.ArgumentParser(description='Run trained YOLO on the robot camera')
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--conf', type=float, default=0.5, help='minimum confidence')
    ap.add_argument('--imgsz', type=int, default=320)
    ap.add_argument('--no-view', action='store_true')
    args, ros_args = ap.parse_known_args()

    if not os.path.exists(args.model):
        raise SystemExit(f'{args.model} not found. Run yolo/train.py and colcon build first.')
    from ultralytics import YOLO
    model = YOLO(args.model)
    names = model.names

    rclpy.init(args=ros_args)
    node = Detector(args.topic)
    spin_thread = threading.Thread(target=spin_quietly, args=(node,), daemon=True)
    spin_thread.start()

    last_seq = 0
    fps = 0.0
    window_shown = False
    try:
        while rclpy.ok():
            with node.lock:
                frame, seq = node.frame, node.seq
            if frame is None or seq == last_seq:
                if not args.no_view and (cv2.waitKey(10) & 0xFF) in (ord('q'), 27):
                    break
                time.sleep(0.005)
                continue
            last_seq = seq

            t0 = time.time()
            result = model.predict(frame, imgsz=args.imgsz, conf=args.conf, device='cpu', verbose=False)[0]
            dt = time.time() - t0
            fps = 0.9 * fps + 0.1 * (1.0 / max(dt, 1e-3)) if fps else 1.0 / max(dt, 1e-3)

            h, w = frame.shape[:2]
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                best = int(boxes.conf.argmax())
                x0, y0, x1, y1 = boxes.xyxy[best].tolist()
                conf = float(boxes.conf[best])
                msg.data = [1.0,
                            ((x0 + x1) / 2 / w) * 2 - 1,
                            ((y0 + y1) / 2 / h) * 2 - 1,
                            (x1 - x0) / w,
                            (y1 - y0) / h,
                            conf]
            node.pub.publish(msg)

            publish_view = node.image_pub.get_subscription_count() > 0
            if publish_view or not args.no_view:
                boxed = draw_boxes(frame, boxes, names)
                text, color = status_text(fps, msg.data)
            if publish_view:
                out = boxed.copy()
                cv2.putText(out, text, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                img_msg = node.bridge.cv2_to_imgmsg(out, 'bgr8')
                img_msg.header.stamp = node.get_clock().now().to_msg()
                node.image_pub.publish(img_msg)

            if not args.no_view:
                view = cv2.resize(boxed, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
                cv2.putText(view, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.imshow(WINDOW, view)
                if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                    break
                # 창이 실제로 뜨기 전에는 보이지 않음(0)으로 나오므로, 한 번 보인 뒤 닫혔을 때만 종료
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1:
                    window_shown = True
                elif window_shown:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        # 컨텍스트를 먼저 내려 스핀 스레드를 끝낸 뒤 노드 정리 (순서가 반대면 종료 시 abort)
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
