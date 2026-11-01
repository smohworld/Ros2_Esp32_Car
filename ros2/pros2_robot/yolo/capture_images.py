#!/usr/bin/env python3
import argparse
import os
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

WS = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(WS, 'datasets', 'toy', 'images_raw')
WINDOW = 'YOLO capture (s: save, a: auto, q: quit)'


class Capture(Node):
    def __init__(self, topic):
        super().__init__('yolo_image_capture')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.create_subscription(Image, topic, self.image_cb, 10)

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        with self.lock:
            self.frame = img
            self.seq += 1


def count_images(folder):
    return len([f for f in os.listdir(folder) if f.lower().endswith(('.jpg', '.png'))])


def save_frame(folder, frame):
    name = 'toy_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3] + '.jpg'
    cv2.imwrite(os.path.join(folder, name), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return name


def spin_quietly(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main():
    ap = argparse.ArgumentParser(description='Save camera frames for YOLO training')
    ap.add_argument('--out', default=DEFAULT_OUT, help='output folder')
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--interval', type=float, default=0.5, help='auto save interval [s]')
    ap.add_argument('--auto', action='store_true', help='start with auto save on')
    ap.add_argument('--max', type=int, default=0, help='stop after saving N images (0 = no limit)')
    ap.add_argument('--no-view', action='store_true', help='no preview window')
    args, ros_args = ap.parse_known_args()

    os.makedirs(args.out, exist_ok=True)
    rclpy.init(args=ros_args)
    node = Capture(args.topic)
    spin_thread = threading.Thread(target=spin_quietly, args=(node,), daemon=True)
    spin_thread.start()

    auto = args.auto
    saved = 0
    last_save = 0.0
    last_saved_seq = -1
    window_shown = False
    print(f'Saving to {args.out} (already {count_images(args.out)} images)')
    print('keys: s = save, a = auto save on/off, q = quit')

    try:
        while rclpy.ok():
            with node.lock:
                frame, seq = node.frame, node.seq

            if frame is None:
                if not args.no_view:
                    wait = np.zeros((240, 480, 3), np.uint8)
                    cv2.putText(wait, f'waiting for {args.topic} ...', (10, 120),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.imshow(WINDOW, wait)
                    if (cv2.waitKey(100) & 0xFF) in (ord('q'), 27):
                        break
                else:
                    time.sleep(0.1)
                continue

            now = time.time()
            if auto and seq != last_saved_seq and now - last_save >= args.interval:
                save_frame(args.out, frame)
                saved += 1
                last_save, last_saved_seq = now, seq

            key = 255
            if not args.no_view:
                view = cv2.resize(frame, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
                total = count_images(args.out) if saved % 10 == 0 else None
                text = f'saved {saved}' + (f' / total {total}' if total is not None else '')
                cv2.putText(view, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if auto:
                    cv2.circle(view, (view.shape[1] - 20, 20), 9, (0, 0, 255), -1)
                    cv2.putText(view, 'AUTO', (view.shape[1] - 90, 27),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow(WINDOW, view)
                key = cv2.waitKey(30) & 0xFF
                # 창이 실제로 뜨기 전에는 보이지 않음(0)으로 나오므로, 한 번 보인 뒤 닫혔을 때만 종료
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1:
                    window_shown = True
                elif window_shown:
                    break
            else:
                time.sleep(0.03)

            if key in (ord('q'), 27):
                break
            elif key == ord('s'):
                name = save_frame(args.out, frame)
                saved += 1
                last_saved_seq = seq
                print(f'saved {name}')
            elif key == ord('a'):
                auto = not auto
                print('auto save', 'ON' if auto else 'OFF')

            if args.max and saved >= args.max:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print(f'Saved {saved} images this session, {count_images(args.out)} total in {args.out}')
        cv2.destroyAllWindows()
        # 컨텍스트를 먼저 내려 스핀 스레드를 끝낸 뒤 노드 정리 (순서가 반대면 종료 시 abort)
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
