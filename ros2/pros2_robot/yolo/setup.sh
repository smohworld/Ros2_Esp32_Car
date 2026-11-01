#!/bin/bash
# YOLO 학습 환경 설치 (한 번만)
#
# ROS가 쓰는 시스템 파이썬(numpy 1.21, cv2 4.5)을 건드리지 않도록 venv에 설치
# --system-site-packages: venv 안에서도 rclpy, cv_bridge 등 ROS 패키지를 그대로 사용
# PyTorch는 CPU 버전 (약 200MB), ultralytics 포함 전체 약 1GB
#
#   bash setup.sh
#   source venv/bin/activate     # 이후 train.py 실행 전에 (toy_detector는 자동으로 이 venv를 사용)
set -e
WS=$(cd "$(dirname "$0")" && pwd)
VENV="$WS/venv"
source /opt/ros/humble/setup.bash

if [ ! -x "$VENV/bin/python" ]; then
  # 이 이미지에는 ensurepip가 없어서 --without-pip로 만들고 시스템 pip로 venv에 설치
  python3 -m venv --system-site-packages --without-pip "$VENV"
fi
PY="$VENV/bin/python"

"$PY" -m pip install --upgrade pip
"$PY" -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
# numpy 2.x는 ROS Humble(cv_bridge)과 맞지 않아서 1.x로 고정
"$PY" -m pip install ultralytics "numpy<2"

"$PY" - <<'EOF'
import torch, ultralytics, numpy, cv2, rclpy, cv_bridge
print(f'torch {torch.__version__}, ultralytics {ultralytics.__version__}, '
      f'numpy {numpy.__version__}, cv2 {cv2.__version__}')
print('rclpy / cv_bridge import OK')
EOF
echo
echo "Done. Activate with:  source $VENV/bin/activate"
