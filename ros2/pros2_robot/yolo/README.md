# 장난감 YOLO 학습 (pros2_robot/yolo)

ESP32-CAM 영상(`mjpeg_camera_publisher.py` → `/camera/image_raw`, 270도 회전된 240x320)으로
장난감 검출 모델을 학습합니다. 학습과 검출에 같은 영상을 쓰므로 방향과 해상도가 그대로 맞습니다.

```
pros2_robot/yolo
├── setup.sh             0. 설치 (한 번만)
├── capture_images.py    1. 이미지 수집
├── label_tool.py        2. 라벨링
├── split_dataset.py     3. train/val 분할 + data.yaml
├── train.py             4. 학습
│   (5. 검출 / 6. 따라가기는 패키지 노드: ros2 run pros2_robot toy_detector / toy_tracker)
├── classes.txt          클래스 이름 (기본: toy)
├── datasets/toy/        images_raw, labels_raw, images/, labels/, data.yaml
├── runs/                학습 결과
└── models/toy_best.pt   최종 모델
```

모든 터미널에서 먼저:
```bash
source /opt/ros/humble/setup.bash
cd ~/pros2_ws/src/pros2_robot/yolo
```

## 0. 설치 (한 번만, 약 1GB 다운로드)
```bash
bash setup.sh
```
ROS 시스템 파이썬은 그대로 두고 `venv`에 PyTorch(CPU)와 ultralytics를 설치합니다.

## 1. 이미지 수집
터미널 1: 카메라
```bash
ros2 launch pros2_robot robot_bringup.launch.py   # 카메라(mjpeg_camera_publisher) 포함
```
터미널 2: 수집
```bash
python3 capture_images.py
```
- 미리보기 창에서 `s` 한 장 저장, `a` 자동 저장(0.5초마다) 켜기/끄기, `q` 종료
- 저장 위치: `datasets/toy/images_raw/`

**잘 찍는 법 (결과의 대부분이 여기서 결정됨)**
- **장난감 있는 사진 150~300장**: 처음엔 150장으로 시작해서 부족하면 추가
- **거리**: 가까이(화면 가득) ~ 멀리(작게, 2~3m)까지 고루
- **각도**: 장난감을 돌려 가며 앞/옆/뒤/위에서
- **위치**: 화면 왼쪽/오른쪽/가장자리에 걸친 것도 (따라가기 중엔 가장자리에 자주 나옴)
- **배경/조명**: 로봇이 실제로 다닐 곳들에서, 밝을 때/어두울 때
- **일부 가려진 장난감**도 조금
- **장난감 없는 배경 사진 20~40장**: 오검출 감소 (라벨링에서 `e`)
- 로봇에 카메라가 달린 상태 그대로 찍는 게 가장 좋음 (실제 높이/각도)
- 자동 저장으로 찍을 땐 비슷한 사진이 너무 많지 않게 장난감이나 로봇을 계속 움직이기

## 2. 라벨링
```bash
python3 label_tool.py
```
| 조작 | 동작 |
|---|---|
| 왼쪽 드래그 | 박스 그리기 |
| 오른쪽 클릭 | 박스 삭제 |
| `d` / `→` / `Space` | 저장 후 다음 |
| `a` / `←` | 저장 후 이전 |
| `e` | 장난감 없음(빈 라벨)으로 저장 후 다음 |
| `n` | 다음 '라벨 안 된' 이미지로 |
| `Ctrl+Z` / `c` | 마지막 박스 취소 / 박스 전부 지우기 |
| `x` | 흐리거나 중복된 사진 제외 (`excluded/`로 이동) |

- 다시 실행하면 라벨 안 된 첫 이미지부터 이어서 합니다.
- 결과: `datasets/toy/labels_raw/<이미지이름>.txt` (YOLO 형식 `class cx cy w h`, 0~1 값)

**라벨링 규칙 (일관성이 제일 중요)**
- 박스는 **보이는 부분에 딱 맞게** (여백 없이, 잘리지 않게). 노란 십자선으로 끝을 맞추면 쉬움
- 일부 가려졌으면 **보이는 부분만**
- 화면 밖으로 잘린 장난감도 보이는 부분이 충분하면 박스
- 너무 흐리거나 장난감인지 애매한 사진은 `x`로 제외 (애매한 걸 억지로 라벨하면 성능이 떨어짐)
- 한 사진에 장난감이 여러 개면 **전부** 박스
- 장난감 없는 사진은 반드시 `e` (안 누르고 넘기면 '라벨 안 됨'으로 남아서 학습에서 빠짐)

## 3. train/val 분할
```bash
python3 split_dataset.py
```
라벨 있는 이미지를 80% 학습 / 20% 검증으로 나누고 `datasets/toy/data.yaml`을 만듭니다.
사진을 추가로 찍고 라벨링하면 다시 실행하면 됩니다.

## 4. 학습
```bash
source venv/bin/activate
python train.py
```
- 기본: `yolov8n`(가장 작고 빠름), 320 해상도, 최대 100 epoch, CPU, 30 epoch 동안 나아지지 않으면 조기 종료
- 시간: 이미지 200장 기준 CPU 16코어로 대략 20~60분
- 결과: `runs/toy/` (그래프 `results.png`, 검증 예시 `val_batch0_pred.jpg`), 최종 모델 `models/toy_best.pt` (이후 `colcon build --packages-select pros2_robot`로 설치해야 toy_detector가 사용)
- 확인: `results.png`의 **mAP50**이 0.8 이상이면 좋음, 0.6 이하면 사진/라벨을 늘리거나 점검

## 5. 실시간 확인
카메라를 켠 상태에서:
```bash
ros2 run pros2_robot toy_detector
```
- 화면에 박스와 확신도, fps 표시. 검출 결과는 `/toy_detection` 토픽으로 발행 (따라가기 노드에서 사용)
  - `data = [found, cx(-1 왼쪽 ~ +1 오른쪽), cy, w, h, conf]`
- 오검출이 많으면 `--conf 0.6`, 잘 못 잡으면 `--conf 0.35`

## 6. 장난감 따라가기 (toy_tracker, 카메라만 사용)
필요한 것: 로봇 연결(`robot_bringup.launch.py`, cmd_vel), 카메라, `toy_detector`
```bash
ros2 launch pros2_robot robot_bringup.launch.py   # 로봇 + 카메라
ros2 run pros2_robot toy_detector                  # 검출 (화면에 cx, h 표시)
ros2 run pros2_robot toy_tracker                   # 따라가기 (기본 30cm 유지)
ros2 run pros2_robot toy_tracker --start-disabled  # 미션용 (미션이 켜고 끔)
```
**처음 한 번 (카메라 보정)**: 장난감을 로봇 **정면** 30cm에 두고
```bash
ros2 run pros2_robot toy_tracker --calibrate               # 다른 거리면 --dist 0.25 처럼 지정
```
→ `~/.ros/toy_tracker_calib.yaml`에 `cx_offset`(정면일 때 cx), `ref_h`(그 거리의 박스 높이) 저장.
카메라를 다시 붙이거나 방향을 바꾸면 다시 보정

**동작**
- 방향 = `cx - cx_offset`, 거리 = `ref_dist * ref_h / h` 로 `--dist`(기본 0.30m) 유지
- 확신도: 놓친 상태에서는 `--conf`(0.7) 한 번 또는 `--keep-conf`(0.5) 3번 연속이면 시작, 따라가는 중에는 `--keep-conf` 이상이면 유지
- 놓치면(0.6초) 바로 놓친 쪽으로 찾을 때까지 회전
  (놓친 쪽: 화면 가장자리면 빠져나간 쪽 > 화면에서 움직이던 쪽 > 정면 기준 장난감 쪽),
  `--search-w`에서 시작해 `--search-acc`로 `--search-w-max`까지 빨라짐
- `/toy_detection`이 0.5초 이상 안 오면(카메라/검출기 끊김) 정지
- 장애물 검사 없음. **Nav2 목표와 동시에 쓰지 말 것** (`/cmd_vel` 충돌). 일시정지/재개:
  ```bash
  ros2 topic pub --once /toy_tracker/enable std_msgs/Bool "{data: false}"
  ```
- 상태 확인: `ros2 run pros2_robot toy_tracker --ros-args --log-level toy_tracker:=debug`

**조정 (`ros2 run pros2_robot toy_tracker -h`로 전체 목록)**
| 증상 | 조정 |
|---|---|
| 회전이 과하게 넘어감 / 좌우로 흔들림 | `--turn-max 0.4` |
| 가다가 화면 옆으로 놓침 | `--turn-max 0.8` |
| 찾을 때 너무 느림 / 빠름 | `--search-w`, `--search-w-max`, `--search-acc` |
| 엉뚱한 물체를 따라감 | `--conf 0.85 --keep-conf 0.6`, 그 물체 사진을 배경(`e`)으로 추가 학습 |
| 자주 놓침 | `toy_detector --conf 0.3`과 `--keep-conf 0.3`, 여러 각도 사진 추가 학습 |
| 정면에 있는데 비스듬히 멈춤 / 거리가 안 맞음 | `--calibrate` 다시 |

## 성능이 부족할 때
| 증상 | 해결 |
|---|---|
| 멀리 있으면 못 잡음 | 멀리서 찍은 사진 추가 |
| 특정 각도에서 못 잡음 | 그 각도 사진 추가 |
| 다른 물건을 장난감으로 착각 | 그 물건이 나오는 배경 사진(`e`) 추가 |
| 어두운 곳에서 못 잡음 | 어두운 곳 사진 추가 |
| 박스가 들쭉날쭉 | 라벨 박스를 다시 딱 맞게 정리 |

사진을 추가했으면 2 → 3 → 4를 다시 하면 됩니다.
