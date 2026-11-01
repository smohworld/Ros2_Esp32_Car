#!/usr/bin/env python3
import argparse
import os
import shutil

WS = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description='Train YOLO on the toy dataset')
    ap.add_argument('--data', default=os.path.join(WS, 'datasets', 'toy', 'data.yaml'))
    ap.add_argument('--model', default='yolov8n.pt', help='pretrained start weights (yolov8n.pt is smallest/fastest)')
    ap.add_argument('--epochs', type=int, default=100)
    ap.add_argument('--imgsz', type=int, default=320, help='camera image is 240x320, so 320 is enough')
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--device', default='cpu', help='cpu or 0 (GPU)')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--patience', type=int, default=30, help='stop early if no improvement for N epochs')
    ap.add_argument('--name', default='toy')
    args = ap.parse_args()

    if not os.path.exists(args.data):
        raise SystemExit(f'{args.data} not found. Run split_dataset.py first.')

    from ultralytics import YOLO       # venv에만 설치되어 있으므로 여기서 import

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        patience=args.patience,
        project=os.path.join(WS, 'runs'),
        name=args.name,
        exist_ok=False,
        # 카메라가 고정 방향이라 위아래 뒤집기는 끄고(flipud=0 기본), 좌우 뒤집기/색/크기 변형은 기본값 사용
    )

    save_dir = str(model.trainer.save_dir)
    best = os.path.join(save_dir, 'weights', 'best.pt')
    os.makedirs(os.path.join(WS, 'models'), exist_ok=True)
    dst = os.path.join(WS, 'models', f'{args.name}_best.pt')
    shutil.copy2(best, dst)
    print(f'\nTraining results: {save_dir}')
    print(f'Best weights copied to: {dst}')
    print('Check results.png (mAP50 should rise toward 0.8~0.95) and val_batch0_pred.jpg')


if __name__ == '__main__':
    main()
