#!/usr/bin/env python3
import argparse
import os
import random
import shutil

WS = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(WS, 'datasets', 'toy')


def load_classes(path):
    if os.path.exists(path):
        with open(path) as f:
            names = [line.strip() for line in f if line.strip()]
        if names:
            return names
    return ['toy']


def count_boxes(label_path):
    with open(label_path) as f:
        return sum(1 for line in f if line.strip())


def main():
    ap = argparse.ArgumentParser(description='Split labeled images into train/val and write data.yaml')
    ap.add_argument('--data', default=DATA, help='dataset folder (has images_raw, labels_raw)')
    ap.add_argument('--classes', default=os.path.join(WS, 'classes.txt'))
    ap.add_argument('--val', type=float, default=0.2, help='validation ratio')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    img_dir = os.path.join(args.data, 'images_raw')
    lbl_dir = os.path.join(args.data, 'labels_raw')
    pairs = []
    for f in sorted(os.listdir(img_dir)):
        if not f.lower().endswith(('.jpg', '.jpeg', '.png')):
            continue
        lbl = os.path.join(lbl_dir, os.path.splitext(f)[0] + '.txt')
        if os.path.exists(lbl):
            pairs.append((os.path.join(img_dir, f), lbl, count_boxes(lbl)))
    total_imgs = len([f for f in os.listdir(img_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))])
    if not pairs:
        raise SystemExit('No labeled images. Run label_tool.py first.')

    # 장난감 있는 사진 / 없는 사진을 각각 같은 비율로 나눔 (val에도 둘 다 들어가게)
    rng = random.Random(args.seed)
    positives = [p for p in pairs if p[2] > 0]
    negatives = [p for p in pairs if p[2] == 0]
    train, val = [], []
    for group in (positives, negatives):
        group = group[:]
        rng.shuffle(group)
        n_val = max(1, round(len(group) * args.val)) if len(group) >= 2 else 0
        val += group[:n_val]
        train += group[n_val:]

    for split in ('train', 'val'):
        for kind in ('images', 'labels'):
            d = os.path.join(args.data, kind, split)
            shutil.rmtree(d, ignore_errors=True)
            os.makedirs(d)
    for split, items in (('train', train), ('val', val)):
        for img, lbl, _ in items:
            shutil.copy2(img, os.path.join(args.data, 'images', split, os.path.basename(img)))
            shutil.copy2(lbl, os.path.join(args.data, 'labels', split, os.path.basename(lbl)))

    names = load_classes(args.classes)
    yaml_path = os.path.join(args.data, 'data.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f'path: {os.path.abspath(args.data)}\n')
        f.write('train: images/train\n')
        f.write('val: images/val\n')
        f.write('names:\n')
        for i, n in enumerate(names):
            f.write(f'  {i}: {n}\n')

    def stats(items):
        pos = sum(1 for p in items if p[2] > 0)
        return f'{len(items)} images ({pos} with toy, {len(items) - pos} background), {sum(p[2] for p in items)} boxes'

    print(f'labeled {len(pairs)} / {total_imgs} images')
    print(f'train: {stats(train)}')
    print(f'val  : {stats(val)}')
    print(f'wrote {yaml_path}')
    if len(positives) < 100:
        print(f'WARNING: only {len(positives)} images with the toy. 150~300 is recommended for good results.')
    if not negatives:
        print('TIP: add some background images (no toy, press "e" in label_tool) to reduce false detections.')


if __name__ == '__main__':
    main()
