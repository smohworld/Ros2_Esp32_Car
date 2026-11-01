#!/usr/bin/env python3
import argparse
import os
import shutil
import tkinter as tk

from PIL import Image, ImageTk

WS = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(WS, 'datasets', 'toy')
COLORS = ['#00ff40', '#ff4040', '#40a0ff', '#ffd000', '#ff40ff',
          '#40ffff', '#ff8000', '#a0ff00', '#8080ff', '#ffffff']
MIN_BOX_PX = 4


def load_classes(path):
    if os.path.exists(path):
        with open(path) as f:
            names = [line.strip() for line in f if line.strip()]
        if names:
            return names
    return ['toy']


class LabelTool:
    def __init__(self, images_dir, labels_dir, classes):
        self.images_dir = images_dir
        self.labels_dir = labels_dir
        self.excluded_dir = os.path.join(os.path.dirname(images_dir.rstrip('/')), 'excluded')
        self.classes = classes
        os.makedirs(labels_dir, exist_ok=True)

        self.files = sorted(f for f in os.listdir(images_dir)
                            if f.lower().endswith(('.jpg', '.jpeg', '.png')))
        self.idx = 0
        self.img = None
        self.boxes = []             # [(cls, x0, y0, x1, y1)] 이미지 픽셀 좌표
        self.touched = False        # 이 이미지에서 편집했는지 (안 건드린 미라벨 이미지는 저장 안 함)
        self.cls = 0
        self.drag = None            # 드래그 시작 (이미지 좌표)
        self.mouse = None           # 현재 마우스 (캔버스 좌표)
        self.scale = 1.0
        self.offx = self.offy = 0
        self.tk_img = None

        self.root = tk.Tk()
        self.root.title('YOLO Label Tool')
        self.root.geometry('900x900')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        top = tk.Frame(self.root)
        top.pack(side=tk.TOP, fill=tk.X)
        self.class_var = tk.StringVar()
        tk.Label(top, textvariable=self.class_var, font=('Helvetica', 12, 'bold')).pack(side=tk.LEFT, padx=6)
        tk.Label(top, text='drag: box | right-click: delete | d/a: next/prev | e: no toy | '
                           'n: next unlabeled | x: exclude', fg='#555').pack(side=tk.LEFT, padx=6)

        self.status = tk.StringVar()
        tk.Label(self.root, textvariable=self.status, anchor='w').pack(side=tk.BOTTOM, fill=tk.X)

        self.canvas = tk.Canvas(self.root, bg='#303030', highlightthickness=0, cursor='none')
        self.canvas.pack(fill=tk.BOTH, expand=True)

        c = self.canvas
        c.bind('<Configure>', lambda e: self.redraw(rescale=True))
        c.bind('<ButtonPress-1>', self.on_press)
        c.bind('<B1-Motion>', self.on_motion)
        c.bind('<ButtonRelease-1>', self.on_release)
        c.bind('<Motion>', self.on_motion)
        c.bind('<Leave>', lambda e: self.set_mouse(None))
        c.bind('<Button-3>', self.on_right_click)
        r = self.root
        for k in ('d', 'Right', 'space'):
            r.bind(f'<KeyPress-{k}>', lambda e: self.go(1))
        for k in ('a', 'Left'):
            r.bind(f'<KeyPress-{k}>', lambda e: self.go(-1))
        r.bind('<KeyPress-e>', lambda e: self.mark_empty())
        r.bind('<KeyPress-n>', lambda e: self.next_unlabeled())
        r.bind('<KeyPress-c>', lambda e: self.clear_boxes())
        r.bind('<KeyPress-x>', lambda e: self.exclude())
        r.bind('<Control-z>', lambda e: self.undo())
        for i in range(min(10, len(self.classes))):
            r.bind(f'<KeyPress-{i}>', lambda e, i=i: self.select_class(i))

        self.select_class(0)
        if self.files:
            first = next((i for i, f in enumerate(self.files) if not os.path.exists(self.label_path(f))), 0)
            self.load(first)
        else:
            self.status.set(f'No images in {images_dir}. Run capture_images.py first.')

    # ---------- 파일 ----------
    def label_path(self, fname):
        return os.path.join(self.labels_dir, os.path.splitext(fname)[0] + '.txt')

    def load(self, idx):
        self.idx = idx
        fname = self.files[idx]
        self.img = Image.open(os.path.join(self.images_dir, fname)).convert('RGB')
        w, h = self.img.size
        self.boxes = []
        path = self.label_path(fname)
        if os.path.exists(path):
            with open(path) as f:
                for line in f:
                    parts = line.split()
                    if len(parts) != 5:
                        continue
                    c, cx, cy, bw, bh = int(parts[0]), *map(float, parts[1:])
                    self.boxes.append((c, (cx - bw / 2) * w, (cy - bh / 2) * h,
                                       (cx + bw / 2) * w, (cy + bh / 2) * h))
        self.touched = False
        self.redraw(rescale=True)

    def save(self):
        """박스가 있거나 이 이미지에서 편집했거나 원래 라벨 파일이 있으면 저장"""
        if self.img is None:
            return
        fname = self.files[self.idx]
        path = self.label_path(fname)
        if not (self.boxes or self.touched or os.path.exists(path)):
            return
        w, h = self.img.size
        with open(path, 'w') as f:
            for c, x0, y0, x1, y1 in self.boxes:
                f.write(f'{c} {(x0 + x1) / 2 / w:.6f} {(y0 + y1) / 2 / h:.6f} '
                        f'{(x1 - x0) / w:.6f} {(y1 - y0) / h:.6f}\n')

    def go(self, step):
        if not self.files:
            return
        self.save()
        self.load(min(max(self.idx + step, 0), len(self.files) - 1))

    def mark_empty(self):
        if not self.files:
            return
        self.boxes = []
        self.touched = True
        self.go(1)

    def next_unlabeled(self):
        if not self.files:
            return
        self.save()
        n = len(self.files)
        for k in range(1, n + 1):
            i = (self.idx + k) % n
            if not os.path.exists(self.label_path(self.files[i])):
                self.load(i)
                return
        self.redraw()
        self.status.set(self.status.get() + '   -> all images labeled!')

    def exclude(self):
        if not self.files:
            return
        fname = self.files[self.idx]
        os.makedirs(self.excluded_dir, exist_ok=True)
        shutil.move(os.path.join(self.images_dir, fname), os.path.join(self.excluded_dir, fname))
        if os.path.exists(self.label_path(fname)):
            os.remove(self.label_path(fname))
        del self.files[self.idx]
        if not self.files:
            self.img = None
            self.canvas.delete('all')
            self.status.set('No images left')
            return
        self.load(min(self.idx, len(self.files) - 1))

    def on_close(self):
        self.save()
        self.root.destroy()

    # ---------- 편집 ----------
    def select_class(self, i):
        self.cls = i
        self.class_var.set(f'class {i}: {self.classes[i]}')
        self.redraw()

    def undo(self):
        if self.boxes:
            self.boxes.pop()
            self.touched = True
            self.redraw()

    def clear_boxes(self):
        if self.boxes:
            self.boxes = []
            self.touched = True
            self.redraw()

    def c2i(self, cx, cy):
        w, h = self.img.size
        x = min(max((cx - self.offx) / self.scale, 0), w)
        y = min(max((cy - self.offy) / self.scale, 0), h)
        return x, y

    def i2c(self, x, y):
        return self.offx + x * self.scale, self.offy + y * self.scale

    def on_press(self, e):
        if self.img is not None:
            self.drag = self.c2i(e.x, e.y)

    def on_motion(self, e):
        self.set_mouse((e.x, e.y))

    def on_release(self, e):
        if self.drag is None:
            return
        x0, y0 = self.drag
        x1, y1 = self.c2i(e.x, e.y)
        self.drag = None
        if abs(x1 - x0) >= MIN_BOX_PX and abs(y1 - y0) >= MIN_BOX_PX:
            self.boxes.append((self.cls, min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))
            self.touched = True
        self.redraw()

    def on_right_click(self, e):
        if self.img is None:
            return
        x, y = self.c2i(e.x, e.y)
        # 클릭한 점을 포함하는 박스 중 가장 작은 것 삭제
        hits = [i for i, (_, x0, y0, x1, y1) in enumerate(self.boxes) if x0 <= x <= x1 and y0 <= y <= y1]
        if hits:
            i = min(hits, key=lambda i: (self.boxes[i][3] - self.boxes[i][1]) * (self.boxes[i][4] - self.boxes[i][2]))
            del self.boxes[i]
            self.touched = True
            self.redraw()

    def set_mouse(self, pos):
        self.mouse = pos
        self.draw_overlay()

    # ---------- 그리기 ----------
    def redraw(self, rescale=False):
        if self.img is None:
            return
        c = self.canvas
        if rescale:
            cw, ch = max(c.winfo_width(), 1), max(c.winfo_height(), 1)
            w, h = self.img.size
            self.scale = max(0.1, min(cw / w, ch / h))
            sw, sh = int(w * self.scale), int(h * self.scale)
            self.offx, self.offy = (cw - sw) // 2, (ch - sh) // 2
            self.tk_img = ImageTk.PhotoImage(self.img.resize((max(sw, 1), max(sh, 1)), Image.NEAREST))
        c.delete('all')
        c.create_image(self.offx, self.offy, image=self.tk_img, anchor='nw')
        for cls, x0, y0, x1, y1 in self.boxes:
            color = COLORS[cls % len(COLORS)]
            a, b = self.i2c(x0, y0), self.i2c(x1, y1)
            c.create_rectangle(*a, *b, outline=color, width=2)
            name = self.classes[cls] if cls < len(self.classes) else str(cls)
            c.create_text(a[0] + 2, a[1] - 2, anchor='sw', text=name, fill=color,
                          font=('Helvetica', 10, 'bold'))
        self.draw_overlay()
        self.update_status()

    def draw_overlay(self):
        c = self.canvas
        c.delete('overlay')
        if self.img is None or self.mouse is None:
            return
        mx, my = self.mouse
        color = COLORS[self.cls % len(COLORS)]
        # 박스를 딱 맞게 그리기 쉽도록 십자선
        x0, y0 = self.i2c(0, 0)
        x1, y1 = self.i2c(*self.img.size)
        c.create_line(x0, my, x1, my, fill='#ffff00', dash=(3, 3), tags='overlay')
        c.create_line(mx, y0, mx, y1, fill='#ffff00', dash=(3, 3), tags='overlay')
        if self.drag is not None:
            a = self.i2c(*self.drag)
            c.create_rectangle(*a, mx, my, outline=color, width=2, dash=(4, 2), tags='overlay')

    def update_status(self):
        n = len(self.files)
        labeled = sum(os.path.exists(self.label_path(f)) for f in self.files)
        fname = self.files[self.idx]
        state = 'labeled' if os.path.exists(self.label_path(fname)) else 'NOT labeled'
        if self.touched:
            state += ' (edited, saves on next/prev)'
        self.status.set(f'{self.idx + 1}/{n}  {fname}  [{state}]  boxes {len(self.boxes)}   '
                        f'progress {labeled}/{n}')


def main():
    ap = argparse.ArgumentParser(description='Minimal YOLO bounding box labeling tool')
    ap.add_argument('--images', default=os.path.join(DATA, 'images_raw'))
    ap.add_argument('--labels', default=os.path.join(DATA, 'labels_raw'))
    ap.add_argument('--classes', default=os.path.join(WS, 'classes.txt'))
    args = ap.parse_args()
    if not os.path.isdir(args.images):
        raise SystemExit(f'{args.images} not found. Run capture_images.py first.')
    tool = LabelTool(args.images, args.labels, load_classes(args.classes))
    tool.root.mainloop()


if __name__ == '__main__':
    main()
