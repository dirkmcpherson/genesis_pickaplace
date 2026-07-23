"""Tile per-episode rollout mp4s into ONE grid video (episodes play side by side).

Shorter episodes freeze on their last frame. Each tile is labeled "<idx>: <outcome>"
from the filename convention <tag>_<idx>_<outcome>.mp4 (eval_core naming).

Usage: tile_videos.py <video_dir_or_globs...> --out tiled.mp4 [--cols 4] [--tile-h 240]
"""
import argparse, glob, math, pathlib as pl
import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('inputs', nargs='+', help='dirs or mp4 paths')
ap.add_argument('--out', required=True)
ap.add_argument('--cols', type=int, default=None, help='default: ceil(sqrt(n))')
ap.add_argument('--tile-h', type=int, default=240)
ap.add_argument('--fps', type=int, default=20)
args = ap.parse_args()

paths = []
for x in args.inputs:
    p = pl.Path(x)
    paths.extend(sorted(p.glob('*.mp4')) if p.is_dir() else [p])
paths = [p for p in paths if p.suffix == '.mp4']
assert paths, 'no input videos'

clips = []
for p in paths:
    cap = cv2.VideoCapture(str(p))
    frames = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(fr)
    cap.release()
    if not frames:
        continue
    parts = p.stem.split('_')
    label = f"{parts[1]}: {'_'.join(parts[2:])}" if len(parts) >= 3 else p.stem
    clips.append((label, frames))
assert clips, 'no readable videos'

n = len(clips)
cols = args.cols or math.ceil(math.sqrt(n))
rows = math.ceil(n / cols)
th = args.tile_h
tw = int(round(th * clips[0][1][0].shape[1] / clips[0][1][0].shape[0]))
T = max(len(f) for _, f in clips)

vw = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'), args.fps,
                     (cols * tw, rows * th))
blank = np.zeros((th, tw, 3), dtype=np.uint8)
for t in range(T):
    grid = []
    for r in range(rows):
        row = []
        for c in range(cols):
            i = r * cols + c
            if i < n:
                label, frames = clips[i]
                fr = frames[min(t, len(frames) - 1)]
                fr = cv2.resize(fr, (tw, th))
                cv2.putText(fr, label, (6, th - 8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(fr, label, (6, th - 8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (255, 255, 255), 1, cv2.LINE_AA)
                row.append(fr)
            else:
                row.append(blank)
        grid.append(np.hstack(row))
    vw.write(np.vstack(grid))
vw.release()
print(f'{args.out}: {n} episodes tiled {rows}x{cols}, {T} frames')
