#!/usr/bin/env python
"""Side-by-side videos of matched-IC tapes: human (dHv2raw) left, machine (dDP) right, assembled from the
stored 64x64 rig frames (top view above the wrist view), 4x nearest-neighbour upscale, 7.5 fps, with uid,
set, decision index and a red-bordered GRASP marker at the closure decision (first grip cmd > 0.5). The
shorter tape freezes on its final (picked) frame, labelled PICKED. No simulation is run.

Usage:
  ~/workspace/genesis_sim2real/venv/bin/python paper/gallery_dDP_vs_dHv2raw_2026-09-04.py \
      --sets ~/wm_fix_2026-09-03/characterization/sets --out ~/wm_fix_2026-09-03/characterization \
      [--uids 232 233 ...]   (default: gallery_uids.json written by characterize_dDP_vs_dHv2raw_2026-09-04.py)
"""
import argparse, glob, json, os, pathlib as pl, subprocess
import cv2, numpy as np

SC, HDR, GAP, HOLD = 4, 44, 8, 12          # upscale, header px, gap px, frames held on the final frame
W = 64 * SC
SETS = {'human': 'dHv2raw', 'machine': 'dDP'}
FONT = cv2.FONT_HERSHEY_SIMPLEX


def load_set(d):
    out = {}
    for f in sorted(glob.glob(os.path.join(d, '*.npz'))):
        z = np.load(f, allow_pickle=True); out[int(z['ic_uid'])] = f
    return out


def tape(f):
    z = np.load(f, allow_pickle=True)
    im = np.asarray(z['images']); a = np.asarray(z['actions']); n = len(a)
    tg = int(np.argmax(a[:, 6] > 0.5)) if (a[:, 6] > 0.5).any() else n - 1
    assert im.shape == (n + 1, 64, 64, 6), im.shape
    return im, n, tg


def column(im, n, tg, t, uid, label, setname):
    """One 2*W-tall column (top view over wrist view) with a header, at global frame t."""
    k = min(t, n)
    top = cv2.resize(np.ascontiguousarray(im[k, :, :, :3]), (W, W), interpolation=cv2.INTER_NEAREST)
    wr = cv2.resize(np.ascontiguousarray(im[k, :, :, 3:]), (W, W), interpolation=cv2.INTER_NEAREST)
    hdr = np.full((HDR, W, 3), 255, np.uint8)
    cv2.putText(hdr, f'uid {uid}  {label} ({setname})', (4, 12), FONT, 0.40, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(hdr, f'decision {k}/{n}   grasp @{tg}', (4, 26), FONT, 0.40, (0, 0, 0), 1, cv2.LINE_AA)
    if t >= n:
        cv2.putText(hdr, 'PICKED (end of tape)', (4, 40), FONT, 0.40, (0, 130, 0), 1, cv2.LINE_AA)
    elif tg <= t <= tg + 3:
        cv2.putText(hdr, 'GRASP (grip cmd > 0.5)', (4, 40), FONT, 0.40, (220, 0, 0), 1, cv2.LINE_AA)
    frame = np.concatenate([hdr, top, wr], axis=0)
    if tg <= t <= tg + 3:
        cv2.rectangle(frame, (0, HDR), (W - 1, HDR + 2 * W - 1), (220, 0, 0), 5)
    elif t >= n:
        cv2.rectangle(frame, (0, HDR), (W - 1, HDR + 2 * W - 1), (0, 130, 0), 3)
    return frame


def render(uid, fh, fm, out_path, fps=7.5):
    imh, nh, tgh = tape(fh); imm, nm, tgm = tape(fm)
    T = max(nh, nm) + HOLD
    H, Wtot = HDR + 2 * W, 2 * W + GAP
    cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{Wtot}x{H}', '-r', str(fps),
           '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18', '-preset', 'medium', '-movflags', '+faststart', str(out_path)]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    gap = np.full((H, GAP, 3), 255, np.uint8)
    for t in range(T):
        fr = np.concatenate([column(imh, nh, tgh, t, uid, 'human', SETS['human']), gap,
                             column(imm, nm, tgm, t, uid, 'machine', SETS['machine'])], axis=1)
        p.stdin.write(np.ascontiguousarray(fr).tobytes())
        if t == max(tgh, tgm):
            cv2.imwrite(str(out_path).replace('.mp4', '_grasp_frame.png'), fr[:, :, ::-1])
    p.stdin.close(); rc = p.wait()
    assert rc == 0, f'ffmpeg rc {rc} for {out_path}'
    return dict(uid=uid, n_human=nh, n_machine=nm, grasp_human=tgh, grasp_machine=tgm, frames=T)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sets', default=os.path.expanduser('~/wm_fix_2026-09-03/characterization/sets'))
    ap.add_argument('--out', default=os.path.expanduser('~/wm_fix_2026-09-03/characterization'))
    ap.add_argument('--uids', type=int, nargs='*', default=None)
    args = ap.parse_args()
    out = pl.Path(args.out); vid = out / 'videos'; vid.mkdir(parents=True, exist_ok=True)
    files = {k: load_set(os.path.join(args.sets, v)) for k, v in SETS.items()}
    uids = args.uids or json.load(open(out / 'gallery_uids.json'))['uids']
    log = []
    for u in uids:
        assert u in files['human'] and u in files['machine'], f'uid {u} not in both sets'
        info = render(u, files['human'][u], files['machine'][u], vid / f'uid{u}_human_L_machine_R.mp4')
        log.append(info); print(info, flush=True)
    json.dump(log, open(vid / 'gallery_log.json', 'w'), indent=1)


if __name__ == '__main__':
    main()
