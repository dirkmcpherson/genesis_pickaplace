"""Render saved measured poses without rerunning contact dynamics.

This is a visualization of the probe trace, not an independent physics replay.
"""
import argparse
import json
import os
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / 'baselines'), str(REPO / 'can_pos_recovery')]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('trace', type=Path)
    p.add_argument('--start', type=int, default=0)
    p.add_argument('--stop', type=int)
    p.add_argument('--stride', type=int, default=3)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    if a.out.exists():
        raise FileExistsError(a.out)
    os.environ['TI_CPU_MAX_NUM_THREADS'] = '1'
    os.environ['OMP_NUM_THREADS'] = '1'
    os.environ.setdefault('TI_OFFLINE_CACHE_FILE_PATH', '/tmp/eef-recovery-ti')
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/eef-recovery-mpl')
    os.environ.setdefault('NUMBA_CACHE_DIR', '/tmp/eef-recovery-numba')
    import numpy as np
    import cv2
    from sim_variant_hook import apply_pre, apply_post
    from genesis_can_env import GenesisCanEnv

    metadata = json.loads(a.trace.with_suffix('.json').read_text())
    data = np.load(a.trace)
    variant = metadata['variant']
    os.environ['GENESIS_SIM_VARIANT'] = variant
    apply_pre(variant)
    env = GenesisCanEnv(backend='cpu', render_size=640, max_steps=10**9)
    if metadata.get('variant_post_hook', True):
        apply_post(env, variant)
    w = env.w
    a.out.parent.mkdir(parents=True, exist_ok=True)
    writer = None
    stop = min(a.stop or len(data['trajectory']), len(data['trajectory']))
    try:
        for i in range(a.start, stop, a.stride):
            row = data['trajectory'][i]
            w['kinova'].set_dofs_position(np.r_[row[:6], data['finger_joint'][i]], w['kdofs'])
            w['bottle'].set_pos(row[13:16]); w['bottle'].set_quat(row[16:20])
            goal = data['goal_pose'][i]
            w['goal'].set_pos(goal[:3]); w['goal'].set_quat(goal[3:])
            # Genesis caches visual transforms by physics step. This renderer
            # never steps physics, so invalidate both visualization caches.
            w['scene']._visualizer._t = -1
            w['cam']._rasterizer._context._t = -1
            frame = np.asarray(w['cam'].render()[0])[:, :, ::-1].copy()
            contacts = data['contact_counts'][i]
            labels = [f'{metadata["uid"]} {metadata["mode"]} | saved measured poses',
                      f'tape frame {i} | sim time {(i+1)*.03:.2f} s',
                      f'contacts: shelf {contacts[0]}  robot {contacts[1]}  goal {contacts[2]}']
            if 'extension_m' in metadata:
                labels.append(f'Added slide motion: {1000*metadata["extension_m"]:.1f} mm')
            for k, label in enumerate(labels):
                cv2.putText(frame, label, (10, 24 + k*23), cv2.FONT_HERSHEY_SIMPLEX, .55,
                            (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(frame, label, (10, 24 + k*23), cv2.FONT_HERSHEY_SIMPLEX, .55,
                            (255, 255, 255), 1, cv2.LINE_AA)
            if writer is None:
                writer = cv2.VideoWriter(str(a.out), cv2.VideoWriter_fourcc(*'mp4v'),
                                         1/(.03*a.stride), (frame.shape[1], frame.shape[0]))
                if not writer.isOpened():
                    raise RuntimeError('Video writer failed')
            writer.write(frame)
            if (i-a.start) % (a.stride*30) == 0:
                print('RENDER', i, '/', stop, flush=True)
    finally:
        if writer is not None:
            writer.release()


if __name__ == '__main__':
    main()
