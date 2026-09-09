"""Render all metric passes from measured poses in one unchanged December18 world."""
import hashlib
import json
import os
from pathlib import Path
import sys

root = Path(__file__).resolve().parent
repo = root.parents[2]
sys.path[:0] = [str(repo / 'baselines'), str(repo / 'can_pos_recovery')]
os.environ['TI_CPU_MAX_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
os.environ.setdefault('TI_OFFLINE_CACHE_FILE_PATH', '/tmp/eef-recovery-ti')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/eef-recovery-mpl')

import cv2
import numpy as np
from genesis_can_env import GenesisCanEnv
from sim_variant_hook import apply_pre, apply_post

rows = json.loads((root / 'dec18_timestamp/results.json').read_text())['records']
rows = [r for r in rows if r['metric']['slide_success']]
variant = 'gc_kp4_riser3_shelf6'
os.environ['GENESIS_SIM_VARIANT'] = variant
apply_pre(variant)
env = GenesisCanEnv(backend='cpu', render_size=640, max_steps=10**9)
apply_post(env, variant)
w = env.w
for record in rows:
    uid = record['uid']
    source = Path(record['source'])
    assert hashlib.sha256(source.read_bytes()).hexdigest() == record['source_sha256']
    meta = json.loads(source.with_suffix('.json').read_text())
    assert meta['variant'] == variant and meta['decision_dt_s'] == .03
    destination = root / 'batch_visuals' / str(uid)
    if (destination / 'rendered.json').exists():
        old = json.loads((destination / 'rendered.json').read_text())
        assert old['source_sha256'] == record['source_sha256']
        continue
    destination.mkdir(parents=True, exist_ok=True)
    video = destination / f'{uid}_timestamp.mp4'
    if video.exists():
        raise RuntimeError(f'Inspect partial video {video}')
    with np.load(source) as data:
        trajectory = data['trajectory']
        indices = np.arange(0, len(trajectory), 3)
        if indices[-1] != len(trajectory) - 1:
            indices = np.r_[indices, len(trajectory) - 1]
        selected = {int(round(f * (len(indices) - 1))) for f in [0, .3, .55, .72, .88, 1]}
        pictures, sampled = [], []
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'mp4v'), 1/.09, (640, 640))
        assert writer.isOpened()
        try:
            for frame, i in enumerate(indices):
                row = trajectory[i]
                w['kinova'].set_dofs_position(np.r_[row[:6], data['finger_joint'][i]], w['kdofs'])
                w['bottle'].set_pos(row[13:16]); w['bottle'].set_quat(row[16:20])
                goal = data['goal_pose'][i]
                w['goal'].set_pos(goal[:3]); w['goal'].set_quat(goal[3:])
                w['scene']._visualizer._t = -1
                w['cam']._rasterizer._context._t = -1
                image = np.asarray(w['cam'].render()[0])[:, :, ::-1].copy()
                contacts = data['contact_counts'][i]
                dist = np.linalg.norm(row[13:15] - goal[:2])
                labels = [f'{uid} | measured poses | {(i+1)*.03:.2f}s',
                          f'center distance {dist*1000:.1f} mm | tilt {row[20]:.1f} deg',
                          f'contacts shelf/hand/goal {contacts.tolist()}']
                for j, label in enumerate(labels):
                    cv2.putText(image, label, (10, 25+25*j), 0, .52, (0,0,0), 3)
                    cv2.putText(image, label, (10, 25+25*j), 0, .52, (255,255,255), 1)
                writer.write(image)
                if frame in selected:
                    pictures.append(cv2.resize(image, (480,480)))
                    sampled.append(int(i))
        finally:
            writer.release()
        assert len(pictures) == 6
        cv2.imwrite(str(destination / 'phases.jpg'), np.vstack([np.hstack(pictures[:3]), np.hstack(pictures[3:])]))
        (destination / 'rendered.json').write_text(json.dumps(dict(uid=uid,
            source_sha256=record['source_sha256'], sampled_trace_frames=sampled,
            render='Saved measured poses; physics is not stepped; one shared world with identical variant',
            visual_review_completed=False,
            video_cadence='Stride 3 at 11.111 fps; final source frame appended if needed, giving at most 60 ms endpoint display overrun'), indent=2))
        print('RENDERED', uid, flush=True)
