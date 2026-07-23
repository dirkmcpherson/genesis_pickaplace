"""Estimate the STATIC goal-can center from the real placement slides (user's method).

Idea: the goal can was static across all demos. In each demo the human slides the picked
can against the goal from some approach direction, so the picked can's final (contact)
center lands one can-diameter (0.066 m) from the goal center, on the approach side. Across
demos the contact points form a RING of radius=diameter around the goal. Two independent
estimators of the goal center:

  (P) project: goal_i = contact_i + diameter * approach_dir_i   (per-trial, then average)
  (C) circle-fit: fit a circle of FIXED radius=diameter to the contact points (direction-free)

Agreement between (P) and (C) is the confidence check. Human perception prunes trials with
a messy/ambiguous slide via --drop.

Data: inthewild_trials/<uid>_cartesian.npy tool_pose (m; cols3-5 deg) + gripper_pos (0..~85).
All in the Kinova base frame == sim frame, so the answer drops straight into the sim goal.

Usage: python can_pos_recovery/goal_from_slides.py [--drop 232 250 ...] [--plot out.png]
"""
import os
import argparse, json, pathlib as pl
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

REPO = pl.Path(__file__).resolve().parent.parent
DIAM = 0.066  # can diameter = 2*radius(0.033); two equal cans touching -> centers this far apart

ap = argparse.ArgumentParser()
ap.add_argument('--drop', type=int, nargs='*', default=[])
ap.add_argument('--only', type=int, nargs='*', default=None, help='restrict to these uids')
ap.add_argument('--plot', default='/home/j/workspace/genesis_pickaplace/can_pos_recovery/_scratch/goal_ring.png')
ap.add_argument('--win', type=int, default=30, help='frames before contact for approach dir')
# when imported (e.g. by goal_from_sim_slides for slide_contact), don't consume the
# importer's argv -> parse defaults; only read real CLI args when run as the main script.
args = ap.parse_args() if __name__ == '__main__' else ap.parse_args([])

trials = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['trials']
succ = sorted(int(u) for u, r in trials.items()
              if r.get('label') == 'success' and u not in ('290', '322'))
if args.only:
    succ = [u for u in succ if u in args.only]
drop = set(args.drop)

def _smooth(a, k=5):
    if len(a) < k:
        return a
    return np.convolve(a, np.ones(k) / k, mode='same')

def slide_contact(xy, gp, min_slide=0.006, x_min=0.55):
    """Core detector: given an xy trajectory (N,2) of the CAN CENTER (or tool proxy) and
    an aligned gripper signal gp (N,), find the placement slide = the last sustained
    horizontal motion of the held can that ends in the placement region and stops.
    Return (contact_xy, approach_unit_xy, quality dict) or None. Frame-agnostic: pass real
    tool_pose xy OR sim can-center xy."""
    xy = np.asarray(xy); gp = np.asarray(gp).ravel()
    thr = gp.min() + 0.4 * (gp.max() - gp.min())
    held = gp > thr
    if held.sum() < 25:
        return None
    idx = np.where(held)[0]
    brk = np.where(np.diff(idx) > 1)[0]
    start = idx[brk[-1] + 1] if len(brk) else idx[0]      # last contiguous held block
    block = np.arange(start, idx[-1] + 1)
    if len(block) < 25:
        return None
    P = xy[block]
    v = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))       # per-frame speed (len-1)
    vs = _smooth(v, 5)
    vmove = max(0.0008, 0.45 * np.percentile(vs, 85))      # adaptive movement threshold
    moving = vs > vmove
    # walk from the end: skip the settled tail, find the end of the last sustained slide
    i = len(vs) - 1
    while i >= 0 and not moving[i]:
        i -= 1
    if i < 0:
        return None
    slide_end = i                                          # last moving frame (in vs index)
    j = i
    while j >= 0 and moving[j]:
        j -= 1
    slide_start = j + 1
    c_idx, s_idx = slide_end + 1, slide_start              # positions in block
    contact = P[c_idx]
    disp = P[c_idx] - P[s_idx]
    dist = float(np.hypot(*disp))
    if dist < min_slide:                                   # slide too short -> ambiguous
        return None
    if contact[0] < x_min:                                 # not in placement region -> carry mis-detect
        return None
    u = disp / dist
    seg = P[s_idx:c_idx + 1]
    plen = float(np.sum(np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1]))))
    straight = dist / plen if plen > 0 else 0.0
    return contact, u, dict(approach_len=round(dist, 4), straight=round(straight, 2),
                            hold_frames=int(len(block)), slide_frames=int(c_idx - s_idx))

def contact_and_dir(uid):
    """Real-data wrapper: load a demo's tool_pose xy + gripper, run slide_contact."""
    d = np.load(REPO / f'inthewild_trials/{uid}_cartesian.npy', allow_pickle=True).item()
    tp = np.asarray(d['tool_pose']); gp = np.asarray(d['gripper_pos']).ravel()
    return slide_contact(tp[:, :2], gp)

rows = []
for uid in succ:
    if uid in drop:
        continue
    r = contact_and_dir(uid)
    if r is None:
        print(f"{uid}: SKIP (no clean hold)"); continue
    contact, u, q = r
    goal_p = contact + DIAM * u
    rows.append(dict(uid=uid, cx=float(contact[0]), cy=float(contact[1]),
                     ux=float(u[0]), uy=float(u[1]),
                     gx=float(goal_p[0]), gy=float(goal_p[1]), **q))

C = np.array([[r['cx'], r['cy']] for r in rows])          # contact points
G = np.array([[r['gx'], r['gy']] for r in rows])          # per-trial goal (project)

# (P) projection estimator
gp_mean = G.mean(0); gp_med = np.median(G, 0); gp_std = G.std(0)

# (C) fixed-radius circle fit: minimize sum( (|c_i - center| - DIAM)^2 ). Gauss-Newton.
center = C.mean(0).copy()
for _ in range(50):
    d = C - center; rn = np.hypot(d[:, 0], d[:, 1]) + 1e-9
    res = rn - DIAM
    J = -d / rn[:, None]                                   # d(res)/d(center)
    center = center - np.linalg.lstsq(J, res, rcond=None)[0]
fit_resid = float(np.sqrt(np.mean((np.hypot(*(C - center).T) - DIAM) ** 2)))

print(f"\n=== goal-from-slides over {len(rows)} trials (dropped {sorted(drop)}) ===")
print(f"(P) projection  mean=({gp_mean[0]:.4f},{gp_mean[1]:.4f})  median=({gp_med[0]:.4f},{gp_med[1]:.4f})  std=({gp_std[0]:.4f},{gp_std[1]:.4f})")
print(f"(C) circle-fit  center=({center[0]:.4f},{center[1]:.4f})  rms_resid={fit_resid*1000:.1f} mm")
print(f"    agreement |P.median - C| = {np.hypot(*(gp_med-center))*1000:.1f} mm")
print(f"    current candidates: A(0.656,-0.103)  B(0.704,-0.113)")
# per-trial, sorted by distance of goal_est from the circle-fit center (outliers last)
rows.sort(key=lambda r: np.hypot(r['gx'] - center[0], r['gy'] - center[1]))
print("\nuid   contact(x,y)     approach(ux,uy)  goal_est(x,y)    alen  straight  hold  dist_to_fit")
for r in rows:
    dd = np.hypot(r['gx'] - center[0], r['gy'] - center[1]) * 1000
    print(f"{r['uid']}  ({r['cx']:.3f},{r['cy']:.3f})  ({r['ux']:+.2f},{r['uy']:+.2f})  "
          f"({r['gx']:.3f},{r['gy']:.3f})  {r['approach_len']:.3f}  {r['straight']:.2f}   "
          f"{r['hold_frames']:4d}  {dd:5.1f}mm")

# plot: contact ring + goal_est cluster + fit
fig, ax = plt.subplots(figsize=(11, 10))
ax.scatter(C[:, 0], C[:, 1], c='steelblue', s=45, label='contact pts (picked-can center)', zorder=3)
for r in rows:  # approach arrows contact -> goal_est
    ax.annotate('', xy=(r['gx'], r['gy']), xytext=(r['cx'], r['cy']),
                arrowprops=dict(arrowstyle='->', color='orange', alpha=0.5, lw=1))
    ax.text(r['cx'], r['cy'], str(r['uid']), fontsize=6, color='navy')
ax.scatter(G[:, 0], G[:, 1], c='red', marker='x', s=35, label='goal_est = contact + diam*approach', zorder=4)
th = np.linspace(0, 2*np.pi, 100)
ax.plot(center[0] + DIAM*np.cos(th), center[1] + DIAM*np.sin(th), 'g--', alpha=0.7,
        label=f'fit circle r={DIAM} m')
ax.scatter([center[0]], [center[1]], c='green', marker='*', s=400,
           label=f'circle-fit goal ({center[0]:.3f},{center[1]:.3f})', zorder=6)
ax.scatter([gp_med[0]], [gp_med[1]], c='black', marker='*', s=250,
           label=f'projection-median ({gp_med[0]:.3f},{gp_med[1]:.3f})', zorder=6)
ax.scatter([0.656], [-0.103], c='cyan', marker='P', s=160, label='A (0.656,-0.103)', zorder=5)
ax.scatter([0.704], [-0.113], c='magenta', marker='P', s=160, label='B (0.704,-0.113)', zorder=5)
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_aspect('equal'); ax.grid(alpha=0.3)
ax.legend(fontsize=8, loc='upper left'); ax.set_title(f'goal from {len(rows)} placement slides')
plt.tight_layout(); plt.savefig(args.plot, dpi=90)
print(f"\nplot -> {args.plot}")

# save estimate
out = REPO / 'can_pos_recovery/goal_from_slides.json'
out.write_text(json.dumps(dict(
    n=len(rows), dropped=sorted(drop),
    projection_median=[float(gp_med[0]), float(gp_med[1])],
    projection_mean=[float(gp_mean[0]), float(gp_mean[1])],
    projection_std=[float(gp_std[0]), float(gp_std[1])],
    circle_fit=[float(center[0]), float(center[1])], circle_rms_mm=fit_resid*1000,
    trials={r['uid']: dict(contact=[r['cx'], r['cy']], goal_est=[r['gx'], r['gy']],
                           straight=r['straight'], approach_len=r['approach_len']) for r in rows},
), indent=2))
print(f"saved -> {out}")
