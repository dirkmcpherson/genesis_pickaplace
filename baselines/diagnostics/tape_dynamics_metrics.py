#!/usr/bin/env python3
"""Candidate WM-relevance metrics on the frozen matched demo sets (world x source).

MOTIVATION (2026-09-01): in the corrected world, EEF-position coverage of machine
demos is nearly human-level (-5%, fig6) and RLPD shows NO source effect (A20 null),
yet the world model (r2dreamer) still strongly prefers human demos (RESULTS 3.1).
This script screens a battery of tape-measurable properties for one that separates
dH from dDP in the CORRECTED world (where plain coverage does not), and ideally
more weakly in the old world (where coverage already separates).

DATA (read-only; images key never touched -- np.load is lazy):
  /cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_v2/{dH,dDP}   (old world, 56+56)
  /cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3/{dH,dDP}   (corrected world, 58+58)
  contract-v1 npz. states (T,17): joints 0:6, grip pos 6, grip effort 7, can xyz 8:11,
  can quat 11:15, goal xy 15:17 (verified empirically 2026-09-01: col6 0->0.6 on close,
  col7 effort 0->16, col8:11 = can at (0.478,0.106,0.11) rising on pick, col15:16 =
  (0.672,-0.221) = GOAL FINAL). actions_delta (T,7): 6 arm deltas + grip cmd.
  eef_pos (T+1,3). One tape per IC in every set (verified) -> per-IC multimodality is
  measured across NEAREST-NEIGHBOUR ICs, not within-IC.

PRE-DECLARED RANKING CRITERION (declared before any metric was computed; see
paper/WM_METRIC_2026-09-01.md): per-tape metrics -> signed Cohen's d (dH - dDP,
pooled SD) per world; criterion C = d_w3 - d_old (we want w3-specific separators).
Set-level metrics (single number per set) -> bootstrap-z (B=1000 over tapes),
criterion C_z = z_w3 - z_old, reported in a SEPARATE table (z is not Cohen's d).
ALL metrics computed are reported -- no cherry-picking. This is a screen of ~30
metrics on one dataset: the winner is hypothesis-generating, not confirmatory.

Run (cluster login node, CPU, ~1 min):
  nice -n 19 python baselines/diagnostics/tape_dynamics_metrics.py <out_dir>
Outputs: tape_dyn_metrics.csv (per-tape), set_level_metrics.csv, ranking.md.
"""
import csv, math, sys
import numpy as np
import pathlib as pl
from collections import Counter, defaultdict

ROOT = pl.Path('/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines')
SETS = [('old', 'dH', ROOT / 'matched_v2' / 'dH'), ('old', 'dDP', ROOT / 'matched_v2' / 'dDP'),
        ('w3', 'dH', ROOT / 'matched_w3' / 'dH'), ('w3', 'dDP', ROOT / 'matched_w3' / 'dDP')]
VOX = 0.02          # m, EEF voxel (same bin as fig6)
DEAD_V = 5e-4       # m/step deadband for Delta-eef direction octants
SPEED_EDGES = np.array([0.0, 0.002, 0.005, 0.010, np.inf])  # m/step
CAN_EPS = 1e-3      # m/step: "can is moving"
GRIP_CLOSED = 0.3   # states[:,6] threshold (range 0..~0.6)
RNG = np.random.default_rng(0)
NPERM, NBOOT = 10000, 1000


def octant(v):
    """Direction octant with deadband: per-component sign in {-1,0,1} -> int code."""
    s = (np.where(v > DEAD_V, 1, np.where(v < -DEAD_V, -1, 0)) + 1)
    return s[:, 0] * 9 + s[:, 1] * 3 + s[:, 2]


def runs(mask, min_len=2):
    """Number of contiguous True runs of length >= min_len."""
    n = cur = 0
    for m in mask:
        cur = cur + 1 if m else 0
        if cur == min_len:
            n += 1
    return n


def autocorr1(x):
    x = x - x.mean()
    v = (x * x).sum()
    return float((x[:-1] * x[1:]).sum() / v) if v > 1e-12 else np.nan


def hf_frac(x):
    """Fraction of non-DC spectral power above Nyquist/2 (freq index > n_rfft/2)."""
    p = np.abs(np.fft.rfft(x - x.mean())) ** 2
    p = p[1:]
    return float(p[len(p) // 2:].sum() / p.sum()) if p.sum() > 1e-12 else np.nan


def perm_entropy(x, order=3):
    """Normalized permutation entropy of a 1-D signal."""
    if len(x) < order + 1:
        return np.nan
    idx = np.array([np.argsort(x[i:i + order]) for i in range(len(x) - order + 1)])
    codes = (idx * (order ** np.arange(order))).sum(axis=1)
    c = np.bincount(codes)
    p = c[c > 0] / c.sum()
    return float(-(p * np.log2(p)).sum() / math.log2(math.factorial(order)))


def entropy_bits(counter):
    tot = sum(counter.values())
    p = np.array([v / tot for v in counter.values()])
    return float(-(p * np.log2(p)).sum())


def resample20(eef):
    steps = np.linalg.norm(np.diff(eef, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(steps)])
    s /= (s[-1] or 1)
    tq = np.linspace(0, 1, 20)
    p = np.stack([np.interp(tq, s, eef[:, i]) for i in range(3)], axis=1)
    return p - p[0]  # start-aligned (fig6 convention)


def tape_metrics(z):
    """Per-tape metric dict. Every metric here enters the ranking table."""
    eef = z['eef_pos'].astype(np.float64)
    st = z['states'].astype(np.float64)
    act = z['actions_delta'].astype(np.float64)
    T = st.shape[0]
    q, grip, effort, can = st[:, :6], st[:, 6], st[:, 7], st[:, 8:11]
    v = np.diff(eef, axis=0)                       # (T,3)
    speed = np.linalg.norm(v, axis=1)
    acc = np.diff(v, axis=0); jrk = np.diff(acc, axis=0)
    path_len = float(speed.sum())
    net = float(np.linalg.norm(eef[-1] - eef[0]))
    dq = np.diff(q, axis=0)
    m = dict(n_rows=T, path_len=round(path_len, 4),
             tortuosity=round(path_len / net, 3) if net > 1e-6 else np.nan)
    # chord-wander (fig6 continuity)
    a, b = eef[0], eef[-1]; ab = b - a; L2 = float(ab @ ab) or 1e-12
    t = np.clip(((eef - a) @ ab) / L2, 0, 1)[:, None]
    m['wander'] = round(float(np.linalg.norm(eef - (a + t * ab), axis=1).mean()), 4)
    # (b) velocity / acceleration / jerk
    m['speed_mean'] = float(speed.mean()); m['speed_std'] = float(speed.std())
    m['speed_cv'] = float(speed.std() / speed.mean()) if speed.mean() > 1e-9 else np.nan
    m['accel_mean'] = float(np.linalg.norm(acc, axis=1).mean())
    m['jerk_mean'] = float(np.linalg.norm(jrk, axis=1).mean())
    m['jerk_per_len'] = float(np.linalg.norm(jrk, axis=1).sum() / path_len) if path_len > 1e-9 else np.nan
    m['pause_frac'] = float((speed < 0.2 * np.median(speed)).mean())
    m['joint_speed_mean'] = float(np.linalg.norm(dq, axis=1).mean())
    m['joint_jerk_mean'] = float(np.linalg.norm(np.diff(dq, 2, axis=0), axis=1).mean()) if T > 3 else np.nan
    # (e) temporal structure of the ACTION stream (6 arm dims)
    m['act_autocorr1'] = float(np.nanmean([autocorr1(act[:, i]) for i in range(6)]))
    m['act_hf_frac'] = float(np.nanmean([hf_frac(act[:, i]) for i in range(6)]))
    sd = act[:, :6].std(axis=0)
    flips = [float((np.diff(np.sign(np.where(np.abs(act[:, i]) < 0.05 * sd[i], 0, act[:, i]))) != 0).mean())
             for i in range(6) if sd[i] > 1e-9]
    m['act_signflip_rate'] = float(np.mean(flips)) if flips else np.nan
    m['speed_perm_entropy'] = perm_entropy(speed)
    m['act_mag_mean'] = float(np.abs(act[:, :6]).mean())
    # (c) contact-phase richness
    dcan = np.linalg.norm(np.diff(can, axis=0), axis=1)
    moving = dcan > CAN_EPS
    closed = grip > GRIP_CLOSED
    m['can_path_len'] = float(dcan.sum())
    m['can_moved_frac'] = float(moving.mean())
    m['contact_events'] = runs(moving, 2)
    m['can_move_open_frac'] = float((moving & ~closed[1:]).mean())   # pre/post-grasp nudges
    m['grip_cmd_flip'] = int((np.diff(np.sign(act[:, 6])) != 0).sum())
    closes = int(((~closed[:-1]) & closed[1:]).sum())
    m['grip_close_events'] = closes                                   # >1 => regrasp-like
    m['effort_std'] = float(effort.std())
    zax = np.stack([2 * (st[:, 12] * st[:, 14] + st[:, 11] * st[:, 13]),
                    2 * (st[:, 13] * st[:, 14] - st[:, 11] * st[:, 12]),
                    1 - 2 * (st[:, 12] ** 2 + st[:, 13] ** 2)], axis=1)  # can z-axis from quat(w,x,y,z)
    tilt = np.degrees(np.arccos(np.clip(zax[:, 2], -1, 1)))
    m['tilt_max'] = float(tilt.max())
    m['tilt_inhand_mean'] = float(tilt[closed].mean()) if closed.any() else np.nan
    # (f) recovery / correction segments
    d_ec = np.linalg.norm(eef[:T] - can, axis=1)
    k = np.ones(5) / 5
    ds = np.convolve(d_ec, k, mode='valid')
    first_close = int(np.argmax(closed)) if closed.any() else T
    pre = np.diff(ds[:max(first_close - 4, 2)])
    m['retreat_frac'] = float((pre > 5e-4).mean()) if len(pre) else np.nan
    m['approach_reversals'] = runs(pre > 5e-4, 3) if len(pre) else np.nan
    if T > 6:
        vv = v[5:]; back = (vv * (eef[5:-1] - eef[:T - 5])).sum(axis=1)
        m['backtrack_frac'] = float((back < 0).mean())
    else:
        m['backtrack_frac'] = np.nan
    # (a) per-tape transition diversity
    oc = octant(v); sb = np.digitize(speed, SPEED_EDGES) - 1
    kinds = Counter(zip(oc.tolist(), sb.tolist()))
    m['trans_kinds'] = len(kinds)
    m['trans_entropy'] = entropy_bits(kinds)
    return m


# ---------------------------------------------------------------- load all tapes
all_rows, cache = [], {}
for world, arm, d in SETS:
    files = sorted(d.glob('*.npz'))
    print(f'{world}/{arm}: {len(files)} tapes', flush=True)
    for f in files:
        z = np.load(f, allow_pickle=True)
        r = dict(world=world, arm=arm, uid=int(z['uid']), ic_uid=int(z['ic_uid']),
                 stage=str(z['stage']))
        r.update(tape_metrics(z))
        eef = z['eef_pos'].astype(np.float64)
        st = z['states'].astype(np.float64)
        cache[(world, arm, int(z['ic_uid']))] = dict(
            eef=eef, v=np.diff(eef, axis=0), q=st[:, :6], dq=np.diff(st[:, :6], axis=0),
            act=z['actions_delta'].astype(np.float64)[:, :6], can0=st[0, 8:10],
            r20=resample20(eef))
        all_rows.append(r)

# per-tape metric needing set context: nearest-neighbour-IC strategy diversity (d)
for world, arm, _ in SETS:
    keys = [k for k in cache if k[0] == world and k[1] == arm]
    can0 = np.array([cache[k]['can0'] for k in keys])
    for i, k in enumerate(keys):
        dist_ic = np.linalg.norm(can0 - can0[i], axis=1)
        nn = np.argsort(dist_ic)[1:4]  # 3 nearest OTHER ICs
        dv = [np.linalg.norm(cache[keys[j]]['r20'] - cache[k]['r20'], axis=1).mean() for j in nn]
        row = next(r for r in all_rows if (r['world'], r['arm'], r['ic_uid']) == k)
        row['nnIC_path_div'] = float(np.mean(dv))

METRICS = [k for k in all_rows[0] if k not in ('world', 'arm', 'uid', 'ic_uid', 'stage')]


# ---------------------------------------------------------------- set-level metrics
def set_level(keys):
    """Coverage/diversity numbers that only exist pooled over a set of tapes."""
    vox_e, vox_te, vox_qdq = set(), set(), set()
    cond = defaultdict(Counter)          # eef voxel -> octant counts
    actv = defaultdict(list)             # eef voxel -> arm action rows
    for k in keys:
        c = cache[k]
        ev = np.floor(c['eef'] / VOX).astype(int)
        vox_e.update(map(tuple, ev))
        oc = octant(c['v']); sb = np.digitize(np.linalg.norm(c['v'], axis=1), SPEED_EDGES) - 1
        for t in range(len(oc)):
            key = tuple(ev[t])
            vox_te.add(key + (int(oc[t]), int(sb[t])))
            cond[key][int(oc[t])] += 1
            actv[key].append(c['act'][t])
        qv = np.round(c['q'] / 0.1).astype(int)
        sg = np.where(c['dq'][:, :3] > 2e-3, 1, np.where(c['dq'][:, :3] < -2e-3, -1, 0))
        for t in range(len(sg)):
            vox_qdq.add(tuple(qv[t]) + tuple(sg[t]))
    ents, wts = [], []
    avars = []
    for key, cnt in cond.items():
        n = sum(cnt.values())
        if n >= 5:
            ents.append(entropy_bits(cnt)); wts.append(n)
            avars.append(np.stack(actv[key]).var(axis=0).mean())
    return dict(cov_eef=len(vox_e), cov_eef_trans=len(vox_te), cov_q_dq=len(vox_qdq),
                cond_doct_entropy=float(np.average(ents, weights=wts)),
                cond_act_var=float(np.average(avars, weights=wts)))


SET_METRICS = ['cov_eef', 'cov_eef_trans', 'cov_q_dq', 'cond_doct_entropy', 'cond_act_var']
set_vals, set_boot = {}, {}
for world, arm, _ in SETS:
    keys = [k for k in cache if k[0] == world and k[1] == arm]
    set_vals[(world, arm)] = set_level(keys)
    boots = {mk: [] for mk in SET_METRICS}
    for _ in range(NBOOT):
        bk = [keys[i] for i in RNG.integers(0, len(keys), len(keys))]
        sv = set_level(bk)
        for mk in SET_METRICS:
            boots[mk].append(sv[mk])
    set_boot[(world, arm)] = {mk: float(np.std(boots[mk])) for mk in SET_METRICS}
    print(f'set-level {world}/{arm} done', flush=True)


# ---------------------------------------------------------------- stats + ranking
def col(w, a, mk):
    x = np.array([r.get(mk, np.nan) for r in all_rows if r['world'] == w and r['arm'] == a], float)
    return x[~np.isnan(x)]


def cohens_d(x, y):
    nx, ny = len(x), len(y)
    sp = math.sqrt(((nx - 1) * x.var(ddof=1) + (ny - 1) * y.var(ddof=1)) / (nx + ny - 2))
    return (x.mean() - y.mean()) / sp if sp > 1e-12 else 0.0


def perm_p(x, y):
    obs = abs(x.mean() - y.mean())
    z = np.concatenate([x, y]); n = len(x); hits = 0
    for _ in range(NPERM):
        RNG.shuffle(z)
        if abs(z[:n].mean() - z[n:].mean()) >= obs - 1e-15:
            hits += 1
    return hits / NPERM


rank = []
for mk in METRICS:
    row = dict(metric=mk)
    for w in ('old', 'w3'):
        h, dp = col(w, 'dH', mk), col(w, 'dDP', mk)
        row[f'mean_dH_{w}'], row[f'mean_dDP_{w}'] = h.mean(), dp.mean()
        row[f'd_{w}'] = cohens_d(h, dp)
        if w == 'w3':
            row['p_w3'] = perm_p(h, dp)
    row['criterion'] = row['d_w3'] - row['d_old']
    rank.append(row)
rank.sort(key=lambda r: -r['criterion'])

set_rank = []
for mk in SET_METRICS:
    row = dict(metric=mk)
    for w in ('old', 'w3'):
        vh, vd = set_vals[(w, 'dH')][mk], set_vals[(w, 'dDP')][mk]
        se = math.sqrt(set_boot[(w, 'dH')][mk] ** 2 + set_boot[(w, 'dDP')][mk] ** 2)
        row[f'dH_{w}'], row[f'dDP_{w}'] = vh, vd
        row[f'z_{w}'] = (vh - vd) / se if se > 1e-12 else 0.0
    row['criterion_z'] = row['z_w3'] - row['z_old']
    set_rank.append(row)
set_rank.sort(key=lambda r: -r['criterion_z'])

# ---------------------------------------------------------------- write outputs
out = pl.Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp')
out.mkdir(parents=True, exist_ok=True)
fields = ['world', 'arm', 'uid', 'ic_uid', 'stage'] + METRICS
with open(out / 'tape_dyn_metrics.csv', 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=fields); w.writeheader()
    for r in all_rows:
        w.writerow({k: r.get(k, '') for k in fields})
with open(out / 'set_level_metrics.csv', 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(set_rank[0].keys())); w.writeheader(); w.writerows(set_rank)

with open(out / 'ranking.md', 'w') as fh:
    def emit(s):
        print(s); fh.write(s + '\n')
    emit('## Per-tape metrics, ranked by pre-declared criterion C = d_w3 - d_old (Cohen\'s d, dH-dDP)')
    emit('| metric | dH_w3 | dDP_w3 | d_w3 | p_w3 | dH_old | dDP_old | d_old | C |')
    emit('|---|---|---|---|---|---|---|---|---|')
    for r in rank:
        emit(f"| {r['metric']} | {r['mean_dH_w3']:.4g} | {r['mean_dDP_w3']:.4g} | {r['d_w3']:+.2f} | "
             f"{r['p_w3']:.3f} | {r['mean_dH_old']:.4g} | {r['mean_dDP_old']:.4g} | {r['d_old']:+.2f} | {r['criterion']:+.2f} |")
    emit('')
    emit('## Set-level metrics (bootstrap z, B=1000 over tapes; NOT comparable to Cohen\'s d above)')
    emit('| metric | dH_w3 | dDP_w3 | z_w3 | dH_old | dDP_old | z_old | C_z |')
    emit('|---|---|---|---|---|---|---|---|')
    for r in set_rank:
        emit(f"| {r['metric']} | {r['dH_w3']:.4g} | {r['dDP_w3']:.4g} | {r['z_w3']:+.2f} | "
             f"{r['dH_old']:.4g} | {r['dDP_old']:.4g} | {r['z_old']:+.2f} | {r['criterion_z']:+.2f} |")
print(f'\nwrote {out}/tape_dyn_metrics.csv, set_level_metrics.csv, ranking.md')
