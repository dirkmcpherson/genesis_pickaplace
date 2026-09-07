"""Discrete-action replay check (paper/DISCRETE_ACTION_REPLAY_2026-09-06.md).

Question: do the operator's joystick commands, snapped to a discrete code, still drive the
simulator through the demonstrations? We replay the RECORDED Cartesian twist command
(`inthewild_trials/<uid>_cartesian.npy`, key `cartesian_velocity`, 40 Hz frames, live axes
x y z at cap 0.11 m/s + wrist pitch at cap 1.0 rad/s) through the commanded-Cartesian path
of record (baselines/cartesian_env.CartesianCanEnv, control='vel': setpoint integration at
DT 0.025 + IK + GenesisCanEnv joint PD, 3 physics steps per frame) in the paper world
gc_kp4_riser3_shelf6 (sim_variant_hook pre/post, exactly as the recorder), from the
recovered can placement of record (GenesisCanEnv.reset(uid=...)), and score the funnel
picked / placed / contact / nested (+ set-down, tipped) with the harness's own predicates.

Codes (arm = 4 axes, grip = gripper motor 0..100 command fed to gripper_targets):
  raw          arm: recorded twist                     grip: recorded gripper_pos
  tern         arm: |v| >= 0.5 cap -> +-cap else 0      grip: ternary ramp (below)
  five         arm: nearest of {0, +-0.5, +-1} x cap    grip: ternary ramp
  held         arm: tern, then word-level mode filter   grip: ternary ramp
               over a 5-frame window (+-2; ties keep the centre word) -- removes step-to-step
               jitter; non-causal by 2 frames (offline tape transform, disclosed)
  tern_rawgrip arm: tern                                grip: recorded (isolates the arm code)
  raw_terngrip arm: recorded                            grip: ternary ramp (isolates the grip code)
Gripper ternary ramp: the real gripper is a button whose position ramps at ~1.5 units/frame
(median |dgp| on the 74 tapes: up 1.49, down 1.52) and stops where the button is released
(closed plateaus 37..88, partial closes are common) -- i.e. a three-valued command
u in {open, hold, close} driving a fixed-rate ramp g <- clip(g + 1.5 u, 0, 100). The
harness has no gripper bit, so this script builds the ramp: u_t = sign(gp_t - g_{t-1}) if
|gp_t - g_{t-1}| > 0.75 else 0 (a tracking encoder of the recorded position; g_0 = gp_0).

  run   --code C --shard-idx i --shard-n n --outdir D [--threads 1] [--uids ...]
  vocab --outdir D                 vocabulary stats (distinct ternary words, top-16 coverage)
  table --outdir D [--ref raw]     funnel table + per-uid flips (markdown to stdout + json)
"""
import os
import sys
import json
import time
import argparse
import pathlib as pl
from collections import Counter

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'can_pos_recovery'))

VCAP, PCAP = 0.11, 1.0            # cartesian_env.CartesianCanEnv.VCAP / PITCH_CAP (asserted at run)
GRIP_RATE = 1.5                   # gripper units per frame (0..100 scale), measured median ramp
CODES = ('raw', 'tern', 'five', 'held', 'tern_rawgrip', 'raw_terngrip')
STAGES = ('picked', 'placed', 'contact', 'nested')
TIP_DEG = 60.0                    # rl/full_env.FullTaskEnv.TIP_DEG
GRIP_OPEN = 0.3                   # rl/full_env.FullTaskEnv.GRIP_OPEN (grip cmd below = released)
VARIANT = 'gc_kp4_riser3_shelf6'


# ---------------------------------------------------------------- codes (pure functions)
def success_uids():
    tbl = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['trials']
    return sorted(int(k) for k, r in tbl.items()
                  if r.get('label') == 'success' and r.get('status') in ('ok', 'ok_batch'))


def load_tape(uid):
    d = np.load(REPO / f'inthewild_trials/{uid}_cartesian.npy', allow_pickle=True).item()
    cv = np.asarray(d['cartesian_velocity'], np.float64)
    gp = np.asarray(d['gripper_pos'], np.float64)[:, 0]
    arm = np.stack([cv[:, 0], cv[:, 1], cv[:, 2], cv[:, 4]], 1)     # x y z pitch (physical)
    assert np.all(np.abs(cv[:, 3]) < 1e-9) and np.all(np.abs(cv[:, 5]) < 1e-9), uid   # roll/yaw never commanded
    return arm, gp


def normalize(arm):
    return arm / np.array([VCAP, VCAP, VCAP, PCAP])


def tern_words(arm):
    """(n,4) int in {-1,0,1}: |v| >= 0.5 cap -> sign, else 0."""
    a = normalize(arm)
    return np.where(np.abs(a) >= 0.5, np.sign(a), 0).astype(int)


def five_levels(arm):
    """(n,4) in {-1,-.5,0,.5,1}: nearest of the five levels per axis."""
    a = np.clip(normalize(arm), -1, 1)
    return np.round(a * 2.0) / 2.0


def held_words(words, half=2):
    """Word-level mode filter: at each frame the most common 4-axis word in [t-half, t+half];
    ties keep the centre frame's word."""
    n = len(words)
    out = words.copy()
    keys = [tuple(w) for w in words]
    for t in range(n):
        lo, hi = max(0, t - half), min(n, t + half + 1)
        c = Counter(keys[lo:hi])
        best = max(c.values())
        if c[keys[t]] < best:
            out[t] = np.array(next(k for k, v in c.items() if v == best))   # first max in window order
    return out


def grip_ramp(gp, rate=GRIP_RATE):
    """Ternary gripper command u_t in {-1,0,1} tracking the recorded position, and the ramp
    it drives (0..100). g_0 = gp_0 (the initial state, like the can IC)."""
    g = np.empty_like(gp); u = np.zeros(len(gp), int)
    g[0] = gp[0]
    for t in range(1, len(gp)):
        d = gp[t] - g[t - 1]
        u[t] = 0 if abs(d) <= rate / 2 else int(np.sign(d))
        g[t] = float(np.clip(g[t - 1] + rate * u[t], 0.0, 100.0))
    return u, g


def encode(arm, gp, code):
    """-> (arm_phys (n,4), grip_cmd (n,) 0..100, words_or_None)"""
    assert code in CODES, code
    cap = np.array([VCAP, VCAP, VCAP, PCAP])
    if code in ('raw', 'raw_terngrip'):
        arm_out = arm.copy(); words = None
    elif code in ('tern', 'tern_rawgrip'):
        words = tern_words(arm); arm_out = words * cap
    elif code == 'five':
        words = five_levels(arm); arm_out = words * cap
    elif code == 'held':
        words = held_words(tern_words(arm)); arm_out = words * cap
    if code in ('raw', 'tern_rawgrip'):
        grip = gp.copy()
    else:
        _, grip = grip_ramp(gp)
    return arm_out, grip, words


# ---------------------------------------------------------------- run (one process, one shard)
def _patch_threads(n):
    os.environ.setdefault('OMP_NUM_THREADS', str(n)); os.environ.setdefault('MKL_NUM_THREADS', str(n))
    import taichi as ti
    orig = ti.init

    def init(*a, **k):
        k.setdefault('cpu_max_num_threads', int(n))
        return orig(*a, **k)
    ti.init = init
    import torch
    torch.set_num_threads(int(n))


def np_(x):
    import torch
    return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)


def run(a):
    if a.threads > 0:
        _patch_threads(a.threads)
    from sim_variant_hook import apply_pre, apply_post
    import sim_variants
    from replay_harness import tilt_deg, in_shelf_footprint, NESTED_TOUCH_DIST, BOTTLE_HEIGHT
    apply_pre(a.variant)
    from cartesian_env import CartesianCanEnv
    assert CartesianCanEnv.VCAP == VCAP and CartesianCanEnv.PITCH_CAP == PCAP
    env = CartesianCanEnv(backend='cpu', max_steps=10 ** 9, control='vel')
    vinfo = apply_post(env, a.variant)
    w = env.w
    shelf_top = sim_variants.shelf_top(a.variant); rest_z = shelf_top + BOTTLE_HEIGHT / 2
    uids = a.uids or success_uids()
    uids = uids[a.shard_idx::a.shard_n]
    out = pl.Path(a.outdir) / a.code; out.mkdir(parents=True, exist_ok=True)
    print(f'[run] code={a.code} variant={a.variant} shard {a.shard_idx}/{a.shard_n} uids={uids} threads={a.threads} '
          f'shelf_top={shelf_top:.3f} goal_start_z={w["goal_start_z"]:.4f} pick_z={w["pick_z"]:.4f}', flush=True)
    for uid in uids:
        arm, gp = load_tape(uid)
        arm_c, grip_c, words = encode(arm, gp, a.code)
        n = len(arm_c)
        t0 = time.time()
        env.reset(uid=uid)
        first = {}; tilt_max = 0.0; tip_free_first = None; tip_any_first = None
        placed_shift = setdown = False
        traj = np.zeros((n, 8), np.float32)   # can xyz, can tilt, tool xyz, grip cmd
        for i in range(n):
            act = np.array([arm_c[i, 0], arm_c[i, 1], arm_c[i, 2], arm_c[i, 3],
                            float(np.clip(grip_c[i] / 100.0, 0.0, 1.0))], np.float64)
            obs, done, info = env.step(act)
            bp = np_(w['bottle'].get_pos()).astype(float); tl = tilt_deg(np_(w['bottle'].get_quat()))
            tool = env._tool_pos()
            traj[i, :3] = bp; traj[i, 3] = tl; traj[i, 4:7] = tool; traj[i, 7] = act[4]
            tilt_max = max(tilt_max, tl)
            if info['picked'] and 'picked' not in first: first['picked'] = i
            if info['contact'] and 'contact' not in first: first['contact'] = i
            if info['placed'] and 'placed_env' not in first: first['placed_env'] = i
            if info['picked'] and in_shelf_footprint(bp) and shelf_top + 0.01 < bp[2] < shelf_top + 0.07:
                if not placed_shift: first['placed'] = i
                placed_shift = True
                if act[4] < 0.5 and abs(bp[2] - rest_z) < 0.015 and tl < 20:
                    if not setdown: first['setdown'] = i
                    setdown = True
            if tl > TIP_DEG:
                if tip_any_first is None: tip_any_first = i
                if act[4] < GRIP_OPEN and tip_free_first is None: tip_free_first = i
        nested = bool(env.env._nested())            # 100 settle steps; eval rule of record
        bp = np_(w['bottle'].get_pos()).astype(float); gq = np_(w['goal'].get_pos()).astype(float)
        can_tilt = float(tilt_deg(np_(w['bottle'].get_quat()))); goal_tilt = float(tilt_deg(np_(w['goal'].get_quat())))
        dist = float(np.hypot(bp[0] - gq[0], bp[1] - gq[1]))
        rec = dict(uid=int(uid), code=a.code, variant=a.variant, n_frames=int(n),
                   picked=bool(env.env._picked), placed=bool(placed_shift), placed_env=bool(env.env._placed),
                   setdown=bool(setdown), contact=bool(env.env._contact), nested=nested,
                   tipped=bool(can_tilt > TIP_DEG), tip_any_first=tip_any_first, tip_free_first=tip_free_first,
                   tilt_max=round(tilt_max, 1), can_tilt_settled=round(can_tilt, 1), goal_tilt_settled=round(goal_tilt, 1),
                   dist_settled=round(dist, 4), touch_dist=float(NESTED_TOUCH_DIST),
                   can_end=[round(float(v), 4) for v in bp], goal_end=[round(float(v), 4) for v in gq],
                   first={k: int(v) for k, v in first.items()},
                   n_words_used=(int(len({tuple(x) for x in words})) if words is not None else None),
                   frac_active=(float(np.mean(np.any(words != 0, 1))) if words is not None else None),
                   grip_ramp_maxdev=(float(np.max(np.abs(grip_c - gp))) if a.code not in ('raw', 'tern_rawgrip') else 0.0),
                   arm_snap_l1=float(np.mean(np.abs(normalize(arm_c) - normalize(arm)))),
                   threads=int(a.threads), elapsed_s=round(time.time() - t0, 1), sim_variant=vinfo)
        rec['stage'] = next((s for s in reversed(STAGES) if rec[s]), 'none')
        (out / f'{uid}.json').write_text(json.dumps(rec))
        np.savez_compressed(out / f'{uid}_traj.npz', traj=traj, grip_cmd=grip_c.astype(np.float32),
                            arm_cmd=arm_c.astype(np.float32))
        print(f'[{a.code}] {uid}: n={n} stage={rec["stage"]} picked={rec["picked"]} placed={placed_shift} setdown={setdown} '
              f'contact={rec["contact"]} nested={nested} tipped={rec["tipped"]} tilt_end={can_tilt:.0f} dist={dist*100:.1f}cm '
              f'first={first} {rec["elapsed_s"]}s', flush=True)
    print('[run] done', flush=True)


# ---------------------------------------------------------------- vocab
def vocab(a):
    uids = a.uids or success_uids()
    words = Counter(); words_g = Counter(); n_tot = n_act = 0; five_c = Counter(); held_c = Counter()
    run_lens = []; run_lens_held = []
    per_uid = []
    for uid in uids:
        arm, gp = load_tape(uid)
        t = tern_words(arm); u, _ = grip_ramp(gp); f = five_levels(arm); h = held_words(t)
        act = np.any(t != 0, 1); n_tot += len(t); n_act += int(act.sum())
        for k in map(tuple, t[act]): words[k] += 1
        for k in map(tuple, np.concatenate([t, u[:, None]], 1)[act | (u != 0)]): words_g[k] += 1
        for k in map(tuple, f[np.any(f != 0, 1)]): five_c[k] += 1
        for k in map(tuple, h[np.any(h != 0, 1)]): held_c[k] += 1
        for seq, store in ((t, run_lens), (h, run_lens_held)):
            keys = [tuple(x) for x in seq]; i = 0
            while i < len(keys):
                j = i
                while j < len(keys) and keys[j] == keys[i]: j += 1
                if keys[i] != (0, 0, 0, 0): store.append(j - i)
                i = j
        per_uid.append(dict(uid=int(uid), n=int(len(t)), words=int(len({tuple(x) for x in t[act]}))))
    tot = sum(words.values())
    top = words.most_common()
    cov = lambda k: sum(c for _, c in top[:k]) / tot
    res = dict(n_uids=len(uids), frames=int(n_tot), active_frac=n_act / n_tot,
               distinct_arm_words=len(words), distinct_arm_grip_words=len(words_g),
               distinct_five_words=len(five_c), distinct_held_words=len(held_c),
               cov_top8=cov(8), cov_top16=cov(16), cov_top32=cov(32),
               words_for_99=next(k for k in range(1, len(top) + 1) if cov(k) >= 0.99),
               words_for_999=next(k for k in range(1, len(top) + 1) if cov(k) >= 0.999),
               top20=[(list(map(int, k)), int(c), round(c / tot, 4)) for k, c in top[:20]],
               run_len_tern=dict(median=float(np.median(run_lens)), p90=float(np.percentile(run_lens, 90)), n=len(run_lens)),
               run_len_held=dict(median=float(np.median(run_lens_held)), p90=float(np.percentile(run_lens_held, 90)), n=len(run_lens_held)),
               per_uid_words=dict(median=float(np.median([p['words'] for p in per_uid])), max=max(p['words'] for p in per_uid)),
               grip_rate=GRIP_RATE)
    pl.Path(a.outdir).mkdir(parents=True, exist_ok=True)
    (pl.Path(a.outdir) / 'vocab.json').write_text(json.dumps(res, indent=1))
    print(json.dumps({k: v for k, v in res.items() if k != 'top20'}, indent=1))
    for k, c, f in res['top20']: print(k, c, f)


# ---------------------------------------------------------------- table
ORD = ['none'] + list(STAGES)


def table(a):
    root = pl.Path(a.outdir)
    codes = [c for c in CODES if (root / c).exists()] + [c for c in sorted(p.name for p in root.iterdir() if p.is_dir())
                                                         if c not in CODES and c != 'logs']
    data = {}
    for c in codes:
        recs = {}
        for p in sorted((root / c).glob('*.json')):
            r = json.loads(p.read_text()); recs[int(r['uid'])] = r
        if recs: data[c] = recs
    ref = a.ref
    uids_all = sorted(set.intersection(*[set(d) for d in data.values()])) if a.common else None
    lines = []
    hdr = '| code | n | picked | placed | set-down | contact | nested | tipped | tilt>60 any | arm L1 | words/uid |'
    lines += [hdr, '|' + '---|' * (hdr.count('|') - 1)]
    summ = {}
    for c, recs in data.items():
        us = uids_all or sorted(recs)
        rs = [recs[u] for u in us]
        f = {k: sum(1 for r in rs if r[k]) for k in ('picked', 'placed', 'setdown', 'contact', 'nested', 'tipped')}
        f['tip_any'] = sum(1 for r in rs if r['tip_any_first'] is not None)
        l1 = np.mean([r['arm_snap_l1'] for r in rs]); wu = [r['n_words_used'] for r in rs if r['n_words_used'] is not None]
        summ[c] = dict(n=len(rs), **f, arm_l1=float(l1), words_per_uid=(float(np.median(wu)) if wu else None))
        lines.append(f'| {c} | {len(rs)} | {f["picked"]} | {f["placed"]} | {f["setdown"]} | {f["contact"]} | {f["nested"]} | '
                     f'{f["tipped"]} | {f["tip_any"]} | {l1:.3f} | {(np.median(wu) if wu else float("nan")):.0f} |')
    flips = {}
    if ref in data:
        rs = data[ref]
        for c, recs in data.items():
            if c == ref: continue
            us = [u for u in (uids_all or sorted(recs)) if u in rs]
            up = sorted(f'{u}:{rs[u]["stage"]}->{recs[u]["stage"]}' for u in us if ORD.index(recs[u]['stage']) > ORD.index(rs[u]['stage']))
            dn = sorted(f'{u}:{rs[u]["stage"]}->{recs[u]["stage"]}' for u in us if ORD.index(recs[u]['stage']) < ORD.index(rs[u]['stage']))
            per_stage = {}
            for s in ('picked', 'placed', 'setdown', 'contact', 'nested', 'tipped'):
                lost = sorted(u for u in us if rs[u][s] and not recs[u][s]); gained = sorted(u for u in us if recs[u][s] and not rs[u][s])
                per_stage[s] = dict(lost=lost, gained=gained)
            flips[c] = dict(ref=ref, n=len(us), stage_up=up, stage_down=dn, per_stage=per_stage)
            lines.append(f'\n**{c} vs {ref}** (n={len(us)}): stage UP {len(up)} {up}; DOWN {len(dn)} {dn}')
            for s, v in per_stage.items():
                lines.append(f'- {s}: lost {len(v["lost"])} {v["lost"]}  gained {len(v["gained"])} {v["gained"]}')
    txt = '\n'.join(lines)
    print(txt)
    (root / 'table.json').write_text(json.dumps(dict(summary=summ, flips=flips, common_uids=uids_all), indent=1))
    (root / 'table.md').write_text(txt + '\n')


def diag(a):
    """Timing vs spatial: for every code vs ref, first-stage frame shifts and settled-position
    shifts, on the uids that flipped and on the uids that did not."""
    root = pl.Path(a.outdir)
    load = lambda c: {int(json.loads(p.read_text())['uid']): json.loads(p.read_text()) for p in (root / c).glob('*.json')}
    rs = load(a.ref)
    for c in [c for c in CODES if c != a.ref and (root / c).exists()]:
        recs = load(c); us = sorted(u for u in recs if u in rs)
        same = [u for u in us if recs[u]['stage'] == rs[u]['stage']]
        flip = [u for u in us if recs[u]['stage'] != rs[u]['stage']]
        dpick = [recs[u]['first']['picked'] - rs[u]['first']['picked'] for u in same
                 if 'picked' in recs[u]['first'] and 'picked' in rs[u]['first']]
        dcon = [recs[u]['first']['contact'] - rs[u]['first']['contact'] for u in same
                if 'contact' in recs[u]['first'] and 'contact' in rs[u]['first']]
        dpos = [float(np.linalg.norm(np.array(recs[u]['can_end'][:2]) - np.array(rs[u]['can_end'][:2]))) * 100 for u in same]
        print(f'\n== {c} vs {a.ref}: same stage {len(same)}, flipped {len(flip)}')
        if dpick: print(f'  same-stage uids: first-pick shift (frames) median {np.median(dpick):+.0f} (IQR {np.percentile(dpick, 25):+.0f}..{np.percentile(dpick, 75):+.0f}); '
                        f'first-contact shift median {(np.median(dcon) if dcon else float("nan")):+.0f} (n {len(dcon)}); '
                        f'settled can xy shift median {np.median(dpos):.1f} cm, p90 {np.percentile(dpos, 90):.1f} cm')
        for u in flip:
            r, k = rs[u], recs[u]
            print(f'  {u}: {r["stage"]} -> {k["stage"]} | first ref {r["first"]} code {k["first"]} | dist {r["dist_settled"]*100:.1f} -> {k["dist_settled"]*100:.1f} cm '
                  f'| tilt {r["can_tilt_settled"]:.0f} -> {k["can_tilt_settled"]:.0f} | tip_free {r["tip_free_first"]} -> {k["tip_free_first"]} | n {r["n_frames"]}')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    d = sub.add_parser('diag'); d.add_argument('--outdir', required=True); d.add_argument('--ref', default='raw')
    r = sub.add_parser('run')
    r.add_argument('--code', required=True, choices=CODES)
    r.add_argument('--outdir', required=True)
    r.add_argument('--variant', default=VARIANT)
    r.add_argument('--shard-idx', type=int, default=0); r.add_argument('--shard-n', type=int, default=1)
    r.add_argument('--uids', type=int, nargs='*', default=None)
    r.add_argument('--threads', type=int, default=1, help='taichi/torch CPU threads per process (0 = library default)')
    v = sub.add_parser('vocab'); v.add_argument('--outdir', required=True); v.add_argument('--uids', type=int, nargs='*', default=None)
    t = sub.add_parser('table'); t.add_argument('--outdir', required=True); t.add_argument('--ref', default='raw')
    t.add_argument('--common', action='store_true', help='restrict every row to the uids present in all codes')
    args = ap.parse_args()
    dict(run=run, vocab=vocab, table=table, diag=diag)[args.cmd](args)
