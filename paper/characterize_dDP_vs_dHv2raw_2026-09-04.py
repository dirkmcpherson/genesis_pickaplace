#!/usr/bin/env python
"""Per-tape descriptors of two contract-v1 demonstration sets in the same world (dDP vs dHv2raw),
per-set tables, a per-IC paired comparison, and a summary figure. READ-ONLY on the tapes; no sim.

Usage (this box):
  ~/workspace/genesis_sim2real/venv/bin/python paper/characterize_dDP_vs_dHv2raw_2026-09-04.py \
      --sets ~/wm_fix_2026-09-03/characterization/sets --out ~/wm_fix_2026-09-03/characterization

Inputs: <sets>/dHv2raw/*.npz, <sets>/dDP/*.npz  (rsync of $LAB/genesis_pickaplace/baselines/matched_w3/{dHv2raw,dDP}).
Contract v1 (baselines/record_demos.py): one row per decision at 7.5 Hz (action_repeat 4 of a 30 Hz sim);
  states (n,17)  [0:6 arm qpos, 6 grip_obs, 7 grip effort, 8:11 can_pos, 11:15 can_quat, 15:17 goal_xy]
  actions (n,7)  absolute joint targets (rad) at the window end + grip cmd in [0,1] (0 open, ~0.8 closed)
  actions_delta (n,7)  the executed normalized decision in [-1,1]; arm step = delta * 4 * delta_cap = 0.1 rad * delta
  eef_pos (n+1,3)  tool position before each decision + final;  picked (n,) sticky; both sets end at the first picked row.

Descriptor definitions are in `descriptors()`; the markdown report restates them.
Outputs (all under --out): tape_descriptors.csv, per_set_table.md, paired_table.md, gallery_uids.json,
  fig_descriptors_dDP_vs_dHv2raw.png, fig_xy_paths_gallery.png, stats.json
"""
import argparse, glob, json, os, pathlib as pl, sys
import numpy as np

HZ = 7.5
DT = 1.0 / HZ
STEP_RAD = 0.1                 # 4 * delta_cap(0.025): rad per unit of actions_delta per decision
IDLE_RAD = 1e-3                # brief: max |delta action| < 1e-3 rad over the 6 arm columns
IDLE_CAP = 1e-3                # fig12 / extract_raw_vs_pruned convention: |actions_delta| < 1e-3 (cap units)
GRASP_CMD = 0.5                # closure = first grip cmd > 0.5
ONSET_CMD = 0.05               # start of the closing ramp
APPROACH_W = 8                 # decisions before ARRIVAL at the grasp pose ~ 1.07 s
ARRIVE_M = 0.015               # arrival = tool xy stays within this of its xy at closure
NUDGE_M = 0.005                # can xy displacement before the grasp counted as a nudge
SETS = {'human': 'dHv2raw', 'machine': 'dDP'}


def load_set(d):
    out = {}
    for f in sorted(glob.glob(os.path.join(d, '*.npz'))):
        z = np.load(f, allow_pickle=True)
        out[int(z['ic_uid'])] = f
    return out


def first_idx(mask, default):
    return int(np.argmax(mask)) if mask.any() else default


def descriptors(f):
    z = np.load(f, allow_pickle=True)
    a = np.asarray(z['actions'], np.float64); ad = np.asarray(z['actions_delta'], np.float64)
    st = np.asarray(z['states'], np.float64); e = np.asarray(z['eef_pos'], np.float64)
    fs = np.asarray(z['final_state'], np.float64); pk = np.asarray(z['picked'], bool)
    n = len(a)
    r = dict(uid=int(z['ic_uid']), tape=int(z['uid']), n_dec=n, dur_s=n * DT)
    # ---- idle: brief definition (rad, absolute-target differences, t = 1..n-1) and the fig12 one (cap units, all t)
    da = np.abs(np.diff(a[:, :6], axis=0)).max(axis=1)              # (n-1,) rad
    idle_rad = da < IDLE_RAD
    r['idle_frac'] = float(idle_rad.mean())
    r['idle_frac_capdef'] = float((np.abs(ad[:, :6]).max(axis=1) < IDLE_CAP).mean())
    r['lead_idle'] = first_idx(~idle_rad, n - 1)                    # decisions before the first arm motion
    # ---- tool path / speed / jerk from eef_pos (n+1,3)
    steps = np.linalg.norm(np.diff(e, axis=0), axis=1)              # (n,) m per decision
    r['path_len'] = float(steps.sum())
    r['mean_speed'] = float(steps.sum() / (n * DT))
    r['peak_speed'] = float(steps.max() / DT)
    r['p95_speed'] = float(np.percentile(steps, 95) / DT)
    d3 = np.diff(e, n=3, axis=0)                                    # (n-2,3)
    r['jerk_rms'] = float(np.sqrt((np.linalg.norm(d3, axis=1) ** 2).mean()) / DT ** 3)
    r['net_disp'] = float(np.linalg.norm(e[-1] - e[0]))
    r['tortuosity'] = float(steps.sum() / max(r['net_disp'], 1e-6))
    # ---- grip timing (cmd 0 open .. ~0.8 closed; grip_obs = states[:,6] measured finger position)
    g = a[:, 6]; gobs = st[:, 6]
    tg = first_idx(g > GRASP_CMD, n - 1); ton = first_idx(g > ONSET_CMD, tg)
    r['t_grasp'] = tg; r['t_grasp_s'] = tg * DT; r['t_onset'] = ton
    r['close_dur_cmd'] = tg - ton                                    # onset -> cmd crosses 0.5 (decisions)
    closed_lvl = 0.9 * fs[6]
    tcl = first_idx(gobs >= closed_lvl, n - 1)
    r['close_dur_obs'] = max(tcl - ton, 0)                           # onset -> fingers at 90% of final closure
    r['grip_cmd_final'] = float(g[-1]); r['grip_obs_final'] = float(fs[6])
    r['post_close_dec'] = (n - 1) - tg                               # decisions from closure to the pick row
    # ---- active (non-idle) motion
    r['active_dec'] = int((~idle_rad).sum())
    r['active_speed'] = float(steps[1:][~idle_rad].sum() / max(r['active_dec'] * DT, DT))   # path covered in non-idle decisions / their time
    r['mean_abs_delta_active'] = float(np.abs(ad[1:, :6][~idle_rad]).mean()) if r['active_dec'] else 0.0
    # ---- approach geometry. Anchor = ARRIVAL at the grasp pose: the first decision from which the tool xy stays
    # within ARRIVE_M of its xy at closure (tg). The cmd>0.5 crossing itself sits ~2 s into the closing ramp,
    # after the tool has stopped, so a window ending there measures nothing. Window = APPROACH_W decisions before arrival.
    can_xy_g = st[tg, 8:10]; can_z_g = st[tg, 10]
    dxy = np.linalg.norm(e[:tg + 1, :2] - e[tg, :2], axis=1)
    far = np.where(dxy > ARRIVE_M)[0]
    ta = int(far[-1] + 1) if len(far) else 0                          # arrival decision (<= tg)
    r['t_arrive'] = ta; r['dwell_dec'] = tg - ta                      # decisions parked at the grasp pose before cmd>0.5
    r['dwell_onset_dec'] = ton - ta                                    # parked before the grip even starts closing (can be <0)
    lo = max(0, ta - APPROACH_W); hi = ta
    r['approach_z_mean'] = float(e[lo:hi + 1, 2].mean())
    r['eef_z_at_arrive'] = float(e[ta, 2])
    r['eef_z_at_grasp'] = float(e[tg, 2])
    r['eef_above_can_at_grasp'] = float(e[tg, 2] - can_z_g)          # tool frame relative to the can centre
    r['z_drop_arrive_to_grasp'] = float(e[ta, 2] - e[tg, 2])          # vertical settle after arrival
    m = e[hi, :2] - e[lo, :2]; c = can_xy_g - e[lo, :2]
    nm, nc = np.linalg.norm(m), np.linalg.norm(c)
    r['approach_xy_travel'] = float(nm)
    r['approach_descent'] = float(e[lo, 2] - e[hi, 2])
    r['approach_dip_deg'] = float(np.degrees(np.arctan2(r['approach_descent'], max(nm, 1e-9))))
    r['approach_misalign_deg'] = float(np.degrees(np.arccos(np.clip(np.dot(m, c) / max(nm * nc, 1e-12), -1, 1)))) if nm > 2e-3 and nc > 2e-3 else np.nan
    v = e[lo, :2] - can_xy_g                                           # where the tool came from, seen from the can (world frame)
    r['approach_bearing_deg'] = float(np.degrees(np.arctan2(v[1], v[0])))
    r['approach_start_dist'] = float(np.linalg.norm(v))
    off = e[tg, :2] - can_xy_g
    r['grasp_xy_offset'] = float(np.linalg.norm(off))                  # tool-can xy offset at closure
    r['grasp_xy_offset_x'] = float(off[0]); r['grasp_xy_offset_y'] = float(off[1])
    # ---- can displacement before the grasp (nudges), xy only
    can0 = st[0, 8:10]
    dpre = np.linalg.norm(st[:tg + 1, 8:10] - can0, axis=1)
    r['can_disp_pre'] = float(np.linalg.norm(can_xy_g - can0))
    r['can_disp_pre_max'] = float(dpre.max())
    r['nudged'] = int(r['can_disp_pre_max'] > NUDGE_M)
    # ---- lift phase
    r['pick_eef_z'] = float(e[-1, 2]); r['pick_can_z'] = float(fs[10])
    cz = np.concatenate([st[tg:, 10], [fs[10]]])
    r['lift_regress'] = float((np.maximum.accumulate(cz) - cz).max())  # largest drop of can z below its running max after closure
    r['picked_rows'] = int(pk.sum())
    # ---- saturation of the executed delta
    sat6 = (np.abs(ad[:, :6]) >= 1 - 1e-6)
    r['sat_frac'] = float(sat6.any(axis=1).mean())
    r['sat_frac_any7'] = float((np.abs(ad) >= 1 - 1e-6).any(axis=1).mean())   # includes the grip column (-1 whenever open)
    r['sat_joint_frac'] = float(sat6.mean())                                    # per-joint-decision
    r['mean_abs_delta'] = float(np.abs(ad[:, :6]).mean())
    r['frac_delta_over_half'] = float((np.abs(ad[:, :6]) > 0.5).any(axis=1).mean())
    return r


DESC = [  # key, label, unit, higher-is
    ('n_dec', 'tape length', 'decisions'),
    ('dur_s', 'duration', 's'),
    ('idle_frac', 'idle fraction (|dq_cmd| < 1e-3 rad)', ''),
    ('idle_frac_capdef', 'idle fraction (|delta| < 1e-3 cap)', ''),
    ('lead_idle', 'leading idle', 'decisions'),
    ('path_len', 'tool path length', 'm'),
    ('net_disp', 'net tool displacement', 'm'),
    ('tortuosity', 'tortuosity (path/net)', ''),
    ('mean_speed', 'mean tool speed', 'm/s'),
    ('peak_speed', 'peak tool speed', 'm/s'),
    ('p95_speed', 'p95 tool speed', 'm/s'),
    ('jerk_rms', 'jerk RMS', 'm/s^3'),
    ('t_grasp', 'time-to-grasp (cmd>0.5)', 'decisions'),
    ('t_grasp_s', 'time-to-grasp', 's'),
    ('t_onset', 'grip onset (cmd>0.05)', 'decisions'),
    ('close_dur_cmd', 'grip-close duration (cmd onset->0.5)', 'decisions'),
    ('close_dur_obs', 'grip-close duration (onset->90% closed, measured)', 'decisions'),
    ('post_close_dec', 'closure -> pick', 'decisions'),
    ('active_dec', 'active (non-idle) decisions', 'decisions'),
    ('active_speed', 'tool speed over active decisions', 'm/s'),
    ('mean_abs_delta_active', 'mean |delta| over active decisions', 'cap units'),
    ('t_arrive', 'arrival at grasp pose', 'decisions'),
    ('dwell_dec', 'dwell: arrival -> cmd>0.5', 'decisions'),
    ('dwell_onset_dec', 'dwell: arrival -> grip onset', 'decisions'),
    ('approach_start_dist', 'tool-can xy distance 8 dec before arrival', 'm'),
    ('z_drop_arrive_to_grasp', 'z settle arrival -> closure', 'm'),
    ('approach_z_mean', 'approach height (mean eef z, 8 dec before arrival)', 'm'),
    ('eef_z_at_arrive', 'eef z at arrival', 'm'),
    ('eef_z_at_grasp', 'eef z at closure', 'm'),
    ('eef_above_can_at_grasp', 'eef z above can centre at closure', 'm'),
    ('approach_xy_travel', 'xy travel in the 8 dec before arrival', 'm'),
    ('approach_descent', 'descent in the 8 dec before arrival', 'm'),
    ('approach_dip_deg', 'approach dip angle', 'deg'),
    ('approach_misalign_deg', 'approach heading vs can', 'deg'),
    ('approach_bearing_deg', 'approach bearing (world)', 'deg'),
    ('grasp_xy_offset', 'tool-can xy offset at closure', 'm'),
    ('can_disp_pre', 'can xy displacement at closure', 'm'),
    ('can_disp_pre_max', 'max can xy displacement before closure', 'm'),
    ('nudged', 'nudged (>5 mm)', 'frac'),
    ('pick_eef_z', 'pick-frame tool height', 'm'),
    ('pick_can_z', 'pick-frame can z', 'm'),
    ('lift_regress', 'largest can-z drop after closure', 'm'),
    ('sat_frac', 'decisions with |delta|==1 (any arm joint)', ''),
    ('sat_joint_frac', 'joint-decisions at |delta|==1', ''),
    ('frac_delta_over_half', 'decisions with |delta|>0.5 (any joint)', ''),
    ('mean_abs_delta', 'mean |delta| (arm)', 'cap units'),
    ('grip_cmd_final', 'grip cmd at pick', ''),
]
CIRCULAR = {'approach_bearing_deg'}


def fmt(x, key):
    if x is None or (isinstance(x, float) and np.isnan(x)): return 'nan'
    if key in ('n_dec', 'lead_idle', 't_grasp', 't_onset', 'close_dur_cmd', 'close_dur_obs', 'post_close_dec'):
        return f'{x:.1f}'
    if abs(x) >= 100: return f'{x:.0f}'
    if abs(x) >= 10: return f'{x:.2f}'
    return f'{x:.3f}'


def md(lab):
    return lab.replace('|', '\\|')      # pipes inside markdown table cells


def pf(p):
    return '<1e-15' if p < 1e-15 else f'{p:.2g}'


def circ_mean_deg(a):
    a = np.radians(np.asarray(a, float)); a = a[np.isfinite(a)]
    return float(np.degrees(np.arctan2(np.sin(a).mean(), np.cos(a).mean())))


def circ_diff_deg(a, b):
    d = np.asarray(a, float) - np.asarray(b, float)
    return (d + 180.0) % 360.0 - 180.0


def paired_stats(x, y, rng, n_perm=200000):
    """x = human, y = machine, same uid order. Returns dict of effect sizes and p-values."""
    from scipy import stats
    d = np.asarray(x, float) - np.asarray(y, float)
    ok = np.isfinite(d); d = d[ok]; n = len(d)
    out = dict(n=n, mean_diff=float(d.mean()), median_diff=float(np.median(d)),
               sd_diff=float(d.std(ddof=1)) if n > 1 else np.nan)
    out['dz'] = out['mean_diff'] / out['sd_diff'] if out['sd_diff'] > 0 else np.nan
    out['frac_human_gt'] = float((d > 0).mean()); out['frac_ties'] = float((d == 0).mean())
    boots = rng.choice(d, size=(10000, n), replace=True).mean(axis=1)
    out['ci95_lo'], out['ci95_hi'] = float(np.percentile(boots, 2.5)), float(np.percentile(boots, 97.5))
    dz = d[d != 0]
    if len(dz) >= 1:
        try:
            out['p_wilcoxon'] = float(stats.wilcoxon(dz, method='exact').pvalue)
            out['wilcoxon_method'] = 'exact'
        except Exception:
            out['p_wilcoxon'] = float(stats.wilcoxon(dz).pvalue); out['wilcoxon_method'] = 'auto'
    else:
        out['p_wilcoxon'] = 1.0; out['wilcoxon_method'] = 'all-ties'
    # sign-flip permutation on the mean paired difference (exact enumeration if n <= 20, else Monte Carlo)
    obs = abs(d.mean())
    if n <= 20:
        signs = np.array([[1 if (i >> k) & 1 else -1 for k in range(n)] for i in range(2 ** n)], float)
        perm = np.abs((signs * d).mean(axis=1))
        out['p_perm'] = float((perm >= obs - 1e-15).mean()); out['perm_method'] = f'exact 2^{n}'
    else:
        signs = rng.choice([-1.0, 1.0], size=(n_perm, n))
        perm = np.abs((signs * d).mean(axis=1))
        out['p_perm'] = float(((perm >= obs - 1e-15).sum() + 1) / (n_perm + 1)); out['perm_method'] = f'MC {n_perm}'
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sets', default=os.path.expanduser('~/wm_fix_2026-09-03/characterization/sets'))
    ap.add_argument('--out', default=os.path.expanduser('~/wm_fix_2026-09-03/characterization'))
    ap.add_argument('--n-gallery', type=int, default=12)
    ap.add_argument('--machine', default='dDP', help='machine set dir name under --sets (2026-09-07: dRL for the machine-first arm)')
    ap.add_argument('--human', default='dHv2raw')
    args = ap.parse_args()
    SETS['machine'] = args.machine; SETS['human'] = args.human
    out = pl.Path(args.out); out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(0)

    files = {k: load_set(os.path.join(args.sets, v)) for k, v in SETS.items()}
    rows = {k: {u: descriptors(f) for u, f in files[k].items()} for k in files}
    paired = sorted(set(rows['human']) & set(rows['machine']))
    only_h = sorted(set(rows['human']) - set(rows['machine'])); only_m = sorted(set(rows['machine']) - set(rows['human']))
    print(f'human {len(rows["human"])} machine {len(rows["machine"])} paired {len(paired)} human-only {only_h} machine-only {only_m}')

    # ---- CSV of every tape
    keys = [k for k, _, _ in DESC] + ['grasp_xy_offset_x', 'grasp_xy_offset_y', 'tape', 'picked_rows', 'sat_frac_any7', 'grip_obs_final']
    with open(out / 'tape_descriptors.csv', 'w') as fh:
        fh.write('set,uid,paired,' + ','.join(keys) + '\n')
        for s in rows:
            for u in sorted(rows[s]):
                r = rows[s][u]
                fh.write(f'{SETS[s]},{u},{int(u in paired)},' + ','.join(str(r[k]) for k in keys) + '\n')

    # ---- per-set table (all tapes of each set)
    lines = ['| descriptor | unit | human %s (n=%d) mean / median [IQR] | machine %s (n=%d) mean / median [IQR] |' % (SETS['human'], len(rows['human']), SETS['machine'], len(rows['machine'])), '|---|---|---|---|']
    per_set = {}
    for k, lab, unit in DESC:
        cells = []
        for s in ('human', 'machine'):
            v = np.array([rows[s][u][k] for u in rows[s]], float); v = v[np.isfinite(v)]
            if k in CIRCULAR:
                cells.append(f'circ-mean {circ_mean_deg(v):.0f}')
                per_set.setdefault(k, {})[s] = dict(circ_mean=circ_mean_deg(v))
            else:
                q1, q3 = np.percentile(v, [25, 75])
                cells.append(f'{fmt(v.mean(), k)} / {fmt(np.median(v), k)} [{fmt(q1, k)}, {fmt(q3, k)}]')
                per_set.setdefault(k, {})[s] = dict(mean=float(v.mean()), median=float(np.median(v)), q1=float(q1), q3=float(q3), n=int(len(v)))
        lines.append(f'| {md(lab)} | {unit} | ' + ' | '.join(cells) + ' |')
    (out / 'per_set_table.md').write_text('\n'.join(lines) + '\n')

    # ---- paired comparison on the common uids
    stats_all = {}
    lines = [f'| descriptor | unit | human mean | machine mean | mean diff H-M [95% CI] | median diff | d_z | frac H>M | Wilcoxon p | perm p |', '|---|---|---|---|---|---|---|---|---|---|']
    for k, lab, unit in DESC:
        x = np.array([rows['human'][u][k] for u in paired], float); y = np.array([rows['machine'][u][k] for u in paired], float)
        if k in CIRCULAR:
            dd = circ_diff_deg(x, y); ok = np.isfinite(dd)
            st = paired_stats(dd[ok], np.zeros(ok.sum()), rng)
            st['human_mean'] = circ_mean_deg(x); st['machine_mean'] = circ_mean_deg(y); st['note'] = 'circular difference H-M in (-180,180]'
        else:
            st = paired_stats(x, y, rng); st['human_mean'] = float(np.nanmean(x)); st['machine_mean'] = float(np.nanmean(y))
        stats_all[k] = st
        lines.append(f'| {md(lab)} | {unit} | {fmt(st["human_mean"], k)} | {fmt(st["machine_mean"], k)} | {fmt(st["mean_diff"], k)} [{fmt(st["ci95_lo"], k)}, {fmt(st["ci95_hi"], k)}] | {fmt(st["median_diff"], k)} | {st["dz"]:+.2f} | {st["frac_human_gt"]:.2f} | {pf(st["p_wilcoxon"])} | {pf(st["p_perm"])} |')
    (out / 'paired_table.md').write_text('\n'.join(lines) + '\n')
    json.dump(dict(paired=paired, human_only=only_h, machine_only=only_m, per_set=per_set, paired_stats=stats_all,
                   constants=dict(HZ=HZ, STEP_RAD=STEP_RAD, IDLE_RAD=IDLE_RAD, IDLE_CAP=IDLE_CAP, GRASP_CMD=GRASP_CMD,
                                  ONSET_CMD=ONSET_CMD, APPROACH_W=APPROACH_W, NUDGE_M=NUDGE_M)),
              open(out / 'stats.json', 'w'), indent=1, default=float)

    # ---- gallery selection: 12 paired uids at even quantiles of the human/machine length ratio
    ratio = np.array([rows['human'][u]['n_dec'] / rows['machine'][u]['n_dec'] for u in paired])
    order = np.argsort(ratio)
    picks = sorted({int(order[int(round(q))]) for q in np.linspace(0, len(paired) - 1, args.n_gallery)})
    gal = [paired[i] for i in picks]
    json.dump(dict(uids=gal, rule='even quantiles of human/machine tape-length ratio over paired uids',
                   ratio={str(paired[i]): float(ratio[i]) for i in picks}), open(out / 'gallery_uids.json', 'w'), indent=1)
    print('gallery uids', gal)

    # ---- summary figure
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    show = [k for k in ('n_dec', 'idle_frac', 'lead_idle', 'path_len', 'mean_speed', 'active_speed', 'peak_speed', 'jerk_rms',
                        't_grasp_s', 'close_dur_cmd', 'dwell_dec', 'post_close_dec', 'approach_z_mean', 'approach_misalign_deg',
                        'approach_dip_deg', 'grasp_xy_offset', 'can_disp_pre_max', 'pick_eef_z', 'sat_frac', 'mean_abs_delta_active',
                        'lift_regress')]
    labels = {k: (lab, unit) for k, lab, unit in DESC}
    ncol = 7; nrow = int(np.ceil(len(show) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(2.3 * ncol, 2.4 * nrow))
    ch, cm = '#d95f02', '#7570b3'   # paper/figures/colstyle.py source colours (human orange, machine purple)
    for ax, k in zip(axes.ravel(), show):
        xs = [np.array([rows[s][u][k] for u in rows[s]], float) for s in ('human', 'machine')]
        xs = [v[np.isfinite(v)] for v in xs]
        vp = ax.violinplot(xs, positions=[0, 1], widths=0.8, showmedians=True, showextrema=False)
        for body, c in zip(vp['bodies'], (ch, cm)): body.set_facecolor(c); body.set_alpha(0.35); body.set_edgecolor(c)
        vp['cmedians'].set_color('k')
        for i, (v, c) in enumerate(zip(xs, (ch, cm))):
            ax.scatter(i + rng.uniform(-0.12, 0.12, len(v)), v, s=5, color=c, alpha=0.6, linewidths=0)
        lab, unit = labels[k]
        st = stats_all[k]
        ax.set_title(f'{lab}\n{unit}  paired p={pf(st["p_wilcoxon"])} d_z={st["dz"]:+.2f}', fontsize=7)
        ax.set_xticks([0, 1]); ax.set_xticklabels(['human', 'machine'], fontsize=7); ax.tick_params(axis='y', labelsize=6)
    for ax in axes.ravel()[len(show):]: ax.axis('off')
    fig.suptitle(f'{SETS["human"]} (human, n={len(rows["human"])}) vs {SETS["machine"]} (machine, n={len(rows["machine"])}) - world gc_kp4_riser3_shelf6, pick scope, 7.5 Hz decisions; p = Wilcoxon on the {len(paired)} paired ICs', fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out / f'fig_descriptors_{SETS["machine"]}_vs_{SETS["human"]}.png', dpi=160); plt.close(fig)

    # ---- xy trajectory panel for the gallery uids
    fig, axes = plt.subplots(3, 4, figsize=(12, 9))
    for ax, u in zip(axes.ravel(), gal):
        for s, cmap, mk in (('human', 'Oranges', 'o'), ('machine', 'Purples', 's')):
            z = np.load(files[s][u], allow_pickle=True); e = np.asarray(z['eef_pos']); st = np.asarray(z['states'])
            t = np.arange(len(e)) / HZ
            ax.scatter(e[:, 0], e[:, 1], c=t, cmap=cmap, s=6, vmin=0, vmax=t.max() * 1.05, linewidths=0)
            ax.plot(e[:, 0], e[:, 1], color=plt.get_cmap(cmap)(0.7), lw=0.6, alpha=0.6, label=f'{SETS[s]} ({len(e) - 1} dec)')
            tg = rows[s][u]['t_grasp']; ax.plot(e[tg, 0], e[tg, 1], marker='x', color=plt.get_cmap(cmap)(0.95), ms=7, mew=1.5)
            can_t = st[:, 8:10]
            ax.plot(can_t[:, 0], can_t[:, 1], color=plt.get_cmap(cmap)(0.9), lw=1.5, alpha=0.9, ls='--')
        z = np.load(files['human'][u], allow_pickle=True); st = np.asarray(z['states'])
        ax.add_patch(plt.Circle(st[0, 8:10], 0.033, fill=False, color='k', lw=1))
        ax.plot(st[0, 15], st[0, 16], marker='*', color='k', ms=8)
        ax.set_title(f'uid {u}  (x = closure, dashed = can)', fontsize=8); ax.set_aspect('equal'); ax.tick_params(labelsize=6)
        ax.legend(fontsize=6, loc='best')
    fig.suptitle('Tool xy paths, time-coloured (light -> dark), human orange vs machine purple; circle = can at t0, star = goal', fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(out / 'fig_xy_paths_gallery.png', dpi=130); plt.close(fig)
    print('wrote', out)


if __name__ == '__main__':
    main()
