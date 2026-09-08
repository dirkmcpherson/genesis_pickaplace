"""End-effector per-decision action distribution over (dx, dy, dz, dyaw).

WHY THIS AND NOT A HISTOGRAM OF vel_cmd
---------------------------------------
The learners do not act in raw joystick space: they emit `delta_joint` actions on a 0.12 s
decision cadence.  The comparable physical quantity across every arm (human tapes, DP, RLPD,
world model) is therefore the END-EFFECTOR DISPLACEMENT PER 0.12 s DECISION -- dx, dy, dz [m]
and dyaw [rad about the world z axis].  This script builds that distribution for the human
reference from the local bags, and is written so a learner's rollout actions drop into the
SAME binning and plot alongside (see --source / --rollouts / --bins below).

CLOCK (CONFOUNDS row 46).  The tapes run at 29.6-39.2 fps (median 32.8) because of a beat
artifact between two 40 Hz sources, so the recorder's "4 tape frames per decision" stride is
wrong by up to 20 %.  The default here (--grid time) resamples on the REAL timestamps in
inthewild_trials/<uid>_timed.npz (`fb_t`, `fb_tool`: every base_feedback message, a clean
40.00 Hz, no windowing, no averaging).  --grid frames reproduces the recorder's naive stride
and exists only to quantify what row 46 costs on this statistic.

POSE CONVENTION (verified, see paper/EEF_ACTION_DIST_2026-09-07.md):
  fb_tool[:, 0:3] = tool-frame position [m] in the robot base frame
  fb_tool[:, 3:6] = (theta_x, theta_y, theta_z) Euler angles in DEGREES, extrinsic xyz
                    (= ROS roll-pitch-yaw): R = Rz(theta_z) Ry(theta_y) Rx(theta_x)
  => theta_z IS the yaw about the world z axis (up to a constant 180 deg tool-frame offset,
     which cancels in a difference).  Roll/pitch are near-constant (per-tape ptp 2.8/0.8 deg),
     i.e. the arm is effectively 4-DOF in task space.
Differencing wraps dyaw into (-pi, pi].

USAGE
  # human reference (default): build + report + plot
  PY=~/workspace/genesis_sim2real/venv/bin/python
  $PY can_pos_recovery/eef_action_dist.py --build --report --plot

  # a learner arm later, on the SAME bins:
  $PY can_pos_recovery/eef_action_dist.py --build --source rlpd_dH \
      --rollouts path/to/rollouts.npz --bins paper/figures/eef_action_bins.npz
  $PY can_pos_recovery/eef_action_dist.py --plot --source human --source rlpd_dH

ROLLOUT FILE FORMAT (for --source != human).  An .npz holding EITHER
  deltas : (N,4) float, per-decision (dx, dy, dz, dyaw) in (m, m, m, rad); optional
           `episode` (N,) int grouping them, or
  poses  : (N,4) float, per-decision EEF pose (x, y, z, yaw_rad) with `episode` (N,) int --
           the script differences within each episode and wraps dyaw.
Anything else raises; nothing is inferred silently.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
TRIALS = REPO / 'inthewild_trials'
FIGDIR = REPO / 'paper' / 'figures'
DT = 0.12                      # learner decision cadence [s]
CH = ('dx', 'dy', 'dz', 'dyaw')
UNITS = ('m', 'm', 'm', 'rad')
DISP = ('mm', 'mm', 'mm', 'deg')          # display units
DISP_SCALE = (1e3, 1e3, 1e3, 180.0 / np.pi)


# ----------------------------------------------------------------------------- human tapes
def human_uids():
    """Every uid with a timestamped re-extraction.  Verified (2026-09-07) to be EXACTLY the
    74-tape full-scope set of record (== the uid list of the w3 honest census)."""
    return sorted(int(p.name.split('_')[0]) for p in TRIALS.glob('*_timed.npz'))


def tape_deltas(uid, grid='time', dt=DT):
    """Per-decision (dx,dy,dz,dyaw) for one human tape, plus a per-decision 'quiet' flag
    (no /cartesian_velocity message landed in the window => joystick centred).

    grid='time'   : real timestamps, uniform dt grid, linear interpolation of the pose
                    (yaw unwrapped first).  THE MEASUREMENT OF RECORD.
    grid='frames' : the recorder's naive stride of round(dt * nominal_fps) tape frames,
                    where nominal_fps is 1/60-window frames per second -- row 46 sensitivity.
    """
    z = np.load(TRIALS / f'{uid}_timed.npz', allow_pickle=True)
    t, tool, cv = z['fb_t'], z['fb_tool'], z['cv_t']
    yaw_un = np.unwrap(np.radians(tool[:, 5]))
    if grid == 'time':
        k = int(np.floor((t[-1] - t[0]) / dt))
        if k < 2:
            return None
        tg = t[0] + dt * np.arange(k + 1)
        cols = [np.interp(tg, t, tool[:, i]) for i in range(3)] + [np.interp(tg, t, yaw_un)]
        d = np.stack([np.diff(c) for c in cols], axis=1)
        quiet = np.histogram(cv, bins=tg)[0] == 0
    elif grid == 'frames':
        tf = z['t_frame']
        stride = 4                                    # HumanFollower consumes 4 frames/decision
        idx = np.arange(0, len(tf), stride)
        if len(idx) < 3:
            return None
        # the recorder's waypoints are the 1/60-window frames; take their pose from fb by time
        cols = [np.interp(tf[idx], t, tool[:, i]) for i in range(3)] + [np.interp(tf[idx], t, yaw_un)]
        d = np.stack([np.diff(c) for c in cols], axis=1)
        quiet = np.histogram(cv, bins=tf[idx])[0] == 0
    else:
        raise ValueError(grid)
    d[:, 3] = (d[:, 3] + np.pi) % (2 * np.pi) - np.pi   # wrap-safe dyaw
    return d, quiet


def build_human(grid='time', dt=DT):
    uids, D, Q, E, skipped = human_uids(), [], [], [], []
    for u in uids:
        r = tape_deltas(u, grid=grid, dt=dt)
        if r is None:
            skipped.append((u, 'fewer than 2 decisions'))
            continue
        d, q = r
        D.append(d); Q.append(q); E.append(np.full(len(d), u))
    return (np.concatenate(D), np.concatenate(Q), np.concatenate(E),
            [u for u in uids if u not in {s[0] for s in skipped}], skipped)


# ------------------------------------------------------------------------- learner rollouts
def build_rollouts(path):
    z = np.load(path, allow_pickle=True)
    keys = set(z.files) if hasattr(z, 'files') else set()
    if 'deltas' in keys:
        d = np.asarray(z['deltas'], float)
        if d.ndim != 2 or d.shape[1] != 4:
            raise SystemExit(f'{path}: deltas must be (N,4), got {d.shape}')
        ep = np.asarray(z['episode']) if 'episode' in keys else np.zeros(len(d), int)
    elif 'poses' in keys:
        p = np.asarray(z['poses'], float)
        if p.ndim != 2 or p.shape[1] != 4:
            raise SystemExit(f'{path}: poses must be (N,4), got {p.shape}')
        if 'episode' not in keys:
            raise SystemExit(f'{path}: poses requires an `episode` array')
        e = np.asarray(z['episode'])
        D, E = [], []
        for u in np.unique(e):
            m = e == u
            q = p[m].copy(); q[:, 3] = np.unwrap(q[:, 3])
            D.append(np.diff(q, axis=0)); E.append(np.full(m.sum() - 1, u))
        d, ep = np.concatenate(D), np.concatenate(E)
    else:
        raise SystemExit(f'{path}: needs a `deltas` or a `poses` array (see module docstring)')
    d[:, 3] = (d[:, 3] + np.pi) % (2 * np.pi) - np.pi
    return d, np.zeros(len(d), bool), ep


# ------------------------------------------------------------------------------------ bins
def make_bins(d, nbin=61, span_pct=99.9):
    """Symmetric per-channel edges spanning the |delta| span_pct percentile of THIS source.
    Written once (by the human build) and reused by every later source via --bins."""
    edges = []
    for i in range(4):
        r = float(np.percentile(np.abs(d[:, i]), span_pct))
        edges.append(np.linspace(-r, r, nbin + 1))
    return np.stack(edges)


# --------------------------------------------------------------------------------- reports
def q_table(d, pcts=(1, 5, 25, 50, 75, 95, 99)):
    return {CH[i]: {str(p): float(np.percentile(d[:, i], p)) for p in pcts} for i in range(4)}


def zero_fracs(d, thresholds):
    return {CH[i]: {f'{t:g}': float((np.abs(d[:, i]) < t).mean()) for t in thresholds[i]}
            for i in range(4)}


def discrete_table(d, ep, bins_list=(3, 5, 7, 9, 15, 21, 31), span_pct=99.9, tol=None):
    """For a uniform B-level grid per channel spanning +-R (R = span_pct of |delta|):
       w          bin width
       clip_frac  fraction of decisions outside +-R (clipped)
       err_p50/95 |delta - snapped| in display units
       err_rel    err / (w/2)  -- ~0 if the signal already sits on levels, ~0.5 if uniform
       drift      open-loop reconstruction error: cumsum(snapped) vs cumsum(delta) per tape,
                  |final| xyz [mm] and |final| yaw [deg]
    """
    R = np.array([np.percentile(np.abs(d[:, i]), span_pct) for i in range(4)])
    out = {'span_pct': span_pct, 'R': {CH[i]: float(R[i]) for i in range(4)}, 'grids': {}}
    if tol is not None:
        out['bins_for_tol'] = {CH[i]: {f'{t:g}{DISP[i]}': int(np.ceil(R[i] * DISP_SCALE[i] / t))
                                       for t in tol[i]} for i in range(4)}
    for B in bins_list:
        w = 2 * R / B
        centers = [(-R[i] + w[i] * (np.arange(B) + 0.5)) for i in range(4)]
        row = {}
        snap = np.zeros_like(d)
        for i in range(4):
            c = np.clip(d[:, i], -R[i], R[i])
            j = np.clip(((c + R[i]) / w[i]).astype(int), 0, B - 1)
            snap[:, i] = centers[i][j]
            err = np.abs(d[:, i] - snap[:, i])
            row[CH[i]] = dict(
                w=float(w[i] * DISP_SCALE[i]),
                has_zero_level=bool(np.min(np.abs(centers[i])) < 1e-12),
                clip_frac=float((np.abs(d[:, i]) > R[i]).mean()),
                err_p50=float(np.percentile(err, 50) * DISP_SCALE[i]),
                err_p95=float(np.percentile(err, 95) * DISP_SCALE[i]),
                err_rel_mean=float(np.mean(err / (w[i] / 2))),
                frac_within_half_w=float((err <= w[i] / 2 + 1e-12).mean()))
        dr_xyz, dr_yaw = [], []
        for u in np.unique(ep):
            m = ep == u
            e = np.cumsum(snap[m] - d[m], axis=0)
            dr_xyz.append(np.linalg.norm(e[-1, :3]) * 1e3)
            dr_yaw.append(abs(e[-1, 3]) * 180 / np.pi)
        row['drift'] = dict(xyz_mm_p50=float(np.percentile(dr_xyz, 50)),
                            xyz_mm_p95=float(np.percentile(dr_xyz, 95)),
                            xyz_mm_max=float(np.max(dr_xyz)),
                            yaw_deg_p50=float(np.percentile(dr_yaw, 50)),
                            yaw_deg_max=float(np.max(dr_yaw)))
        out['grids'][B] = row
    return out


# ------------------------------------------------------------------------------------ plot
def plot_grid(sources, out_name, log_marginal=True):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    sys.path.insert(0, str(FIGDIR))
    try:
        import colstyle as cs
        cs.setup(); W = cs.W
        COL = dict(human=cs.C['human'], machine=cs.C['machine'], DP=cs.C['DP'],
                   RLPD=cs.C['RLPD'], WM=cs.C['WM'], DV3=cs.C['DV3'])
    except Exception:                                   # keep the script runnable standalone
        W = 3.35; COL = dict(human='#d95f02', machine='#7570b3')
        plt.rcParams.update({'font.size': 8, 'axes.linewidth': 0.7, 'figure.dpi': 200,
                             'savefig.bbox': 'tight', 'pdf.fonttype': 42})
    palette = ['#d95f02', '#7570b3', '#1f77b4', '#d62728', '#2ca02c', '#9467bd']

    edges = None
    packs = []
    for k, s in enumerate(sources):
        z = np.load(FIGDIR / f'eef_action_dist_{s}.npz', allow_pickle=True)
        d = z['deltas']
        if edges is None:
            edges = z['bin_edges']
        col = COL.get(s, palette[k % len(palette)])
        packs.append((s, d, col))

    ed = [edges[i] * DISP_SCALE[i] for i in range(4)]
    fig, axs = plt.subplots(4, 4, figsize=(W, W * 1.06))
    solo = len(packs) == 1
    for r in range(4):
        for c in range(4):
            ax = axs[r, c]
            ax.tick_params(labelsize=5.2, length=1.6, pad=1)
            for sp in ('top', 'right'):
                ax.spines[sp].set_visible(False)
            if r == c:                                            # marginal
                for s, d, col in packs:
                    ax.hist(d[:, r] * DISP_SCALE[r], bins=ed[r], histtype='step',
                            color=col, lw=0.8, density=True, log=log_marginal)
                ax.set_yticks([])
                ax.set_xlim(ed[r][0], ed[r][-1])
            elif r > c:                                           # 2D density (lower triangle)
                for k, (s, d, col) in enumerate(packs):
                    H, _, _ = np.histogram2d(d[:, c] * DISP_SCALE[c], d[:, r] * DISP_SCALE[r],
                                             bins=[ed[c], ed[r]])
                    H = H.T / max(H.sum(), 1)
                    if solo:
                        ax.pcolormesh(ed[c], ed[r], np.log10(H + 1e-7), cmap='magma_r',
                                      rasterized=True, shading='flat')
                    else:
                        lv = [np.quantile(H[H > 0], q) for q in (0.90, 0.99)]
                        ax.contour(0.5 * (ed[c][:-1] + ed[c][1:]), 0.5 * (ed[r][:-1] + ed[r][1:]),
                                   H, levels=sorted(set(lv)), colors=col, linewidths=0.6)
                ax.axhline(0, color='0.7', lw=0.3, zorder=0)
                ax.axvline(0, color='0.7', lw=0.3, zorder=0)
                ax.set_xlim(ed[c][0], ed[c][-1]); ax.set_ylim(ed[r][0], ed[r][-1])
            else:                                                 # upper triangle: Pearson r
                ax.axis('off')
                n = len(packs)
                for k, (s, d, col) in enumerate(packs):
                    ax.text(0.5, 0.5 + (n - 1) * 0.16 - k * 0.32,
                            '%+.2f' % np.corrcoef(d[:, r], d[:, c])[0, 1],
                            ha='center', va='center', fontsize=6.4, color=col,
                            transform=ax.transAxes)
            if r == 3 and r != c:
                ax.set_xlabel(f'{CH[c]} [{DISP[c]}]', fontsize=6, labelpad=1)
            if r == 3 and r == c:
                ax.set_xlabel(f'{CH[3]} [{DISP[3]}]', fontsize=6, labelpad=1)
            if c == 0 and r > 0:
                ax.set_ylabel(f'{CH[r]} [{DISP[r]}]', fontsize=6, labelpad=1)
            if r < 3:
                ax.set_xticklabels([])
            if c > 0 and r > c:
                ax.set_yticklabels([])
    fig.suptitle(f'EEF displacement per {DT:g} s decision', fontsize=7.5, y=1.005)
    sub = ('log$_{10}$ density; upper triangle = Pearson r' if solo else
           'contours = 90th/99th density pct; upper triangle = Pearson r')
    hs = [plt.Line2D([], [], color=col, lw=1.2, label=s) for s, _, col in packs]
    fig.legend(handles=hs, loc='upper right', bbox_to_anchor=(0.995, 0.995), fontsize=6,
               frameon=False, handlelength=1.1, labelspacing=0.25)
    fig.text(0.5, -0.012, sub, ha='center', fontsize=5.8, color='0.35')
    fig.subplots_adjust(hspace=0.16, wspace=0.16)
    fig.savefig(FIGDIR / f'{out_name}.png'); fig.savefig(FIGDIR / f'{out_name}.pdf')
    plt.close(fig)
    print('wrote', FIGDIR / f'{out_name}.png')


# ------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--source', action='append', default=None,
                    help="arm name; 'human' (default) reads the local tapes, anything else "
                         "needs --rollouts")
    ap.add_argument('--rollouts', default=None, help='npz of learner rollout actions/poses')
    ap.add_argument('--from-tapes', action='store_true',
                    help='build from the local human tapes even under a non-"human" --source '
                         '(used for the --grid frames row-46 sensitivity arm)')
    ap.add_argument('--grid', choices=('time', 'frames'), default='time',
                    help="'time' = real timestamps (of record); 'frames' = recorder stride")
    ap.add_argument('--dt', type=float, default=DT)
    ap.add_argument('--bins', default=None, help='reuse bin edges from this npz')
    ap.add_argument('--nbin', type=int, default=61)
    ap.add_argument('--build', action='store_true')
    ap.add_argument('--report', action='store_true')
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--fig-name', default=None)
    ap.add_argument('--json-out', default=None)
    a = ap.parse_args()
    sources = a.source or ['human']
    FIGDIR.mkdir(parents=True, exist_ok=True)

    if a.build:
        s = sources[0]
        if s == 'human' or a.from_tapes:
            d, quiet, ep, uids, skipped = build_human(grid=a.grid, dt=a.dt)
            meta = dict(source=s, grid=a.grid, dt=a.dt, n_tapes=len(uids), uids=uids,
                        skipped=skipped, pose_signal='fb_tool (base_feedback, 40.00 Hz)',
                        yaw='tool_pose theta_z, extrinsic-xyz yaw about world z')
        else:
            if not a.rollouts:
                raise SystemExit(f'--source {s} needs --rollouts')
            d, quiet, ep = build_rollouts(a.rollouts)
            uids = sorted(int(x) for x in np.unique(ep))
            meta = dict(source=s, grid='rollout', dt=a.dt, n_tapes=len(uids),
                        rollouts=str(a.rollouts), skipped=[])
        edges = np.load(a.bins)['bin_edges'] if a.bins else make_bins(d, a.nbin)
        out = FIGDIR / f'eef_action_dist_{s}.npz'
        np.savez_compressed(out, deltas=d, quiet=quiet, episode=ep, bin_edges=edges,
                            channels=np.array(CH), units=np.array(UNITS),
                            meta=json.dumps(meta))
        if s == 'human' and not a.bins:
            np.savez_compressed(FIGDIR / 'eef_action_bins.npz', bin_edges=edges,
                                channels=np.array(CH), units=np.array(UNITS))
        print(f'wrote {out}  n_decisions={len(d)} n_tapes={meta["n_tapes"]} '
              f'skipped={len(meta["skipped"])}')

    if a.report:
        rep = {}
        for s in sources:
            z = np.load(FIGDIR / f'eef_action_dist_{s}.npz', allow_pickle=True)
            d, quiet, ep = z['deltas'], z['quiet'], z['episode']
            m = json.loads(str(z['meta']))
            thr = [[1e-5, 5e-5, 1e-4, 2.5e-4, 5e-4, 1e-3, 2e-3]] * 3 + \
                  [[np.radians(x) for x in (0.02, 0.1, 0.2, 0.5, 1, 2)]]
            tol = [[2.0, 1.0, 0.5]] * 3 + [[2.0, 1.0, 0.5]]
            r = dict(meta=m, n_decisions=int(len(d)),
                     quantiles=q_table(d), zero_fracs=zero_fracs(d, thr),
                     corr=np.round(np.corrcoef(d.T), 4).tolist(),
                     abs_max={CH[i]: float(np.abs(d[:, i]).max()) for i in range(4)},
                     discrete=discrete_table(d, ep, tol=tol))
            if quiet.any():
                r['quiet'] = dict(frac=float(quiet.mean()),
                                  abs_p50={CH[i]: float(np.percentile(np.abs(d[quiet, i]), 50))
                                           for i in range(4)},
                                  abs_p95={CH[i]: float(np.percentile(np.abs(d[quiet, i]), 95))
                                           for i in range(4)})
            rep[s] = r
        txt = json.dumps(rep, indent=1)
        print(txt)
        if a.json_out:
            Path(a.json_out).write_text(txt)

    if a.plot:
        name = a.fig_name or ('fig_eef_action_dist_' + '_'.join(sources))
        plot_grid(sources, name)


if __name__ == '__main__':
    main()
