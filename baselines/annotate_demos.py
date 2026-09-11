#!/usr/bin/env python
"""Phase-annotated video of a DEMONSTRATION tape (Lane 5, LADDER_UNIFY_BRIEF_2026-09-10).

    # census: re-execute every tape, no video, one JSON row per tape
    python baselines/annotate_demos.py census --in <set-dir> --out <census.json> [--procs 8]

    # render: one mp4 per named tape
    python baselines/annotate_demos.py render --in <set-dir> --out-dir <dir> \
        --tapes genesis-100000-013-256.npz ... [--procs 6] [--set human]

WHY THIS FILE EXISTS
--------------------
The user wants to check the phase annotations BY EYE, on the demonstrations themselves, before
a training batch relies on them. Every predicate the unified ladder pays or logs is a claim
about what happened in the world; the only cheap way to falsify such a claim is to watch the
frame the chip lights on and see whether the world agrees.

WHAT IT DOES, AND WHAT IT DOES NOT
----------------------------------
It RE-EXECUTES the tape through `FullTaskEnv(scope='full', ladder=...)` -- the same code path
training uses, the same world, the same action semantics -- exactly as `baselines/rl/
relabel_reward.py` (D5) does, and it IMPORTS that module's tape reader, IC restore and env
invariants rather than re-implementing them. It computes NO predicate of its own: every chip
is read from `info` / `env._granted` / `env.tracker`, which is what makes the video evidence
about the ladder rather than about this file. The one number it derives is the SPARSE ladder's
counterfactual pay (see `sparse would pay` below), and that derivation is stated on screen.

THE OVERLAY
-----------
LADDER row  (bright, the rungs that pay under 'staged'):
    PICK   picked          +1   env-owned (genesis_can_env's hardened held-can flag)
    PLACE  placed_v2       +1   release + shelf footprint + z-band + tilt, sustained 10 frames
    PUSH   contact_push    +2   RELEASE-GATED: placed_v2 granted, can<->goal contact, tool on
                                the far side, gripper clear of the goal
    NEST2  nested_v2        0   LOGGED, never paid, never terminal under 'staged'
    SLIDE  slide_success   +4   released AND pushed AND nested_v2 -- the only non-tip terminal
LEGACY row  (dim, visually separated, behind a rule -- these pay NOTHING and terminate NOTHING):
    nestP  `nested`, the withdrawn training proxy (precision 0.114 human / 0.029 machine)
    pushL  `contact_push_legacy`, the (g) geometry with NO release requirement
    slidL  `slide_success_legacy`, the (l) predicate with the grip < 0.3 clause (p) withdrew
A chip lights ON THE FRAME its stage is granted (a YELLOW flash for 6 decisions -- the tuple
(60, 255, 255) is yellow in OpenCV's BGR order), then stays GREEN, and carries the decision
index it fired on. Grey outline = not granted yet.

Diagnostics line: decision index, `lever_m` (|tool_xy - can_xy|), `in_hand`, `at_rest`,
`goalward_gain_m`, `dist_xy` to the goal, and the COMMANDED grip -- all read from the tracker
and the action, never recomputed. `grip` is shown because the user will ask about it; NO
predicate on this ladder reads it, which is the point of amendments (l)/(p)/(x).

Reward line: `staged` = what the env actually paid, cumulative, out of the ladder max.
`sparse would pay` = the counterfactual for `ladder='sparse'` (nested_v2 = 1, terminal): 0 until
the first nested_v2 frame, 1.0 from it, with `(ENDS)` marking that the sparse episode would
have terminated there. It is a counterfactual on THIS trajectory, not a second run.

Terminal card (held at the end): end reason, the settled `nested_honest` from ONE
`end_of_episode()` settle, `nested_v2` / `slide_success` / `pushed`, and the tape's RECORDED
(old-ladder) reward beside the re-executed one.

FRAME BUDGET
------------
<= MAX_FRAMES per clip. The body is uniformly subsampled, but every frame on which a stage is
granted is FORCE-INCLUDED together with its neighbours (t-1, t, t+1), so a subsample can never
hide the event the clip exists to show. Playback fps is set to preserve wall-clock, so
"the can is at rest" looks like rest.

Rendering is CPU (Genesis `backend='cpu'`, its CPU rasteriser); the local GPU is left free.
mp4v is written first and transcoded to H.264 with ffmpeg -c:v libx264.

ONE GENESIS WORLD PER PROCESS, so --procs N re-launches this script as N shards.
"""
import argparse
import json
import os
import pathlib as pl
import re
import subprocess
import sys
import time

import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

# The tape reader, the IC restore and the env invariants come from the D5 relabel path.
# Importing them is the point: a second reader is a second set of assumptions.
from relabel_reward import tape_layout, tape_stream, reset_to_tape_ic, scalar, set_meta  # noqa: E402

MAX_FRAMES = 400
CARD_FRAMES = 18           # held terminal card
BODY_BUDGET = 340          # uniform body before forced frames are added
RENDER_HW = (360, 480)     # camera render (H, W) -- native 480 px wide, no upscale blur
SCALE = 1.0
PANEL_H = 124

# (chip label, info/_granted key). LADDER = the rungs of the 'staged' ladder plus nested_v2,
# which is logged there and is the whole ladder under 'sparse'. LEGACY pays nothing.
# FAR/HOME are the Ladder-N rungs (2026-09-11): they light under every ladder because the
# tracker computes them under every ladder -- which is what lets one annotated clip answer
# 'what would the nested ladder have paid here'.
LADDER_CHIPS = [('PICK', 'picked'), ('PLACE', 'placed_v2'), ('PUSH', 'contact_push'),
                ('NEST2', 'nested_v2'), ('SLIDE', 'slide_success'),
                ('FAR', 'farside'), ('HOME', 'home')]
LEGACY_CHIPS = [('nestP', 'nested'), ('pushL', 'contact_push_legacy'),
                ('slidL', 'slide_success_legacy')]
ALL_CHIP_KEYS = [k for _, k in LADDER_CHIPS + LEGACY_CHIPS]
# Stages whose first grant decision is recorded per tape.
REPORT = ('picked', 'placed_v2', 'contact_push', 'slide_success', 'nested_v2',
          'released', 'pushed', 'farside', 'home', 'nested', 'contact', 'placed',
          'contact_push_legacy', 'slide_success_legacy')


# ----------------------------------------------------------------- uid resolution
def uid_map(set_dir, src_dir):
    """{segment basename: (rollout uid, ic_uid)}.

    The segment filename is `genesis-{uid:06d}-{k:03d}-{T}.npz` where `uid` is the SOURCE
    tape's own `uid` stamp and `k` its index in the WRITE PLAN (to_dreamer_native.py:294).
    The trial the start came from is the source tape's `ic_uid`, which the segment layout does
    not carry.

    `uid` is the authoritative key and its uniqueness in the source set is asserted. `k` is
    used only as a CROSS-CHECK, and only when the write plan cannot have been filtered
    (len(source) == len(segments)): `dDPfull_first` was written with --one-per-ic-first from a
    195-tape harvest, so there k indexes the surviving 72, not the sorted 195. The tape length
    is checked in both cases -- a wrong uid stamped on a video would be worse than no uid.
    """
    if not src_dir:
        return {}
    src = sorted(pl.Path(src_dir).glob('*.npz'))
    by_uid, out = {}, {}
    for i, sf in enumerate(src):
        z = np.load(sf, allow_pickle=True)
        u = int(scalar(z, 'uid'))
        assert u not in by_uid, f'duplicate uid {u} in {src_dir}'
        by_uid[u] = (i, int(scalar(z, 'ic_uid')), int(scalar(z, 'n')))
    segs = sorted(pl.Path(set_dir).glob('genesis-*.npz'))
    unfiltered = len(segs) == len(src)
    for f in segs:
        m = re.match(r'genesis-(\d{6})-(\d{3})-(\d+)\.npz$', f.name)
        if not m:
            continue
        u, k, T = int(m.group(1)), int(m.group(2)), int(m.group(3))
        if u not in by_uid:
            continue
        i, ic, n = by_uid[u]
        assert n + 1 == T, f'{f.name}: source n={n} but segment T={T}'
        if unfiltered:
            assert i == k, f'{f.name}: uid lookup gives index {i}, filename says {k}'
        out[f.name] = (u, ic)
    assert len(out) == len(segs), f'{len(segs) - len(out)} segment tape(s) unresolved in {src_dir}'
    return out


# ----------------------------------------------------------------- env
def build_env(sim_variant, max_sim_steps, ladder, with_video):
    """FullTaskEnv in the end-to-end contract MDP. Every knob asserted (the silent-default
    rule); identical to relabel_reward.build_env except for `render_size`, which is what makes
    the camera exist."""
    sys.path.insert(0, str(REPO / 'can_pos_recovery'))
    os.environ['GENESIS_SIM_VARIANT'] = sim_variant
    from sim_variant_hook import apply_pre, apply_post
    from full_env import FullTaskEnv, refuse_legacy_gates
    refuse_legacy_gates()
    apply_pre(sim_variant)
    t0 = time.time()
    env = FullTaskEnv(backend='cpu', max_steps=int(max_sim_steps), scope='full', ladder=ladder,
                      action_mode='delta_joint', delta_cap=0.025, delta_leash_mult=5.0,
                      action_repeat=4, delta_ref='target', camera_rig=False,
                      render_size=(RENDER_HW if with_video else None))
    apply_post(env, sim_variant)
    assert env.scope == 'full' and env.action_mode == 'delta_joint' and env.delta_ref == 'target'
    assert env.action_repeat == 4 and abs(env.delta_cap - 0.025) < 1e-12
    assert abs(env.delta_leash - 0.125) < 1e-12, env.delta_leash
    assert env.genv.max_steps >= 10 ** 8, 'inner env must never truncate (#26)'
    assert not env.goalward_shaping, 'an annotation must show the unshaped ladder'
    if with_video:
        assert env.genv.w.get('cam') is not None, 'render_size set but no camera in the world'
    print(f'[env] built in {time.time() - t0:.1f}s | variant {sim_variant} | ladder {env.ladder} '
          f'{env.stage_reward} terminal {env.terminal_stages}+tipped | shelf_top_z {env.shelf_top_z:.3f}',
          flush=True)
    return env


# ----------------------------------------------------------------- drawing
def _draw_panel(cv2, W, i, n_dec, dec, first, sticky, diag, staged_r, staged_max,
                sparse_r, sparse_dec, stride_note):
    F = cv2.FONT_HERSHEY_SIMPLEX
    pan = np.full((PANEL_H, W, 3), 22, np.uint8)

    def chips(items, x0, y0, bright):
        x = x0
        for label, key in items:
            on = key in sticky
            fresh = on and 0 <= dec - first.get(key, -10 ** 9) < 6
            if on:
                col = (90, 240, 90) if bright else (150, 150, 90)
            else:
                col = (85, 85, 85) if bright else (62, 62, 62)
            if fresh:
                col = (60, 255, 255)
            w_ = int((10 if bright else 8) * len(label)) + 12
            cv2.rectangle(pan, (x, y0), (x + w_, y0 + 20), col, -1 if on else 1)
            cv2.putText(pan, label, (x + 5, y0 + 15), F, 0.40 if bright else 0.34,
                        (20, 20, 20) if on else (165, 165, 165), 1, cv2.LINE_AA)
            # the grant decision is shown ONLY once it has happened. Printing a future
            # grant would both spoil the judgement the clip exists to support and read as
            # "already granted" on a frame where the chip is dark.
            if on and key in first:
                cv2.putText(pan, 'd%d' % first[key], (x + 5, y0 + 32), F, 0.31,
                            (150, 220, 150) if bright else (150, 150, 110), 1, cv2.LINE_AA)
            x += w_ + 6
        return x

    cv2.putText(pan, 'LADDER', (6, 16), F, 0.34, (210, 210, 210), 1, cv2.LINE_AA)
    chips(LADDER_CHIPS, 66, 4, True)
    # the legacy row sits behind a rule and is dim: it pays nothing and terminates nothing
    cv2.line(pan, (6, 42), (W - 6, 42), (70, 70, 70), 1)
    cv2.putText(pan, 'legacy', (6, 58), F, 0.32, (140, 140, 140), 1, cv2.LINE_AA)
    x_end = chips(LEGACY_CHIPS, 66, 46, False)
    cv2.putText(pan, '(pays 0)', (x_end + 4, 60), F, 0.30, (130, 130, 130), 1, cv2.LINE_AA)

    cv2.putText(pan, 'd%d/%d lever %.3f hand=%d rest=%d gain %.3f push=%d dist %.3f grip %.2f'
                % (dec, n_dec, diag['lever_m'], int(diag['in_hand']), int(diag['at_rest']),
                   diag['goalward_gain_m'], int(diag['pushed']), diag['dist_xy_m'], diag['grip_cmd']),
                (6, 87), F, 0.355, (205, 205, 205), 1, cv2.LINE_AA)
    sp = ('%.1f%s' % (sparse_r, '  (ENDS here)' if sparse_dec is not None and dec >= sparse_dec else '')
          if sparse_r else '0.0')
    cv2.putText(pan, 'staged paid %.1f/%.0f     sparse would pay %s' % (staged_r, staged_max, sp),
                (6, 103), F, 0.37, (120, 230, 255), 1, cv2.LINE_AA)
    if stride_note:     # right end of the LEGACY row: the only strip nothing else reaches
        # (the ladder row runs to the SLIDE chip; the reward line runs long when sparse
        # adds "(ENDS here)"; the diagnostics line fills its whole width)
        cv2.putText(pan, stride_note, (W - 132, 62), F, 0.32, (140, 140, 140), 1, cv2.LINE_AA)
    bar_y = PANEL_H - 7
    cv2.line(pan, (6, bar_y), (W - 6, bar_y), (70, 70, 70), 2)
    if n_dec > 0:
        px = int(6 + (W - 12) * min(dec, n_dec) / n_dec)
        for key, fd in first.items():
            if fd > dec:            # no future markers: same spoiler rule as the chips
                continue
            fx = int(6 + (W - 12) * min(fd, n_dec) / n_dec)
            cv2.line(pan, (fx, bar_y - 4), (fx, bar_y + 4),
                     (90, 240, 90) if key in [k for _, k in LADDER_CHIPS] else (140, 140, 90), 1)
        cv2.line(pan, (px, bar_y - 4), (px, bar_y + 4), (245, 245, 245), 2)
    return pan


def _terminal_card(cv2, im, row):
    """Dark card over the last sim frame: what ended the episode, the settled reference, and
    the recorded reward beside the re-executed one."""
    F = cv2.FONT_HERSHEY_SIMPLEX
    H, W = im.shape[:2]
    ov = im.copy()
    cv2.rectangle(ov, (0, 0), (W, H), (12, 12, 12), -1)
    im = cv2.addWeighted(ov, 0.80, im, 0.20, 0)
    g = row['grants']
    lines = [
        ('END: %s @ decision %d of %d' % (row['end_reason'], row['end_decision'], row['decisions']),
         (80, 230, 255)),
        ('reward  re-executed %.1f (%s)   recorded %.1f (old ladder)'
         % (row['reward_staged'], row['ladder'], row['reward_recorded']), (215, 215, 215)),
        ('sparse ladder would pay %.1f%s'
         % (row['reward_sparse'],
            '' if row['sparse_decision'] is None else ' @d%d' % row['sparse_decision']), (215, 215, 215)),
        ('', None),
        ('picked %d @d%s   placed_v2 %d @d%s' % (int('picked' in g), g.get('picked', '-'),
                                                 int('placed_v2' in g), g.get('placed_v2', '-')),
         (190, 235, 190)),
        ('contact_push %d @d%s   pushed %d @d%s' % (int('contact_push' in g), g.get('contact_push', '-'),
                                                    int(row['pushed']), g.get('pushed', '-')),
         (190, 235, 190)),
        ('nested_v2 %d @d%s   slide_success %d @d%s' % (int(row['nested_v2']), g.get('nested_v2', '-'),
                                                        int(row['slide_success']), g.get('slide_success', '-')),
         (190, 235, 190)),
        ('nested_honest (settled) %d        tipped %d' % (int(row['nested_honest']), int(row['tipped'])),
         (255, 220, 130)),
        ('legacy: nested_proxy %d   contact %d   pushL %d   slideL %d'
         % (int('nested' in g), int('contact' in g), int('contact_push_legacy' in g),
            int('slide_success_legacy' in g)), (150, 150, 150)),
    ]
    y = 30
    for txt, col in lines:
        if txt:
            cv2.putText(im, txt, (10, y), F, 0.40, col, 1, cv2.LINE_AA)
        y += 22
    return im


def write_mp4(cv2, path, frames, fps):
    """mp4v first, then ffmpeg -> H.264. cv2's 'avc1' fourcc is not available in every OpenCV
    build, and a silently-mpeg4 file would be a claim this file did not check."""
    tmp = str(path) + '.mp4v.mp4'
    H, W = frames[0].shape[:2]
    vw = cv2.VideoWriter(tmp, cv2.VideoWriter_fourcc(*'mp4v'), float(fps), (W, H))
    assert vw.isOpened(), f'cannot open writer for {tmp}'
    for fr in frames:
        vw.write(np.ascontiguousarray(fr))
    vw.release()
    r = subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', tmp,
                        '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '22',
                        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        os.replace(tmp, str(path))
        return 'mp4v (ffmpeg failed: %s)' % (r.stderr or '').strip()[:120]
    os.remove(tmp)
    return 'h264'


# ----------------------------------------------------------------- one tape
def annotate_one(env, path, out_mp4, ic_tol, ic_uid=None, set_name=''):
    import cv2
    z = np.load(path, allow_pickle=True)
    layout = tape_layout(z)
    _s0, acts, old_rew, _want = tape_stream(z, layout)
    n = int(acts.shape[0])
    recorded = float(np.asarray(old_rew, np.float64).sum())
    how, d_can, d_goal = reset_to_tape_ic(env, z)
    assert d_can <= ic_tol and d_goal <= ic_tol, (
        f'{os.path.basename(path)}: restored IC differs from the tape ({how}): '
        f'can {d_can * 1000:.1f} mm, goal {d_goal * 1000:.1f} mm > {ic_tol * 1000:.1f} mm')

    video = out_mp4 is not None
    # uniform body schedule over the decisions the tape CAN reach; forced frames are added live
    keep_plan = set(np.linspace(0, n, num=min(BODY_BUDGET, n + 1), dtype=int).tolist()) if n else {0}
    frames, snaps, diags, decs, rewards = [], [], [], [], []
    prev_drop = None          # the most recent NOT-kept frame, for the t-1 of a grant
    force_next = False

    def render():
        return np.asarray(env.genv.w['cam'].render()[0])[:, :, ::-1].astype(np.uint8)

    def push(dec, img, sticky, diag, staged_r):
        frames.append(img); snaps.append(frozenset(sticky)); diags.append(dict(diag))
        decs.append(dec); rewards.append(staged_r)

    diag0 = dict(lever_m=float('nan'), in_hand=False, at_rest=False, goalward_gain_m=0.0,
                 pushed=False, dist_xy_m=float('nan'), grip_cmd=float('nan'))
    if video:
        push(0, render(), set(), diag0, 0.0)
    first, grants = {}, {}
    staged_r, sparse_dec = 0.0, None
    end_reason, end_dec, tipped = 'stream_exhausted', n, False
    t0 = time.time()
    for t in range(n):
        obs, r, term, trunc, info = env.step(acts[t])
        staged_r += float(r)
        sticky = set(env._granted) | {k for k in ALL_CHIP_KEYS + list(REPORT) if info.get(k)}
        for k in sticky:
            if k in REPORT and k not in grants:
                grants[k] = t
        for k in ALL_CHIP_KEYS:
            if k in sticky and k not in first:
                first[k] = t
        if info.get('nested_v2') and sparse_dec is None:
            sparse_dec = t                      # sparse pays 1 and TERMINATES here
        tipped = tipped or bool(info.get('tipped'))
        tr = env.tracker
        diag = dict(lever_m=float(tr.lever_m), in_hand=bool(tr.in_hand), at_rest=bool(tr.at_rest),
                    goalward_gain_m=float(tr.goalward_gain_m), pushed=bool(tr.pushed),
                    dist_xy_m=float(tr.dist_xy_m),
                    grip_cmd=float((np.clip(float(acts[t][6]), -1.0, 1.0) + 1.0) / 2.0))
        if video:
            fired = any(first.get(k) == t for k in ALL_CHIP_KEYS)
            want = (t + 1) in keep_plan or fired or force_next or term or trunc or t == n - 1
            img = render() if want or fired else None
            if fired and prev_drop is not None:
                frames.append(prev_drop[0]); snaps.append(prev_drop[1]); diags.append(prev_drop[2])
                decs.append(prev_drop[3]); rewards.append(prev_drop[4])
            if want:
                push(t + 1, img if img is not None else render(), sticky, diag, staged_r)
                prev_drop = None
            else:
                # keep ONE frame in reserve so a grant can always show its predecessor
                im2 = render()
                prev_drop = (im2, frozenset(sticky), dict(diag), t + 1, staged_r)
            force_next = fired
        if term or trunc:
            end_reason = ('tipped' if info.get('tipped') else 'truncated' if trunc else
                          next((s for s in env.terminal_stages if info.get(s)), 'terminated'))
            end_dec = t + 1
            break
    end = env.genv.end_of_episode()      # ONE settle, after the last rendered frame
    gr = set(env._granted)
    row = dict(
        file=os.path.basename(path), set=set_name, ic_uid=ic_uid, decisions=n, layout=layout,
        ladder=str(env.ladder), ic=how, ic_can_mm=round(d_can * 1000, 2), ic_goal_mm=round(d_goal * 1000, 2),
        end_reason=end_reason, end_decision=int(end_dec), tipped=bool(tipped),
        reward_staged=float(staged_r), reward_recorded=recorded,
        reward_sparse=(1.0 if sparse_dec is not None else 0.0),
        sparse_decision=(None if sparse_dec is None else int(sparse_dec)),
        grants={k: int(v) for k, v in sorted(grants.items())},
        picked=bool('picked' in gr), placed_v2=bool('placed_v2' in gr),
        contact_push=bool('contact_push' in gr), nested_v2=bool('nested_v2' in gr),
        slide_success=bool('slide_success' in gr), pushed=bool(env.tracker.pushed),
        nested_proxy=bool('nested' in gr), contact=bool('contact' in gr),
        contact_push_legacy=bool('contact_push_legacy' in grants),
        slide_success_legacy=bool('slide_success_legacy' in grants),
        nested_honest=bool(end['nested']), slide_settle=bool(end['slide_success']),
        slide_route=str(end.get('slide_route')), seconds=round(time.time() - t0, 1))
    row['klass'] = classify(row)

    if video and frames:
        n_dec_axis = max(end_dec, 1)
        W = int(RENDER_HW[1] * SCALE) // 2 * 2
        H = int(RENDER_HW[0] * SCALE) // 2 * 2
        staged_max = float(sum(env.stage_reward.values()))
        kept = len(frames)
        stride_note = ('' if kept >= end_dec else 'subsampled %d/%d' % (kept, end_dec + 1))
        out = []
        for j, fr in enumerate(frames):
            im = cv2.resize(np.ascontiguousarray(fr), (W, H), interpolation=cv2.INTER_LINEAR)
            pan = _draw_panel(cv2, W, j, n_dec_axis, decs[j], first, snaps[j], diags[j],
                              rewards[j], staged_max, row['reward_sparse'], sparse_dec, stride_note)
            out.append(np.vstack([im, pan]))
        card = _terminal_card(cv2, out[-1][:H].copy(), row)
        for _ in range(CARD_FRAMES):
            out.append(np.vstack([card, out[-1][H:]]))
        assert len(out) <= MAX_FRAMES, f'{len(out)} frames > {MAX_FRAMES}'
        # preserve wall-clock: the episode runs at 30/repeat = 7.5 decisions per second
        fps = float(np.clip(7.5 * (end_dec + 1) / max(kept, 1), 6.0, 30.0))
        row['codec'] = write_mp4(cv2, out_mp4, out, fps)
        row['video'] = str(out_mp4)
        row['video_frames'] = len(out)
        row['video_fps'] = round(fps, 2)
        row['video_bytes'] = os.path.getsize(out_mp4)
    return row


def classify(r):
    """The stratification classes. Ordered, first match wins, so every tape has exactly one."""
    g = r['grants']
    if r['slide_success']:
        return 'slide'
    if not r['picked']:
        return 'nopick'
    if r['end_reason'] == 'tipped':
        return 'tipped'
    if r['nested_v2'] and not r['pushed']:
        return 'nested_drop'
    if r['nested_v2']:
        return 'nested_pushed'
    if r['contact'] and ('placed_v2' not in g or g['contact'] < g['placed_v2']):
        return 'carry_in'
    if r['contact_push']:
        return 'push_no_nest'
    if r['nested_proxy']:
        return 'proxy_only'
    if r['placed_v2']:
        return 'placed_only'
    return 'picked_only'


# ----------------------------------------------------------------- driver
def files_of(set_dir, names=None):
    fs = sorted(pl.Path(set_dir).glob('genesis-*.npz'))
    if names:
        want = {os.path.basename(x) for x in names}
        fs = [f for f in fs if f.name in want]
        missing = want - {f.name for f in fs}
        assert not missing, f'not in {set_dir}: {sorted(missing)}'
    assert fs, f'no tapes in {set_dir}'
    return fs


def run_shard(args, files, meta, umap):
    env = build_env(args.sim_variant or meta['sim_variant'], meta['max_sim_steps'],
                    args.ladder, args.cmd == 'render')
    rows = []
    for i, f in enumerate(files):
        u, ic = umap.get(f.name, (None, None))
        out_mp4 = None
        if args.cmd == 'render':
            pl.Path(args.out_dir).mkdir(parents=True, exist_ok=True)
            stem = f'{args.set_name}_{ic if ic is not None else f.stem}'
            out_mp4 = pl.Path(args.out_dir) / f'{stem}_PENDING.mp4'
        r = annotate_one(env, str(f), out_mp4, args.ic_tol, ic_uid=ic, set_name=args.set_name)
        r['rollout_uid'] = u
        if out_mp4 is not None:      # rename now that the class is known
            final = pl.Path(args.out_dir) / f'{args.set_name}_{ic if ic is not None else f.stem}_{r["klass"]}.mp4'
            os.replace(out_mp4, final)
            r['video'] = str(final); r['video_bytes'] = os.path.getsize(final)
        rows.append(r)
        print(f'[{i + 1}/{len(files)}] {f.name} uid={u} ic={ic}: {r["decisions"]} dec, '
              f'reward {r["reward_recorded"]:.1f} -> {r["reward_staged"]:.1f}, end {r["end_reason"]}'
              f'@{r["end_decision"]}, class {r["klass"]}, grants {r["grants"]}, '
              f'nestedH={int(r["nested_honest"])} [{r["seconds"]:.0f}s]', flush=True)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=('census', 'render'))
    ap.add_argument('--in', dest='inp', required=True, help='segment set dir (with repeat.json)')
    ap.add_argument('--src-dir', default=None,
                    help='contract-v1 SOURCE set, used only to resolve each segment tape to its '
                         'trial ic_uid (the segment layout does not carry one)')
    ap.add_argument('--out', default=None, help='census: JSON path')
    ap.add_argument('--out-dir', default=None, help='render: directory for the mp4s')
    ap.add_argument('--set-name', default='set', help='human | machine; prefixes the clip names')
    ap.add_argument('--tapes', nargs='*', default=None, help='render: segment basenames')
    ap.add_argument('--ladder', choices=('staged', 'sparse', 'nested_sparse', 'nested_ramp'),
                    default='staged')
    ap.add_argument('--sim-variant', default=None, help="default: the set's own stamp")
    ap.add_argument('--max-sim-steps', type=int, default=None)
    ap.add_argument('--ic-tol', type=float, default=0.002)
    ap.add_argument('--procs', type=int, default=1)
    ap.add_argument('--limit', type=int, default=None)
    ap.add_argument('--shard', type=int, default=None)
    ap.add_argument('--nshards', type=int, default=None)
    args = ap.parse_args()
    if args.cmd == 'census':
        assert args.out, 'census needs --out'
    else:
        assert args.out_dir, 'render needs --out-dir'

    files = files_of(args.inp, args.tapes)
    if args.limit:
        files = files[:int(args.limit)]
    meta = set_meta(args.inp, [str(f) for f in files], args)
    umap = uid_map(args.inp, args.src_dir)

    if args.shard is not None:
        mine = [f for i, f in enumerate(files) if i % args.nshards == args.shard]
        rows = run_shard(args, mine, meta, umap)
        json.dump(rows, open(f'{args.out or args.out_dir}/_shard{args.shard}.json'
                             if args.cmd == 'render' else f'{args.out}._shard{args.shard}.json', 'w'), indent=1)
        return

    n_proc = max(1, min(int(args.procs), 8, len(files)))
    if n_proc == 1:
        rows = run_shard(args, files, meta, umap)
    else:
        base = [sys.executable, os.path.abspath(__file__), args.cmd, '--in', args.inp,
                '--ladder', args.ladder, '--ic-tol', str(args.ic_tol), '--set-name', args.set_name,
                '--max-sim-steps', str(meta['max_sim_steps'])]
        if args.src_dir:
            base += ['--src-dir', args.src_dir]
        if args.out:
            base += ['--out', args.out]
        if args.out_dir:
            base += ['--out-dir', args.out_dir]
            pl.Path(args.out_dir).mkdir(parents=True, exist_ok=True)
        if args.tapes:
            base += ['--tapes'] + list(args.tapes)
        if args.limit:
            base += ['--limit', str(args.limit)]
        if args.sim_variant:
            base += ['--sim-variant', args.sim_variant]
        procs = [subprocess.Popen(base + ['--shard', str(i), '--nshards', str(n_proc)])
                 for i in range(n_proc)]
        rcs = [p.wait() for p in procs]
        assert all(rc == 0 for rc in rcs), f'shard failures: {rcs}'
        rows = []
        for i in range(n_proc):
            p = (f'{args.out_dir}/_shard{i}.json' if args.cmd == 'render'
                 else f'{args.out}._shard{i}.json')
            rows += json.load(open(p))
            os.remove(p)
    rows.sort(key=lambda r: r['file'])
    if args.cmd == 'census':
        klass = {}
        for r in rows:
            klass[r['klass']] = klass.get(r['klass'], 0) + 1
        per_rung = {k: sum(1 for r in rows if k in r['grants']) for k in REPORT}
        ends = {}
        for r in rows:
            ends[r['end_reason']] = ends.get(r['end_reason'], 0) + 1
        out = dict(set=args.set_name, source=os.path.abspath(args.inp), ladder=args.ladder,
                   sim_variant=args.sim_variant or meta['sim_variant'], n_tapes=len(rows),
                   reward_recorded_total=float(sum(r['reward_recorded'] for r in rows)),
                   reward_staged_total=float(sum(r['reward_staged'] for r in rows)),
                   reward_sparse_total=float(sum(r['reward_sparse'] for r in rows)),
                   tapes_granting=per_rung, end_reasons=ends, classes=klass,
                   nested_honest=sum(1 for r in rows if r['nested_honest']),
                   nested_v2=sum(1 for r in rows if r['nested_v2']),
                   rows=rows)
        json.dump(out, open(args.out, 'w'), indent=1)
        print('\n' + '=' * 76)
        print(f'set {args.set_name}: {len(rows)} tapes, ladder {args.ladder}')
        print(f'reward recorded {out["reward_recorded_total"]:.0f} -> re-executed {out["reward_staged_total"]:.0f} '
              f'(sparse would pay {out["reward_sparse_total"]:.0f})')
        print(f'tapes granting : { {k: v for k, v in per_rung.items() if v} }')
        print(f'end reasons    : {ends}')
        print(f'classes        : {klass}')
        print(f'nested_v2 {out["nested_v2"]}  nested_honest {out["nested_honest"]}')
        print(f'wrote {args.out}')
    else:
        tot = sum(r.get('video_bytes', 0) for r in rows)
        for r in rows:
            print(f'{os.path.basename(r["video"])}  {r.get("video_bytes", 0) / 1e6:.1f} MB  '
                  f'{r.get("video_frames")} frames @ {r.get("video_fps")} fps  [{r.get("codec")}]')
        print(f'total {tot / 1e6:.1f} MB in {args.out_dir}')
        json.dump(rows, open(os.path.join(args.out_dir, f'_rendered_{args.set_name}.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
