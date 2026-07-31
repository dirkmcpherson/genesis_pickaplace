"""Relabel episodes_cartesian -> forward SACfD transitions with staged rewards.

Cartesian twin of train_sacfd_full.relabel_full: same staged proxies + manifest
gating + tip rule as to_dreamer_demos_cartesian (the reward logic validated there),
but emitting SB3 (obs, action, reward, next_obs, done) tuples with PHYSICAL
actions (inject_into_replay_buffer applies normalize_action).

Layout: state 18 = [ee3, eq4, grip, effort, can3, canq4, goal2]; action 5 =
[vx,vy,vz,wy, grip01]. can_z=s[:,11]; grip cmd=a[:,4].
Tip rule: episode truncates at the first tilt>60 & grip-open frame, done=True,
reward -0.5 there (mirrors CartesianFullTaskEnv).
"""
import json
import pathlib as pl

import numpy as np

REPO = pl.Path(__file__).resolve().parents[2]

PICK_Z_DEFAULT = 0.1505
SHELF_LO, SHELF_HI = 0.16, 0.30
TOUCH_XY = 0.081
GRIP_CLOSED = 0.5
STAGE_RANK = {'no-pick': 0, 'picked': 1, 'placed': 2, 'contact': 3, 'nested': 4}
STAGE_REWARD = {'picked': 1.0, 'placed': 1.0, 'contact': 2.0, 'nested': 4.0}
TIP_DEG, TIP_PENALTY, GRIP_OPEN = 60.0, 0.0, 0.3


def relabel_cartesian(paths, pick_z=PICK_Z_DEFAULT):
    man = json.loads((REPO / 'baselines/demo_manifest_auth.json').read_text())
    transitions, stats = [], {k: 0 for k in STAGE_REWARD}
    stats['tipped'] = 0
    for p in paths:
        d = np.load(p, allow_pickle=True)
        uid = int(d['uid'])
        stage = man.get(str(uid), {}).get('stage', 'no-pick')
        rank = STAGE_RANK.get(stage, 0)
        s = d['states'].astype(np.float32)
        a = d['actions'].astype(np.float32)
        # Normalize both npz layouts to len(s) == len(a)+1 (transition-complete):
        # realized store n+1 states / n actions already; commanded store n/n --
        # drop the final action (it has no successor state).
        if len(a) == len(s):
            a = a[:-1]
        assert len(s) == len(a) + 1, (len(s), len(a))
        n = len(a)                       # transitions
        q = s[:n, 12:16]
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
        # grip column depends on the action encoding: 5-dim (vel/delta/abs) -> 4,
        # 7-dim (abs6/delta6: pos3+rot3+grip) -> 6. Reading 4 on a 7-dim action uses
        # a ROTATION component as the gripper (audit finding, 2026-07-30: shifted the
        # tip truncation by 8 frames and corrupted picked_f for 6-DOF demo sets).
        gidx = 6 if a.shape[1] >= 7 else 4
        tipped_free = (tilt > TIP_DEG) & (a[:, gidx] < GRIP_OPEN)
        j_tip = int(np.argmax(tipped_free)) if tipped_free.any() else -1
        if j_tip >= 0:
            s, a = s[:j_tip + 2], a[:j_tip + 1]
            n = len(a)
        if n < 2:
            continue
        can_z = s[:, 11]
        grip = a[:, gidx]
        rew = np.zeros(n, dtype=np.float32)
        done = np.zeros(n, dtype=bool)
        picked_f = (can_z[:n] > pick_z) & (grip > GRIP_CLOSED)
        j_pick = int(np.argmax(picked_f)) if picked_f.any() and rank >= 1 else -1
        if j_pick == 0:
            continue
        if j_pick > 0:
            rew[j_pick] += STAGE_REWARD['picked']; stats['picked'] += 1
            if rank >= 2:
                pl_f = (np.arange(n) > j_pick) & (can_z[:n] > SHELF_LO) & (can_z[:n] < SHELF_HI)
                j_pl = int(np.argmax(pl_f)) if pl_f.any() else -1
                if j_pl > 0:
                    rew[j_pl] += STAGE_REWARD['placed']; stats['placed'] += 1
                    if rank >= 3:
                        dxy = np.hypot(s[:n, 9] - s[:n, 16], s[:n, 10] - s[:n, 17])
                        c_f = (np.arange(n) > j_pl) & (dxy < TOUCH_XY)
                        j_c = int(np.argmax(c_f)) if c_f.any() else -1
                        if j_c > 0:
                            rew[j_c] += STAGE_REWARD['contact']; stats['contact'] += 1
                            if rank >= 4:
                                rew[n - 1] += STAGE_REWARD['nested']; stats['nested'] += 1
                                done[n - 1] = True
        if j_tip >= 0 and not done[n - 1]:
            rew[n - 1] += TIP_PENALTY
            done[n - 1] = True
            stats['tipped'] += 1
        for i in range(n):
            transitions.append((s[i], a[i], float(rew[i]), s[i + 1], bool(done[i])))
    return transitions, stats
