"""Unit tests for the 2026-08-23 demo-set protections (PREREG §4.1/§4.3, build item B2):

  * full_env.terminal_from_tape on legacy and contract-v1 tapes
  * train_sacfd_full.relabel_full / delta_encode_transitions with --demo-terminal-guard
    on (tip -> done=True r=0, later frames dropped; pick -> done=True, later dropped;
    timeout -> done=False) and off (pre-08-23 output)
  * train_sacfd_full.native_demo_transitions incl. demo shaping with phi(terminal)=0
  * make_dDPsucc-style per-frame `terminated` flags

Pure numpy: genesis / torch / gymnasium are STUBBED (the modules under test import them
at module level; nothing in the tested functions touches them). Run with any python that
has numpy:  ~/.wandb-venv/bin/python baselines/tests/test_terminal_guard.py
"""
import os
import sys
import types
import pathlib as pl
import numpy as np

REPO = pl.Path(__file__).resolve().parents[2]
os.environ.setdefault('GENESIS_PICKAPLACE_ROOT', str(REPO))


def _stub(name, **attrs):
    m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m


for _mod in ('genesis', 'torch'):
    if _mod not in sys.modules:
        try:
            __import__(_mod)
        except ImportError:
            _stub(_mod, Tensor=type('Tensor', (), {}))
try:
    import gymnasium  # noqa: F401
except ImportError:
    _gym = _stub('gymnasium', Env=type('Env', (), {'__init__': lambda self: None,
                                                  'reset': lambda self, seed=None, options=None: None}))
    _gym.spaces = _stub('gymnasium.spaces', Box=lambda *a, **k: None)
for _p in ('baselines', 'baselines/rl', 'can_pos_recovery'):
    sys.path.insert(0, str(REPO / _p))
try:
    import kinova  # noqa: F401
except ImportError:
    _stub('kinova', JOINT_NAMES=[], EEF_NAME='')

import full_env  # noqa: E402
import train_sacfd_full as T  # noqa: E402

PICK_Z = 0.1505
UPRIGHT = np.array([1.0, 0.0, 0.0, 0.0])
TIPPED = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])   # 90 deg about x


def legacy_tape(n_frames, pick_at=None, tip_at=None, stage='no-pick', grip_closed_from=None,
                tmp=None, name='t.npz', extra=None):
    """states (n,17), actions (n,7): can on the table, optionally lifted from frame
    pick_at (grip commanded closed from grip_closed_from or pick_at), optionally tipped
    (grip open) from frame tip_at. Written to an npz so the loaders see a real file."""
    s = np.zeros((n_frames, 17), dtype=np.float32)
    a = np.zeros((n_frames, 7), dtype=np.float32)
    s[:, :6] = np.linspace(0, 0.1, n_frames)[:, None]          # slowly moving arm
    a[:, :6] = s[:, :6] + 0.001                                  # command leads pose
    s[:, 8:11] = (0.6, -0.1, 0.04)
    s[:, 11:15] = UPRIGHT
    s[:, 15:17] = (0.672, -0.221)
    a[:, 6] = 0.0                                                # grip open
    if pick_at is not None:
        g0 = pick_at if grip_closed_from is None else grip_closed_from
        a[g0:, 6] = 1.0
        s[pick_at:, 10] = PICK_Z + 0.02
    if tip_at is not None:
        s[tip_at:, 11:15] = TIPPED
    d = dict(states=s, actions=a, uid=np.array(1), n=np.array(n_frames),
             label=np.array('success' if pick_at is not None else 'fail'), stage=np.array(stage))
    if extra:
        d.update(extra)
    path = pl.Path(tmp) / name
    np.savez_compressed(path, **d)
    return str(path)


def v1_tape(tmp, name='v1.npz', n=5, terminal='pick', shaping_geometry=None):
    s = np.zeros((n, 17), dtype=np.float32); fs = np.zeros(17, dtype=np.float32)
    s[:, 8:11] = (0.6, -0.1, 0.04); fs[8:11] = (0.6, -0.1, 0.04)
    s[:, 11:15] = UPRIGHT; fs[11:15] = UPRIGHT
    a = np.zeros((n, 7), dtype=np.float32); a[:, 0] = 0.5; a[:, 6] = -1.0
    r = np.zeros(n, dtype=np.float32); term = np.zeros(n, dtype=bool); trunc = np.zeros(n, dtype=bool)
    picked = np.zeros(n, dtype=bool); tipped = np.zeros(n, dtype=bool)
    if terminal == 'pick':
        r[-1] = 1.0; term[-1] = True; picked[-1] = True; fs[10] = PICK_Z + 0.02
    elif terminal == 'tip':
        term[-1] = True; tipped[-1] = True; fs[11:15] = TIPPED
    else:
        trunc[-1] = True
    # eef walks toward the can: distance 0.5, 0.4, ..., so phi is hand-computable
    eef = np.zeros((n + 1, 3), dtype=np.float32)
    eef[:, 0] = 0.6 - np.linspace(0.5, 0.5 - 0.1 * n, n + 1)
    eef[:, 1] = -0.1; eef[:, 2] = 0.04
    d = dict(states=s, final_state=fs, actions_delta=a, actions=np.zeros((n, 7), np.float32),
             rewards=r, terminated=term, truncated=trunc, picked=picked, tipped=tipped,
             eef_pos=eef, images=np.zeros((n + 1, 2, 2, 6), np.uint8),
             uid=np.array(100000), ic_uid=np.array(232), label=np.array('success' if terminal == 'pick' else 'fail'),
             stage=np.array('picked' if terminal == 'pick' else 'no-pick'), teacher=np.array('test'),
             action_repeat=np.array(4), delta_cap=np.array(0.025), delta_leash=np.array(0.125),
             delta_ref=np.array('target'), contract=np.array('v1'), n=np.array(n))
    path = pl.Path(tmp) / name
    np.savez_compressed(path, **d)
    return str(path), d


def main():
    import tempfile
    tmp = tempfile.mkdtemp(prefix='tguard_')
    EXPECT = dict(action_repeat=4, delta_cap=0.025, delta_leash=0.125, delta_ref='target')

    # ---- 1. legacy TIPPED fail tape: grip open, can tips from frame 20 of 30 --------
    p = legacy_tape(30, tip_at=20, tmp=tmp, name='tip.npz')
    tm = full_env.terminal_from_tape(np.load(p, allow_pickle=True), pick_z=PICK_Z, scope='pick')
    assert tm == dict(t_term=19, kind='tip', reward=0.0, layout='legacy'), tm
    on, st_on = T.relabel_full([p], PICK_Z, scope='pick', terminal_guard=True)
    off, st_off = T.relabel_full([p], PICK_Z, scope='pick', terminal_guard=False)
    assert len(on) == 20 and on[-1][4] is True and on[-1][2] == 0.0, (len(on), on[-1][2:])
    assert not any(t[4] for t in on[:-1])
    assert len(off) == 29 and not any(t[4] for t in off), len(off)     # pre-08-23: whole, no done
    assert st_on['guard_tip'] == 1 and st_on['guard_frames_dropped'] == 9, st_on
    # encoder: prefix bit-identical, guard drops the tail, done flips on the cut row
    e_on = T.delta_encode_transitions([p], PICK_Z, 'pick', 0.025, terminal_guard=True)
    e_off = T.delta_encode_transitions([p], PICK_Z, 'pick', 0.025, terminal_guard=False)
    assert len(e_on) == 20 and len(e_off) == 29
    for i in range(19):
        assert np.array_equal(e_on[i][1], e_off[i][1]) and e_on[i][4] == e_off[i][4] is False
    assert e_on[19][4] is True and e_off[19][4] is False
    e_rep = T.delta_encode_transitions_repeat([p], PICK_Z, 'pick', 0.025, 4, terminal_guard=True)
    assert len(e_rep) == 5 and e_rep[-1][4] is True and sum(t[2] for t in e_rep) == 0.0, len(e_rep)
    print('1. tipped fail tape: guard on -> 20 transitions, done@19 r=0; off -> 29, no done  OK')

    # ---- 2. legacy PICK tape (scope pick): lift from frame 10 of 30, grip closed --------
    p = legacy_tape(30, pick_at=10, stage='picked', tmp=tmp, name='pick.npz')
    on, st_on = T.relabel_full([p], PICK_Z, scope='pick', terminal_guard=True)
    off, _ = T.relabel_full([p], PICK_Z, scope='pick', terminal_guard=False)
    assert len(on) == 11 and on[-1][4] is True and on[-1][2] == 1.0, (len(on), on[-1][2:])
    assert len(off) == 29 and not any(t[4] for t in off) and off[10][2] == 1.0
    assert st_on['guard_pick'] == 1 and st_on['guard_frames_dropped'] == 18, st_on
    e_on = T.delta_encode_transitions([p], PICK_Z, 'pick', 0.025, terminal_guard=True)
    e_off = T.delta_encode_transitions([p], PICK_Z, 'pick', 0.025, terminal_guard=False)
    assert len(e_on) == 11 and e_on[-1][4] is True
    assert len(e_off) == 29 and e_off[10][4] is True and e_off[11][4] is False   # old: done marked, tail kept
    # scope='full' (the dreamer converters): the pick does NOT cut, only tips do
    full_on, _ = T.relabel_full([p], PICK_Z, scope='full', terminal_guard=True)
    assert len(full_on) == 29 and not any(t[4] for t in full_on)
    print('2. pick tape: guard on -> 11 transitions, done@10 r=1; off -> 29 (done marked, tail kept)  OK')

    # ---- 3. timeout tape: neither pick nor tip -> bootstrap (done=False), both modes --
    p = legacy_tape(30, tmp=tmp, name='timeout.npz')
    on, st_on = T.relabel_full([p], PICK_Z, scope='pick', terminal_guard=True)
    off, _ = T.relabel_full([p], PICK_Z, scope='pick', terminal_guard=False)
    assert len(on) == len(off) == 29 and not any(t[4] for t in on) and st_on['guard_none'] == 1
    tm = full_env.terminal_from_tape(np.load(p, allow_pickle=True), pick_z=PICK_Z, scope='pick')
    assert tm['t_term'] is None and tm['kind'] == 'none'
    print('3. timeout tape: 29 transitions, no done, both modes  OK')

    # ---- 4. pick THEN tip (can dropped after lift, grip opened): earliest wins = pick ----
    p = legacy_tape(30, pick_at=10, tip_at=20, stage='picked', tmp=tmp, name='pick_tip.npz')
    d = np.load(p, allow_pickle=True)
    a = d['actions'].copy(); a[20:, 6] = 0.0                      # grip opens at the tip
    np.savez_compressed(p, **{k: (a if k == 'actions' else d[k]) for k in d.files})
    on, _ = T.relabel_full([p], PICK_Z, scope='pick', terminal_guard=True)
    assert len(on) == 11 and on[-1][4] is True and on[-1][2] == 1.0
    full_on, st = T.relabel_full([p], PICK_Z, scope='full', terminal_guard=True)
    # grip opens AT frame 20 (a[20,6]=0) and s[21] is tipped -> env-consistent tip
    # transition is 20 (the env checks the command just applied + the pose after it)
    assert len(full_on) == 21 and full_on[-1][4] is True and st['guard_tip'] == 1, (len(full_on), st)
    print('4. pick-then-tip: scope pick ends at the pick (11), scope full at the tip (21)  OK')

    # ---- 5. hold-region relabel: tipped negative tape is cut at the tip ----
    p = legacy_tape(40, tip_at=25, tmp=tmp, name='hold_tip.npz')
    tr, c = T.relabel_hold_region([p], PICK_Z, 5, terminal_guard=True)
    assert len(tr) == 25 and tr[-1][4] is True and c['guard_tip'] == 1, (len(tr), c['guard_tip'])
    tr_off, _ = T.relabel_hold_region([p], PICK_Z, 5, terminal_guard=False)
    assert len(tr_off) == 39 and not any(t[4] for t in tr_off)
    print('5. hold-region: tipped negative cut at 25 with done; off 39  OK')

    # ---- 6. make_dDPsucc-style per-frame terminated flags (tiptrunc) ----
    n = 21
    flags = np.zeros(n, dtype=bool); flags[-1] = True
    p = legacy_tape(n, tip_at=20, tmp=tmp, name='tiptrunc.npz',
                    extra=dict(terminated=flags, tipped=flags.copy(), terminal_kind=np.array('tip')))
    tm = full_env.terminal_from_tape(np.load(p, allow_pickle=True), pick_z=PICK_Z, scope='pick')
    assert tm['t_term'] == 19 and tm['kind'] == 'tip', tm
    on, _ = T.relabel_full([p], PICK_Z, scope='pick', terminal_guard=True)
    assert len(on) == 20 and on[-1][4] is True
    print('6. tiptrunc per-frame flags -> terminal transition 19, done  OK')

    # ---- 7. contract-v1 tapes: native transitions + shaping hand-check ----
    p_pick, d = v1_tape(tmp, 'v1_pick.npz', n=5, terminal='pick')
    tm = full_env.terminal_from_tape(np.load(p_pick, allow_pickle=True))
    assert tm == dict(t_term=4, kind='pick', reward=1.0, layout='v1'), tm
    tr, c = T.native_demo_transitions([p_pick], EXPECT, shaping=False)
    assert len(tr) == 5 and tr[-1][4] is True and tr[-1][2] == 1.0 and not any(t[4] for t in tr[:-1])
    assert np.array_equal(tr[2][3], d['states'][3]) and np.array_equal(tr[4][3], d['final_state'])
    assert np.array_equal(tr[1][1], d['actions_delta'][1])
    assert c['n_success'] == 1 and c['n_transitions'] == 5 and c['first_lift'] == [4]
    # shaping: eef-can distance = 0.5, 0.4, 0.3, 0.2, 0.1, 0.0 -> phi = -1.0, -0.8, ..., 0
    gamma = 0.998
    tr_s, cs = T.native_demo_transitions([p_pick], EXPECT, gamma=gamma, shaping=True)
    phi = [-2.0 * dd for dd in (0.5, 0.4, 0.3, 0.2, 0.1, 0.0)]
    for t in range(4):
        want = gamma * phi[t + 1] - phi[t]
        assert abs(tr_s[t][2] - want) < 1e-6, (t, tr_s[t][2], want)
    # terminal row: phi(terminal) = 0 -> F = 0 - phi[4] = +0.2, plus the +1 pick
    assert abs(tr_s[4][2] - (1.0 + (0.0 - phi[4]))) < 1e-6, tr_s[4][2]
    assert abs(cs['shaping_sum'] - sum(gamma * phi[t + 1] - phi[t] for t in range(4)) - (0.0 - phi[4])) < 1e-6
    # tip-terminated v1 tape: done=True, r=0, counted as tip
    p_tip, _ = v1_tape(tmp, 'v1_tip.npz', n=4, terminal='tip')
    tr, c = T.native_demo_transitions([p_tip], EXPECT)
    assert len(tr) == 4 and tr[-1][4] is True and tr[-1][2] == 0.0 and c['n_tip'] == 1 and c['n_fail'] == 1
    # truncated v1 tape: done=False on the last row, counted as truncated
    p_tr, _ = v1_tape(tmp, 'v1_trunc.npz', n=4, terminal='trunc')
    tr, c = T.native_demo_transitions([p_tr], EXPECT)
    assert not any(t[4] for t in tr) and c['n_truncated'] == 1
    # stamp mismatch is refused
    try:
        T.native_demo_transitions([p_pick], dict(EXPECT, action_repeat=1)); raise SystemExit('stamp check failed to fire')
    except AssertionError as e:
        assert 'action_repeat' in str(e)
    print('7. contract-v1: native transitions, shaping (phi(terminal)=0), tip/trunc, stamp gate  OK')

    # ---- 8. pick_shaping_phi is the single definition ----
    assert full_env.pick_shaping_phi((0, 0, 0), (0.3, 0.4, 0)) == -2.0 * 0.5
    assert full_env.pick_shaping_phi((0, 0, 0), (0.3, 0.4, 0), terminal=True) == 0.0
    assert full_env.FullTaskEnv._pick_phi is not None
    sha = T.demo_dir_sha256([p_pick, p_tip])
    assert len(sha) == 64 and sha == T.demo_dir_sha256([p_tip, p_pick])
    print('8. phi definition + demo sha  OK')
    print('ALL OK')


if __name__ == '__main__':
    main()
