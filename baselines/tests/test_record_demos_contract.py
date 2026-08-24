"""Contract-v1 validator tests (numpy only; no genesis).

Run:  ~/.wandb-venv/bin/python baselines/tests/test_record_demos_contract.py
  or: python -m pytest baselines/tests/test_record_demos_contract.py
"""
import os
import sys
import pathlib as pl

import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parents[1]))
from record_demos import validate_tape, CONTRACT, REPEAT, DELTA_CAP, ROLLOUT_BASE  # noqa: E402


def synth(n=12, picked=True, tipped=False, truncated=False, images=True, sim=True, uid=100007):
    """A well-formed contract-v1 tape dict."""
    assert not (picked and truncated)
    S = np.zeros((n, 17), np.float32); S[:, 10] = 0.05
    A = np.clip(np.random.default_rng(0).normal(0, 0.3, (n, 7)), -1, 1).astype(np.float32)
    C = np.zeros((n, 7), np.float32); C[:, 6] = 0.5
    term = np.zeros(n, bool); trunc = np.zeros(n, bool); pk = np.zeros(n, bool); tp = np.zeros(n, bool)
    if picked:
        term[-1] = True; pk[-1] = True
    elif tipped:
        term[-1] = True; tp[-1] = True
    else:
        trunc[-1] = True
    d = dict(states=S, final_state=S[-1].copy(), actions_delta=A, actions=C,
             rewards=np.zeros(n, np.float32), terminated=term, truncated=trunc, picked=pk, tipped=tp,
             eef_pos=np.zeros((n + 1, 3), np.float32), uid=uid, ic_uid=232,
             label='success' if picked else 'fail', stage='picked' if picked else 'none',
             teacher='random', teacher_ckpt='', act_mode='sample', action_repeat=REPEAT,
             delta_cap=DELTA_CAP, delta_leash=5 * DELTA_CAP, delta_ref='target', pick_z=0.1505, n=n,
             git_sha='abc', env_class='FullTaskEnv', recorder='record_demos.py v1', contract=CONTRACT,
             end_reason='terminated' if (picked or tipped) else 'env_truncated')
    if images:
        d['images'] = np.zeros((n + 1, 64, 64, 6), np.uint8)
    if sim:
        d['sim_states'] = np.zeros((4 * n - 2, 17), np.float32)
        d['sim_actions'] = np.zeros((4 * n - 2, 7), np.float32)
    return d


def synth_full(n=20, nested_at=None, picked_at=3, tipped=False, truncated=False, uid=100021):
    """A well-formed FULL-SCOPE contract tape: sticky stage flags, label=success iff nested."""
    d = synth(n=n, picked=False, tipped=tipped, truncated=(nested_at is None and not tipped) or truncated,
              images=False, sim=False, uid=uid)
    for k, at in (('picked', picked_at), ('placed', picked_at + 4 if (nested_at or n > picked_at + 8) else None),
                  ('contact', nested_at - 1 if nested_at else None), ('nested', nested_at)):
        f = np.zeros(n, bool)
        if at is not None and at < n:
            f[at:] = True
        d[k] = f
    if nested_at is not None:
        d['terminated'] = np.zeros(n, bool); d['terminated'][-1] = True
        d['truncated'] = np.zeros(n, bool)
        assert d['nested'][-1]
    d['scope'] = 'full'; d['max_sim_steps'] = 2400
    deepest = next((st for st in ('nested', 'contact', 'placed', 'picked') if d[st][-1]), None)
    d['label'] = 'success' if d['nested'][-1] else 'fail'
    d['stage'] = deepest or 'none'
    d['end_reason'] = 'terminated' if d['terminated'][-1] else 'env_truncated'
    return d


def test_full_scope_validator():
    from record_demos import validate_tape
    ok = synth_full(nested_at=18)
    assert validate_tape(npz_roundtrip(ok)) == [], validate_tape(npz_roundtrip(ok))
    # partial: picked, never nested, truncated -> label fail, stage placed
    part = synth_full(nested_at=None)
    assert part['label'] == 'fail' and part['stage'] in ('picked', 'placed')
    assert validate_tape(npz_roundtrip(part)) == [], validate_tape(npz_roundtrip(part))
    # picked BEFORE the last row is legal in full scope (illegal in pick scope)
    assert part['picked'][:-1].any()
    # violations
    bad = synth_full(nested_at=18); bad['label'] = 'fail'
    assert any('label/nested' in e for e in validate_tape(npz_roundtrip(bad)))
    bad = synth_full(nested_at=18); bad['stage'] = 'picked'
    assert any('deepest' in e for e in validate_tape(npz_roundtrip(bad)))
    bad = synth_full(nested_at=None)
    if bad['placed'].any():
        f = bad['placed'].copy(); f[int(np.argmax(f)) + 1] = False; bad['placed'] = f
        assert any('sticky' in e for e in validate_tape(npz_roundtrip(bad)))
    bad = dict(synth_full(nested_at=None)); bad['n'] = 700  # > 2400//4+1
    assert any('max_sim_steps' in e for e in validate_tape(npz_roundtrip(bad)))
    print('checked full-scope validator paths')


def test_recorder_full_scope():
    from record_demos import Recorder, RandomTeacher
    env = _FakeEnv(pick_at=3)
    env.scope = 'full'
    def _step_once(a, _env=env):
        _env._dj_target = _env._dj_target + np.clip(a[:6], -1, 1) * 0.025
        _env.genv.q = 0.5 * _env.genv.q + 0.5 * _env._dj_target
        _env._t += 1
        info = {}; term = False; r = 0.0
        if _env._t >= 13:
            info['picked'] = True
            if 'picked' not in _env._granted: r += 1.0; _env._granted.add('picked')
        if _env._t >= 21:
            info['placed'] = True
            if 'placed' not in _env._granted: r += 1.0; _env._granted.add('placed')
        if _env._t >= 33:
            info['nested'] = True
            if 'nested' not in _env._granted: r += 4.0; _env._granted.add('nested')
            term = True
        trunc = (not term) and _env._t >= _env.max_steps
        return _env.genv.state(), r, term, trunc, info
    env._step_once = _step_once
    tape, r = Recorder(env, images=False, sim_tape=False).run(RandomTeacher(0), dict(uid=232))
    assert tape['label'] == 'success' and tape['stage'] == 'nested', (tape['label'], tape['stage'])
    assert tape['picked'][3] and not tape['picked'][2] and tape['nested'][-1] and tape['terminated'][-1]
    assert r['grant_decisions']['picked'] == 3 and r['grant_decisions']['nested'] == len(tape['picked']) - 1
    assert r['outcome'] == 'nested'
    print('checked full-scope recorder path')


def npz_roundtrip(d):
    import io
    buf = io.BytesIO(); np.savez(buf, **d); buf.seek(0)
    return np.load(buf, allow_pickle=True)


def test_valid_variants():
    assert validate_tape(synth()) == []
    assert validate_tape(synth(picked=False, tipped=True)) == []
    assert validate_tape(synth(picked=False, truncated=True)) == []
    assert validate_tape(synth(images=False, sim=False)) == []
    assert validate_tape(npz_roundtrip(synth()), require_images=True, require_sim=True) == []


def test_requirements():
    assert 'images required but absent' in validate_tape(synth(images=False), require_images=True)
    assert 'sim_* required but absent' in validate_tape(synth(sim=False), require_sim=True)


def test_violations():
    d = synth(); d['label'] = 'fail'
    assert any('label/picked' in e for e in validate_tape(d))
    d = synth(); d['terminated'][3] = True
    assert any('terminated=True before' in e for e in validate_tape(d))
    d = synth(); d['terminated'][-1] = False            # picked but not terminated / no end flag
    errs = validate_tape(d); assert any('exactly one of' in e for e in errs) and any('not terminated' in e for e in errs)
    d = synth(); d['actions_delta'][0, 0] = 1.5
    assert any('outside [-1,1]' in e for e in validate_tape(d))
    d = synth(); d['picked'][2] = True
    assert any('picked=True before' in e for e in validate_tape(d))
    d = synth(uid=232)
    assert any('rollout index' in e for e in validate_tape(d))
    d = synth(); d['contract'] = 'v0'
    assert any('contract' in e for e in validate_tape(d))
    d = synth(); d['action_repeat'] = 1
    assert any('action_repeat' in e for e in validate_tape(d))
    d = synth(); d['eef_pos'] = d['eef_pos'][:-1]
    assert any('eef_pos shape' in e for e in validate_tape(d))
    d = synth(); d['images'] = d['images'][:-1]
    assert any('images shape' in e for e in validate_tape(d))
    d = synth(); d['sim_states'] = np.zeros((5 * 12, 17), np.float32); d['sim_actions'] = np.zeros((5 * 12, 7), np.float32)
    assert any('sim tape length' in e for e in validate_tape(d))
    d = synth(); del d['eef_pos']
    assert validate_tape(d) == ['missing key eef_pos']
    d = synth(); d['truncated'][-1] = True               # both terminated and truncated on the last row
    assert any('exactly one of' in e for e in validate_tape(d))




# ----------------------------------------------------------------- Recorder on a fake env
class _FakeGenv:
    def __init__(self):
        self.q = np.zeros(6); self.can_z = 0.05
    def _calib_tool_offset(self): pass
    def tool_pos(self): return np.array([0.5, 0.0, 0.2])
    def rig_obs(self): return np.zeros((64, 64, 6), np.uint8)
    def state(self):
        s = np.zeros(17, np.float32); s[:6] = self.q; s[10] = self.can_z; return s
    def _obs(self): return dict(state=self.state())


class _FakeEnv:
    """Mimics the FullTaskEnv surface Recorder uses: target-ref delta integration,
    terminate with picked at decision `pick_at` (sim step 4*pick_at+1), truncate at 1200."""
    def __init__(self, pick_at=None, tip_at=None):
        self.genv = _FakeGenv(); self.pick_z = 0.1505; self.success_uids = [232, 234]
        self.delta_cap = 0.025; self.delta_leash = 0.125; self.action_repeat = 4; self.max_steps = 1200
        self.pick_shaping = False; self.pick_hold_reward = False
        self.pick_at = pick_at; self.tip_at = tip_at
    def reset(self, options=None):
        self._t = 0; self._granted = set(); self.genv.q[:] = 0; self.genv.can_z = 0.05
        self._dj_target = np.zeros(6); return self.genv.state(), {'uid': options['uid']}
    def reset_to(self, ic): return self.reset(options={'uid': -1})
    def _step_once(self, a):
        self._dj_target = self._dj_target + np.clip(a[:6], -1, 1) * self.delta_cap
        self.genv.q = 0.5 * self.genv.q + 0.5 * self._dj_target          # lazy PD
        self._t += 1
        info = {}; term = False; r = 0.0
        if self.pick_at is not None and self._t >= 4 * self.pick_at + 1:
            info['picked'] = True; self._granted.add('picked'); term = True; r = 1.0
        if self.tip_at is not None and self._t >= self.tip_at:
            info['tipped'] = True; term = True
        trunc = (not term) and self._t >= self.max_steps
        return self.genv.state(), r, term, trunc, info
    def step(self, a):
        for _ in range(4):
            o, r, term, trunc, info = self._step_once(a)
            if term or trunc: break
        return o, r, term, trunc, info


def test_recorder_fake_env():
    from record_demos import Recorder, RandomTeacher, HumanFollower
    rec = Recorder(_FakeEnv(pick_at=5), images=True, sim_tape=True)
    tape, r = rec.run(RandomTeacher(0), dict(uid=232))
    assert tape['n'] == 6 and tape['terminated'][-1] and tape['picked'][-1] and tape['label'] == 'success'
    assert tape['sim_states'].shape[0] == 5 * 4 + 1 and tape['images'].shape[0] == 7 and tape['eef_pos'].shape == (7, 3)
    d = dict(tape, uid=100000, ic_uid=232, teacher='random', teacher_ckpt='', act_mode='sample', action_repeat=4,
             delta_cap=0.025, delta_leash=0.125, delta_ref='target', pick_z=0.1505, git_sha='x', env_class='FullTaskEnv',
             recorder='record_demos.py v1', contract='v1')
    assert validate_tape(d, require_images=True, require_sim=True) == [], validate_tape(d, True, True)
    assert rec.verify(tape, dict(uid=232)) is True
    # truncation at the cap: 300 decisions, truncated on the last row, label fail
    rec = Recorder(_FakeEnv(), images=False, sim_tape=False)
    tape, r = rec.run(RandomTeacher(0), dict(uid=232))
    assert tape['n'] == 300 and tape['truncated'][-1] and not tape['terminated'][-1] and tape['label'] == 'fail'
    assert r['outcome'] == 'env_truncated'
    # tip mid-window: the window breaks early, sim tape shorter than 4n
    rec = Recorder(_FakeEnv(tip_at=6), images=False, sim_tape=True)
    tape, r = rec.run(RandomTeacher(0), dict(uid=232))
    assert tape['n'] == 2 and tape['tipped'][-1] and tape['terminated'][-1] and tape['sim_states'].shape[0] == 6
    # human follower on a synthetic source tape: waypoints exhausted -> settle -> adapter_exhausted
    import tempfile
    n = 20; S = np.zeros((n, 17), np.float32); A = np.zeros((n, 7), np.float32)
    A[:, 0] = np.linspace(0, 0.2, n); S[1:, 0] = A[:-1, 0]; A[:, 6] = 1.0
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, '232.npz'); np.savez(p, states=S, actions=A, uid=232, label='success', stage='picked')
        hf = HumanFollower(tol=0.05, settle=3); hf.load(p)
        rec = Recorder(_FakeEnv(), images=False, sim_tape=False)
        tape, r = rec.run(hf, dict(uid=232))
        assert r['end_reason'] == 'adapter_exhausted' and tape['truncated'][-1] and r['waypoints_reached'] == n
        assert abs(tape['actions'][-1, 0] - 0.2) < 0.03, tape['actions'][-1, 0]
        assert r['settle_decisions'] == 3 and 'dilation' in r


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn(); print(f'{name}: ok')
    print('ALL OK')
