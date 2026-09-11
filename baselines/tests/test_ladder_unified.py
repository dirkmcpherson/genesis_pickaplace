"""Unit tests for the UNIFIED end-to-end ladder (LADDER_UNIFY_BRIEF_2026-09-10, D1/D2/D6/D7/D8).

What is under test is `FullTaskEnv`'s scope='full' REWARD LOOP and TERMINATION -- not the
predicates. Genesis is never built: the env instance is created with `__new__` and driven
against a fake `genv` (fake solver contacts, fake poses), and the stage tracker is a fake
whose flags each test sets by hand. That separation is deliberate:

  * the LADDER tests must keep passing when Lane 1 replaces `baselines/stage_predicates.py`
    with the calibrated module, so they must not depend on its internals;
  * the PREDICATE tests at the end exercise `stage_predicates.StageTracker` through its
    documented interface only, so they state the contract Lane 1's module must satisfy.

Cases (the four the brief names, plus the provenance stamp):
  1. pressing a HELD can against the goal pays only `picked`   (no release -> no push rung)
  2. release -> push -> nested pays 1 + 1 + 2 + 4 = 8 and TERMINATES on slide_success
  3. `nested_v2` alone never terminates, and the legacy `nested` proxy never terminates
  4. the provenance stamp is present, stable, and moves when the ladder moves
  5. `FULLENV_REWARD_X` is refused; the amendment-(w) episode record is always on
  6. goalward shaping (D7) is OFF by default and is a potential when on

Run (pytest is not installed in the project venv -- both forms work):
  ~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_ladder_unified.py
  ~/workspace/genesis_sim2real/venv/bin/python -m pytest baselines/tests -q
"""
import os
import sys
import types
import pathlib as pl
import numpy as np

REPO = pl.Path(__file__).resolve().parents[2]
os.environ.setdefault('GENESIS_PICKAPLACE_ROOT', str(REPO))
# the ladder must be importable with the legacy gate unset; test 5 sets it deliberately
os.environ.pop('FULLENV_REWARD_X', None)


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

import full_env                      # noqa: E402
import stage_predicates as SP        # noqa: E402
from full_env import FullTaskEnv     # noqa: E402

UPRIGHT = [1.0, 0.0, 0.0, 0.0]
TIPPED = [float(np.cos(np.pi / 4)), float(np.sin(np.pi / 4)), 0.0, 0.0]   # 90 deg about x
SHELF_TOP = 0.17            # a plausible corrected-world shelf top; the fake world uses it
IN_FOOTPRINT = (0.75, -0.1875)      # centre of the shelf rectangle (replay_harness BOX_POS)
GOAL_XY = (0.672, -0.221)


# ------------------------------------------------------------------------------ fake world
class _Ent:
    def __init__(self, pos, quat=UPRIGHT):
        self._p = np.asarray(pos, float); self._q = np.asarray(quat, float)
        self.contacts = {}

    def get_pos(self):
        return self._p

    def get_quat(self):
        return self._q

    def set(self, pos=None, quat=None):
        if pos is not None:
            self._p = np.asarray(pos, float)
        if quat is not None:
            self._q = np.asarray(quat, float)

    def get_contacts(self, other):
        n = int(self.contacts.get(id(other), 0))
        return {'position': np.zeros((n, 3)), 'link_a': np.zeros(n)}


class FakeGenv:
    """Everything FullTaskEnv._step_once touches, and nothing else."""

    def __init__(self):
        self.w = dict(bottle=_Ent((0.6, -0.1, 0.05)), goal=_Ent(GOAL_XY + (0.05,)),
                      kinova=_Ent((0.0, 0.0, 0.0)), eef=_Ent((0.4, -0.1, 0.2)),
                      pick_z=0.09, scene=None)
        self.max_steps = 10 ** 9
        self._picked = False
        self._contact_push = False
        self.tool = (0.4, -0.1)
        self.info = dict(picked=False, placed=False, contact=False, can_goal_touch=False,
                         gripper_goal_touch=False, contact_push=False, slide_success=False)
        self.grip_seen = []

    # -- the two calls full_env makes into the inner env ---------------------------------
    def step(self, a_phys):
        self.grip_seen.append(float(a_phys[6]))
        return ({'state': np.zeros(17, np.float32)}, False, dict(self.info))

    def tool_pos(self):
        return np.array([self.tool[0], self.tool[1], 0.2])

    def _obs(self):
        return {'state': np.zeros(17, np.float32)}


class FakeTracker:
    """Stands in for stage_predicates.StageTracker: flags are whatever the test says.

    The key set is built from the REAL module's FLAG_KEYS + DIAG_KEYS, not hand-written, so a
    flag Lane 1 adds cannot go missing here and make `_full_scope_predicates` KeyError against
    a fake that silently lags the contract."""

    def __init__(self):
        self.flags = {k: False for k in SP.FLAG_KEYS}
        self.flags.update({k: 0.0 for k in SP.DIAG_KEYS})
        self.flags['lever_m'] = 0.5
        self.calls = []

    def reset(self):
        self.calls = []

    def constants(self):
        return dict(fake=True)

    def update(self, **kw):
        self.calls.append(kw)
        return dict(self.flags)


def make_env(goalward=False, ladder='staged', far_release=False):
    """A scope='full' FullTaskEnv with no Genesis: __init__ is bypassed on purpose."""
    e = FullTaskEnv.__new__(FullTaskEnv)
    e.genv = FakeGenv()
    e.scope = 'full'
    e.ladder = ladder
    e.far_release = bool(far_release)
    e.never_terminate = False
    e.stage_reward, e.terminal_stages = full_env.ladder_spec(ladder)
    e.action_mode = 'absolute'
    e.action_repeat = 1
    e.max_steps = 1200
    e.shelf_top_z = SHELF_TOP
    e.pick_z = 0.09
    e.phase_sparse = False
    e.shaping = False
    e.pick_shaping = False
    e.pick_hold_reward = False
    e.pick_hold_k = 25
    e.pick_shaping_terminal_zero = True
    e._pick_gamma = 0.998
    e._pick_phi_prev = 0.0
    e.contact_grant = None
    e.goalward_shaping = bool(goalward)
    e.goalward_scale = FullTaskEnv.GOALWARD_SCALE
    e.goalward_gamma = FullTaskEnv.GOALWARD_GAMMA
    e._goalward_phi_prev = 0.0
    e._t = 0
    e._acct = full_env.LadderAccountant(
        ladder, scope='full',
        pay_stages=full_env.LadderAccountant.pay_stages_for('full', False))
    e._granted = e._acct.reset()
    e._hold_run = 0
    e._pv2_run = 0
    e._phi = 0.0
    e._track = {}
    e.tracker = FakeTracker()
    return e


def denorm_grip(phys):
    """FullTaskEnv in 'absolute' mode denormalizes the action; drive it with the physical
    grip we want by inverting pick_env.denormalize_action's affine grip map."""
    return 2.0 * float(phys) - 1.0


def act(grip_phys):
    a = np.zeros(7, np.float32)
    a[6] = denorm_grip(grip_phys)
    return a


def place_can_on_shelf(env):
    """Put the can where placed_v2's clauses hold (footprint, z-band, upright)."""
    env.genv.w['bottle'].set(pos=(IN_FOOTPRINT[0], IN_FOOTPRINT[1], SHELF_TOP + 0.03), quat=UPRIGHT)


def run_placed_v2(env, n=None):
    """Drive the sustained placed_v2 window with the grip commanded open. Returns the
    summed reward over those decisions."""
    place_can_on_shelf(env)
    env.genv.info['picked'] = True
    r = 0.0
    for _ in range(n or FullTaskEnv.PLACE_SUSTAIN):
        _, rr, term, trunc, _ = env.step(act(0.0))
        r += rr
        assert not term and not trunc, 'placed_v2 must not terminate'
    return r


# ================================================================================== tests
def test_1_pressing_a_held_can_pays_only_picked():
    """Brief §Why item 4: both old ladders paid a contact rung that fires while the robot
    still HOLDS the can, so policies press the held can into the goal and run to the
    horizon. Under the unified ladder the push rung requires a prior release, so this
    episode earns +1 and nothing else."""
    env = make_env()
    env.genv.info.update(picked=True, contact=True, can_goal_touch=True)
    env.genv._contact_push = True                  # the legacy (g) predicate DOES fire
    env.tracker.flags.update(in_hand=True, contact_push=False, slide_success=False)
    total = 0.0
    for _ in range(20):
        _, r, term, trunc, info = env.step(act(1.0))     # grip commanded CLOSED: still held
        total += r
        assert not term, 'pressing a held can must not terminate'
    assert total == 1.0, f'held press paid {total}, expected only the +1 pick'
    assert env._granted == {'picked', 'contact'}, env._granted
    assert info['contact_push'] is False and info['contact_push_legacy'] is True, info
    print('1. held can pressed into the goal: reward 1.0 (picked only), no terminal  OK')


def test_2_release_push_nest_pays_8_and_terminates():
    """The full ladder, in order: picked 1, placed_v2 1, contact_push 2, slide_success 4."""
    env = make_env()
    r = 0.0
    # (a) the pick
    env.genv.info['picked'] = True
    _, rr, term, _, _ = env.step(act(1.0)); r += rr
    assert r == 1.0 and not term
    # (b) release on the shelf: placed_v2 after PLACE_SUSTAIN consecutive frames
    r += run_placed_v2(env)
    assert 'placed_v2' in env._granted and r == 2.0, (env._granted, r)
    # (c) the push: tracker reports contact_push (release already granted)
    env.genv.info.update(contact=True, can_goal_touch=True)
    env.tracker.flags.update(released=True, contact_push=True)
    _, rr, term, _, info = env.step(act(0.0)); r += rr
    assert r == 4.0 and not term, (r, term)
    assert info['contact_push'] is True
    # (d) the slide: pushed + nested_v2 -> slide_success, PAID and TERMINAL
    env.tracker.flags.update(pushed=True, nested_v2=True, slide_success=True)
    _, rr, term, trunc, info = env.step(act(0.0)); r += rr
    assert r == 8.0, f'full ladder paid {r}, expected 8.0'
    assert term and not trunc, 'slide_success must terminate the episode'
    assert info['nested_v2'] is True
    # the episode record (amendment (w), D8) is on the terminal exit, ungated
    assert info['episode_end'] is True
    for k in ('ep_picked', 'ep_placed_v2', 'ep_contact_push', 'ep_slide_success'):
        assert info[k] is True, (k, info[k])
    # paid once: repeating the slide frame pays nothing more (the env is terminal, but the
    # grant set is what guarantees pay-once)
    assert full_env.STAGE_REWARD == dict(picked=1.0, placed_v2=1.0, contact_push=2.0,
                                         slide_success=4.0)
    assert sum(full_env.STAGE_REWARD.values()) == 8.0, 'return_clamp 8 assumes this ceiling'
    print('2. release -> push -> nest: reward 8.0 (1+1+2+4), terminates on slide_success  OK')


def test_3_nested_v2_and_the_legacy_proxy_never_terminate():
    """Brief D2: nested_v2 is LOGGED, never paid, never terminal; the old `nested` proxy no
    longer terminates anything. This is defect 5: the proxy's clauses are a subset of the
    slide's, so terminating on it ended the episode on the first frame of the slide window
    and the +4 rung could never be paid in training."""
    env = make_env()
    env.genv.info['picked'] = True
    env.step(act(1.0))
    run_placed_v2(env)
    # nested_v2 true, slide_success false (e.g. arrived by a DROP, never pushed)
    env.genv.info.update(contact=True, can_goal_touch=True)
    env.tracker.flags.update(released=True, nested_v2=True, pushed=False, slide_success=False)
    first = None
    for _ in range(30):
        _, r, term, trunc, info = env.step(act(0.0))
        first = first or info
        assert not term, 'nested_v2 alone must never terminate'
        assert r == 0.0, f'nested_v2 must pay nothing, paid {r}'
    assert info['nested_v2'] is True and 'nested_v2' in env._granted
    assert 'nested_v2' not in full_env.STAGE_REWARD
    # the LEGACY proxy: contact + grip commanded open + both cans upright. It fires on the
    # FIRST such frame (it is written into `info` only until it is granted, then carried by
    # `_granted`) and must still not terminate on that frame or any later one.
    assert first['nested'] is True, 'the legacy proxy should be computed and logged'
    assert 'nested' in env._granted and 'nested' not in full_env.STAGE_REWARD
    # the tip rule DOES still terminate
    env.genv.w['bottle'].set(quat=TIPPED)
    _, r, term, trunc, info = env.step(act(0.0))
    assert term and info.get('tipped') is True, 'the tip rule must still terminate'
    assert info['episode_end'] is True and info['ep_tipped'] is True
    print('3. nested_v2 / legacy proxy: logged, unpaid, non-terminal; tip still terminates  OK')


def test_4_provenance_stamp():
    """D6: a stamp that names the ladder AND the code that produced it, stable across calls
    and different when the ladder differs."""
    p = full_env.ladder_provenance()
    assert p['stage_reward'] == dict(full_env.STAGE_REWARD)
    assert p['terminal_stages'] == ['slide_success', 'tipped'], p['terminal_stages']
    assert p['shaping'] is None
    for k in ('full_env', 'genesis_can_env', 'stage_predicates'):
        assert len(p['sha256'][k]) == 64 and p['sha256'][k] != 'unreadable', (k, p['sha256'][k])
    assert full_env.ladder_stamp() == full_env.ladder_stamp(), 'stamp must be stable'
    shaped = full_env.ladder_stamp('staged', dict(kind='goalward_potential', scale=2.0, gamma=0.999))
    assert shaped != full_env.ladder_stamp(), 'shaping must change the stamp'
    assert 'shaping=off' in full_env.ladder_stamp()
    env = make_env()
    assert env.shaping_config() is None
    prov = env.provenance()
    assert prov['scope'] == 'full' and prov['stamp'] == full_env.ladder_stamp()
    print('4. provenance stamp present, stable, ladder- and shaping-sensitive  OK')


def test_5_legacy_gate_is_refused():
    """D1: the gate is the defect, not its default -- a tree with no gates must say so."""
    os.environ['FULLENV_REWARD_X'] = '1'
    try:
        try:
            full_env.refuse_legacy_gates()
        except SystemExit as e:
            assert 'legacy gate set' in str(e) and 'no gates' in str(e), str(e)
        else:
            raise AssertionError('refuse_legacy_gates() did not fire with FULLENV_REWARD_X=1')
    finally:
        os.environ.pop('FULLENV_REWARD_X', None)
    full_env.refuse_legacy_gates()                     # unset: no-op
    src = (REPO / 'baselines' / 'rl' / 'full_env.py').read_text()
    assert "environ.get('FULLENV_REWARD_X'" not in src, 'the gate must not be READ anywhere'
    # D8: the episode record is always on. The NAME may appear in the comment that explains
    # its removal; what must not exist is a READ of it.
    for _read in ("environ.get('FULLENV_EPISODE_RECORD'", "environ['FULLENV_EPISODE_RECORD'"):
        assert _read not in src, 'the (w) record must be ungated (D8)'
    print('5. FULLENV_REWARD_X refused; no reward/record gate is read anywhere  OK')


def test_6_goalward_shaping_off_by_default_and_is_a_potential():
    """D7: built now, OFF by default, a constructor argument and never an env var."""
    env = make_env(goalward=False)
    assert env.goalward_shaping is False
    env.genv.info['picked'] = True
    _, r, _, _, _ = env.step(act(1.0))
    assert r == 1.0, 'shaping off must leave the sparse ladder alone'
    src = (REPO / 'baselines' / 'rl' / 'full_env.py').read_text()
    assert 'GOALWARD' in src and "environ.get('GOALWARD" not in src, 'no env var for D7'
    sh = make_env(goalward=True)
    # gate closed (placed_v2 not granted): phi == 0, so no shaping term is paid
    sh.genv.info['picked'] = True
    _, r, _, _, _ = sh.step(act(1.0))
    assert r == 1.0, f'shaping outside its gate must pay 0, paid {r - 1.0}'
    # gate open: placed_v2 granted, not in hand, can touching the goal
    run_placed_v2(sh)
    sh.genv.info.update(contact=True, can_goal_touch=True)
    sh.tracker.flags.update(released=True, in_hand=False)
    sh.genv.w['bottle'].set(pos=(GOAL_XY[0] + 0.20, GOAL_XY[1], SHELF_TOP + 0.03))
    sh.step(act(0.0))                                    # seeds phi_prev at -2*0.20
    d0 = 0.20
    sh.genv.w['bottle'].set(pos=(GOAL_XY[0] + 0.15, GOAL_XY[1], SHELF_TOP + 0.03))
    _, r, _, _, _ = sh.step(act(0.0))                    # moved 5 cm toward the goal
    want = sh.goalward_gamma * (-2.0 * 0.15) - (-2.0 * d0)
    assert abs(r - want) < 1e-6, f'goalward term {r} != gamma*phi(s+) - phi(s) = {want}'
    assert want > 0, 'moving the can toward the goal must be rewarded'
    print('6. goalward shaping: off by default, constructor-only, exact potential form  OK')


def _drive_full_episode(env):
    """picked -> placed_v2 -> contact_push -> pushed+nested_v2 (-> slide_success).

    The SAME fake episode for both ladders: identical actions, identical tracker flags,
    identical logged stages. Only the reward and the terminal may differ."""
    out = []
    env.genv.info['picked'] = True
    out.append(env.step(act(1.0)))                                  # the pick
    for _ in range(FullTaskEnv.PLACE_SUSTAIN):                      # the release, sustained
        place_can_on_shelf(env)
        out.append(env.step(act(0.0)))
    env.genv.info.update(contact=True, can_goal_touch=True)
    env.tracker.flags.update(released=True, contact_push=True)
    out.append(env.step(act(0.0)))                                  # the push
    env.tracker.flags.update(pushed=True, nested_v2=True, slide_success=True)
    out.append(env.step(act(0.0)))                                  # arrival
    return out


def test_9a_sparse_ladder_pays_one_and_terminates_on_nested_v2():
    """User, 2026-09-11: a SPARSE ladder as a first-class option. Same episode, same logged
    stages, same diagnostics -- reward and terminal are the only differences."""
    staged = make_env(ladder='staged')
    sparse = make_env(ladder='sparse')
    r_staged = sum(s[1] for s in _drive_full_episode(staged))
    r_sparse = sum(s[1] for s in _drive_full_episode(sparse))
    assert r_staged == 8.0, r_staged
    assert r_sparse == 1.0, r_sparse
    # terminals: staged ends on slide_success, sparse on nested_v2
    assert full_env.ladder_spec('staged')[1] == ('slide_success',)
    assert full_env.ladder_spec('sparse')[1] == ('nested_v2',)
    assert full_env.max_return('staged') == 8.0 and full_env.max_return('sparse') == 1.0
    # the logged stage set is IDENTICAL -- the arms differ only in what was paid
    assert staged._granted == sparse._granted, (staged._granted, sparse._granted)
    for k in ('picked', 'placed_v2', 'contact_push', 'nested_v2', 'slide_success'):
        assert k in staged._granted, k
    # sparse terminates EARLIER in general: nested_v2 fires no later than slide_success
    # (slide_success requires it). Here they fire on the same decision, so both end there.
    assert _drive_full_episode(make_env(ladder='sparse'))[-1][2] is True, 'sparse must terminate'
    assert _drive_full_episode(make_env(ladder='staged'))[-1][2] is True, 'staged must terminate'
    # an unknown ladder must RAISE, not fall back to the default objective
    try:
        full_env.ladder_spec('stagd')
    except ValueError as e:
        assert 'unknown ladder' in str(e)
    else:
        raise AssertionError('a typo in the ladder name must not silently select a default')
    print('9a. sparse ladder: same episode pays 8 staged / 1 sparse, same logged stages  OK')


def test_9b_nested_v2_pays_nothing_under_staged_and_terminates_only_under_sparse():
    """The discriminating case: arrival WITHOUT a push (a drop at the goal). Under staged it
    is worth 0 and does not end the episode; under sparse it is the whole reward."""
    for ladder, want_r, want_term in (('staged', 0.0, False), ('sparse', 1.0, True)):
        env = make_env(ladder=ladder)
        env.genv.info['picked'] = True
        env.step(act(1.0))
        run_placed_v2(env)
        env.tracker.flags.update(released=True, nested_v2=True, pushed=False, slide_success=False)
        _, r, term, _, info = env.step(act(0.0))
        assert info['nested_v2'] is True
        assert r == want_r, (ladder, r, want_r)
        assert term is want_term, (ladder, term, want_term)
    # and the provenance distinguishes them
    assert full_env.ladder_stamp('staged') != full_env.ladder_stamp('sparse')
    for L in ('staged', 'sparse'):
        p = full_env.ladder_provenance(L)
        assert p['ladder'] == L and p['stage_reward'] == full_env.ladder_spec(L)[0]
        assert p['terminal_stages'] == list(full_env.ladder_spec(L)[1]) + ['tipped']
        assert p['return_clamp_required'] == full_env.max_return(L)
    print('9b. nested_v2 alone: 0 and non-terminal under staged, 1 and terminal under sparse  OK')


def test_9_constructor_signature_matches_its_callers():
    """Lane 3's BLOCKER: 809601d copied full_env.py in from another tree and silently reverted
    the PHASE_PLAN (p) `contact_grant` parameter, while train_rlpd.py:298 and eval_place.py:162
    still pass `contact_grant=` -- on EVERY scope, not only contact. FullTaskEnv construction
    therefore raised TypeError for every train_rlpd run on this branch. This test asserts the
    signature against its real callers so the revert cannot happen silently again.

    Signature only: building the env needs Genesis, so the call is checked with
    inspect.signature().bind() rather than executed."""
    import inspect
    sig = inspect.signature(FullTaskEnv.__init__)
    # exactly the call train_rlpd.py makes
    sig.bind(None, backend='cpu', max_steps=1200, scope='full', action_mode='delta_joint',
             action_repeat=4, delta_ref='target', entry_bank=None, phase_sparse=False,
             contact_grant=None, pick_hold_reward=False, pick_hold_k=25, pick_shaping=False,
             pick_shaping_gamma=0.99, pick_shaping_terminal_zero=True, ladder='staged',
             goalward_shaping=False, goalward_gamma=0.99)
    # the Ladder-N call the nested-ladder launchers and the relabel make
    sig.bind(None, backend='cpu', max_steps=2400, scope='full', ladder='nested_ramp',
             far_release=True, action_mode='delta_joint', delta_cap=0.025,
             delta_leash_mult=5.0, action_repeat=4, delta_ref='target', camera_rig=False)
    # exactly the call eval_place.py makes
    sig.bind(None, backend='cpu', max_steps=600, scope='place', entry_bank='bank.json',
             phase_sparse=True, contact_grant=None, action_mode='delta_joint', delta_cap=0.025,
             delta_leash_mult=5.0, action_repeat=4, delta_ref='target', render_size=None)
    # and the calls the recorder / the two evaluators / the r2dreamer adapter make
    sig.bind(None, backend='cpu', max_steps=2400, scope='full', action_mode='delta_joint',
             delta_cap=0.025, delta_leash_mult=5.0, action_repeat=4, delta_ref='target',
             camera_rig=True, pick_shaping=False)
    sig.bind(None, backend='cpu', max_steps=10 ** 9, scope='full', render_size=(64, 64),
             camera_rig=True, shaping=False, pick_shaping=False, entry_bank=None, ladder='sparse')
    # the (p) guards are reachable: a contact-scope env still refuses an unnamed grant
    src = (REPO / 'baselines' / 'rl' / 'full_env.py').read_text()
    assert 'scope=contact needs an explicit contact_grant' in src
    assert "contact_grant in ('bare_contact', 'slide_success', 'prior_release')" in src
    print('9. constructor signature binds every real call site (contact_grant restored)  OK')


# ======================================================= LADDER N (user, 2026-09-11) =====
# These run the REAL StageTracker against the REAL LadderAccountant -- no fakes -- because the
# ramp's value is a function of a tracker quantity, so a fake tracker would be testing the
# test. `_ladder_run` reproduces exactly what FullTaskEnv does per env frame: feed the tracker,
# copy its flags into `info` beside the env-owned `picked`/`placed_v2`, hand `info` to the
# accountant, stop at the ladder's terminal.
N_GOAL = np.array([0.700, -0.200, 0.147])
N_SHELF_TOP = 0.110
N_REST_Z = 0.148


def _n_frame(can_d, tool_gap, *, picked=True, placed_v2=True, z=N_REST_Z,
             can_quat=UPRIGHT, tool_dir='far', can_goal_contact=False):
    """One env frame `can_d` metres from the goal (approached from +x) with the tool `tool_gap`
    from the can centre. `tool_dir`: 'far' = behind the can (dot < 0, the pushing pose),
    'near' = between can and goal, 'perp' = to the side (dot == 0)."""
    can = np.array([N_GOAL[0] + can_d, N_GOAL[1]])
    off = {'far': (tool_gap, 0.0), 'near': (-tool_gap, 0.0), 'perp': (0.0, tool_gap)}[tool_dir]
    tool = can + np.asarray(off, float)
    return dict(can_pos=np.array([can[0], can[1], z]), can_quat=can_quat, goal_pos=N_GOAL,
                goal_quat=UPRIGHT, tool_xy=tool, grip_cmd=0.4, picked=picked,
                placed_v2=placed_v2, can_goal_contact=can_goal_contact,
                gripper_goal_contact=False)


def _ladder_run(frames, ladder, far_release=False):
    """-> (total reward, terminal frame index or None, accountant, tracker episode)."""
    tr = SP.StageTracker(N_GOAL[:2], N_SHELF_TOP, far_release=far_release)
    acct = full_env.LadderAccountant(
        ladder, scope='full', pay_stages=full_env.LadderAccountant.pay_stages_for('full', False))
    acct.reset()
    total, term_at = 0.0, None
    for i, f in enumerate(frames):
        info = dict(tr.update(**f))
        info['picked'] = bool(f['picked'])
        info['placed_v2'] = bool(f['placed_v2'])
        r, term = acct.frame(info)
        total += r
        if term:
            term_at = i
            break
    return total, term_at, acct, tr.episode()


def _release_farside_push_home(start_d=0.105, end_d=0.005, n_push=40):
    """Release at `start_d`, tool moves to the far side, pushes the can `start_d - end_d`
    metres home, then the can settles inside the touch distance."""
    frames = [_n_frame(start_d, 0.15) for _ in range(SP.AT_REST_FRAMES + 4)]
    for i in range(n_push):
        d = start_d + (end_d - start_d) * (i + 1) / n_push
        frames.append(_n_frame(d, 0.035, can_goal_contact=(d <= 0.070)))
    frames += [_n_frame(end_d, 0.035 + 0.006 * (i + 1), can_goal_contact=True)
               for i in range(SP.AT_REST_FRAMES + 8)]
    return frames


def test_10a_nested_ramp_pays_9_for_a_full_slide_and_nested_sparse_pays_1():
    """The user's ladder, end to end: pick 1, release 1, far side 1, a 10 cm slide 2, home 4."""
    frames = _release_farside_push_home()          # 10.0 cm of push -> the ramp saturates
    r, term, acct, ep = _ladder_run(frames, 'nested_ramp')
    assert abs(ep['slide_gain_m'] - 0.100) < 1e-9, ep['slide_gain_m']
    assert abs(r - 9.0) < 1e-9, f'nested_ramp paid {r}, expected 1+1+1+2+4 = 9'
    assert term is not None and ep['home'], 'home must terminate'
    assert acct.paid == {'picked', 'placed_v2', 'farside', 'home'}
    assert 'slide_event' in acct.granted, 'the named three-clause flag is logged, not paid'
    assert 'slide_event' not in acct.paid
    assert abs(acct.ramp_paid - 2.0) < 1e-9, 'the ramp saturates at its scale'
    assert full_env.max_return('nested_ramp') == 9.0
    r, term, acct, ep = _ladder_run(frames, 'nested_sparse')
    assert abs(r - 1.0) < 1e-9, f'nested_sparse paid {r}, expected 1'
    assert term is not None and acct.paid == {'home'}
    assert full_env.max_return('nested_sparse') == 1.0
    print('10a. full slide: nested_ramp 9 (1+1+1+2+4), nested_sparse 1, both terminal  OK')


def _drop_at_the_goal(retreat='far'):
    """Carry the can to the goal, open the hand, walk away. `retreat` is the DIRECTION the
    tool withdraws in -- 'far' straight back along the can->goal line, 'perp' sideways."""
    frames = [_n_frame(0.060, 0.015, placed_v2=False) for _ in range(6)]        # carried in
    frames += [_n_frame(0.060, 0.015 + 0.01 * i, can_goal_contact=True, tool_dir=retreat)
               for i in range(4)]                                              # released
    frames += [_n_frame(0.060, 0.30, can_goal_contact=True, tool_dir=retreat)
               for _ in range(SP.AT_REST_FRAMES + 8)]                          # tool retreats
    return frames


def test_10b_a_drop_at_the_goal_pays_no_ramp_and_no_home():
    """The behaviour the redesign exists to stop paying for: carry the can to the goal, open
    the hand, walk away.

    MEASURED SPEC DEVIATION, disclosed rather than patched away. The brief predicted 2
    (picked + placed_v2). It is 3, because a gripper that withdraws STRAIGHT BACK from a
    set-down passes through the far-side band on its way out -- 2.5 to 8 cm behind the can, on
    the opposite side from the goal -- which is exactly what `farside` says. No clause in the
    brief's definition excludes a withdrawal, and adding one ("the tool must approach, not
    leave") would be a new predicate invented in a test. So `farside` is cheap on the drop
    route, and what carries the contrast is the pair above it: the dense ramp needs goalward
    motion made from that pose, and `home` needs both. A drop earns neither, under either
    variant -- and for reference the pilot's `staged` ladder pays this same drop 4.

    A withdrawal to the SIDE does not grant it, which is the control that the clause is the
    geometry and not a rubber stamp."""
    frames = _drop_at_the_goal('far')
    r, term, acct, ep = _ladder_run(frames, 'nested_ramp')
    assert ep['nested_v2'] and not ep['home'], 'a drop IS a nest and is NOT home'
    assert ep['farside'] and ep['slide_gain_m'] == 0.0, 'in the pose, no progress from it'
    assert abs(r - 3.0) < 1e-9, f'nested_ramp paid {r} for a drop, expected 1+1+1'
    assert acct.paid == {'picked', 'placed_v2', 'farside'} and term is None
    assert acct.ramp_paid == 0.0, 'the DENSE rung is what a drop cannot earn'
    r, term, acct, _ = _ladder_run(frames, 'nested_sparse')
    assert r == 0.0 and term is None, (r, term)
    # the same drop under the pilot's ladders, for contrast
    assert _ladder_run(frames, 'staged')[0] == 4.0, 'staged pays a drop 4 of its 8'
    assert _ladder_run(frames, 'sparse')[0] == 1.0, 'sparse pays a drop-in nest in full'
    # sideways withdrawal: never behind the can, so not even the cheap rung
    side = _drop_at_the_goal('perp')
    r, _, acct, ep = _ladder_run(side, 'nested_ramp')
    assert not ep['farside'] and abs(r - 2.0) < 1e-9, (r, ep['farside'])
    print('10b. drop at the goal: ramp 0 and home 0 under both variants (3 / 0 total)  OK')


def test_10c_the_ramp_pays_once_for_net_progress():
    """Oscillating the can cannot farm the dense rung: the ramp is a function of a MONOTONE
    tracker quantity, so pushing 3 cm, letting it come back, and pushing again pays 3 cm."""
    once = [_n_frame(0.120, 0.15) for _ in range(SP.AT_REST_FRAMES + 4)]
    for d in np.linspace(0.120, 0.090, 31):
        once.append(_n_frame(float(d), 0.035))
    twice = list(once)
    for d in np.linspace(0.090, 0.120, 31):
        twice.append(_n_frame(float(d), 0.035))
    for d in np.linspace(0.120, 0.090, 31):
        twice.append(_n_frame(float(d), 0.035))
    r1, _, a1, e1 = _ladder_run(once, 'nested_ramp')
    r2, _, a2, e2 = _ladder_run(twice, 'nested_ramp')
    assert abs(e1['slide_gain_m'] - 0.030) < 1e-9 and abs(e2['slide_gain_m'] - 0.030) < 1e-9
    # span 0.05 m per amendment (aa): 30 mm of net progress pays 2 * 30/50 = 1.2
    assert abs(a1.ramp_paid - 1.2) < 1e-9, '2 * 30mm/50mm'
    assert abs(r1 - r2) < 1e-9, f'the second lap paid {r2 - r1} extra'
    assert abs(r1 - (1.0 + 1.0 + 1.0 + 1.2)) < 1e-9, r1
    print('10c. ramp: paid once for net progress, oscillation adds nothing  OK')


def test_10d_far_release_blocks_a_drop_and_nudge():
    """With the switch on, a release 6 cm from the goal earns no farside, no ramp and no home
    however far the can is then nudged; the same episode released at 10.5 cm pays in full."""
    close = _release_farside_push_home(start_d=0.060, end_d=0.005, n_push=40)
    far = _release_farside_push_home(start_d=0.105, end_d=0.005, n_push=40)
    r_close_off = _ladder_run(close, 'nested_ramp', far_release=False)[0]
    r_close_on, term, acct, ep = _ladder_run(close, 'nested_ramp', far_release=True)
    assert r_close_off > 2.0, 'with the switch OFF the nudge does pay'
    assert abs(r_close_on - 2.0) < 1e-9, f'gated release paid {r_close_on}, expected 2'
    assert not ep['farside'] and not ep['home'] and ep['slide_gain_m'] == 0.0
    assert term is None and acct.ramp_paid == 0.0
    assert ep['release_far'] is False, 'and the measurement says why'
    r_far_on, term, _, ep = _ladder_run(far, 'nested_ramp', far_release=True)
    assert abs(r_far_on - 9.0) < 1e-9 and term is not None and ep['release_far'] is True
    assert _ladder_run(close, 'nested_sparse', far_release=True)[0] == 0.0
    print('10d. far_release: a release inside FAR_RELEASE_DIST_M earns no farside/ramp/home  OK')


def test_10e_requires_holds_the_rungs_in_order():
    """`placed_v2` at reset with the arm at home (4 of the 30 rnd30 starts, CONFOUNDS row 82)
    pays nothing until the pick, and then pays in full -- an out-of-order rung is deferred,
    never forfeited."""
    frames = [_n_frame(0.150, 0.30, picked=False, placed_v2=True) for _ in range(8)]
    r, term, acct, ep = _ladder_run(frames, 'nested_ramp')
    assert r == 0.0 and acct.paid == set() and not ep['released']
    assert 'placed_v2' in acct.granted, 'reached, and recorded as reached'
    frames += [_n_frame(0.150, 0.30, picked=True, placed_v2=True) for _ in range(4)]
    r, term, acct, _ = _ladder_run(frames, 'nested_ramp')
    assert abs(r - 2.0) < 1e-9, f'deferred rungs must pay once the pick lands, got {r}'
    assert acct.paid == {'picked', 'placed_v2'}
    # ...and a rung whose requirement first fires on the SAME frame pays on that frame, not
    # the next: `home` is the terminal, so deferring it by one frame would end the episode
    # with the rung unpaid -- defect 5, re-created one level down.
    acct = full_env.LadderAccountant('nested_ramp', scope='full', pay_stages=None)
    acct.reset()
    r, term = acct.frame(dict(picked=True, placed_v2=True, farside=True, slide_event=True,
                              home=True, nested_v2=True, slide_gain_m=0.10))
    assert term and abs(r - 9.0) < 1e-9, f'same-frame chain paid {r}, expected the full 9'
    print('10e. requires: deferred when out of order, paid when the chain lands together  OK')


def test_10f_staged_and_sparse_are_unchanged_by_ladder_n():
    """The compatibility claim, on histories rather than on inspection: the same frames scored
    under 'staged' and 'sparse' pay exactly what they paid before Ladder N existed -- the
    accountant/`_paid` split and the new tracker flags move neither."""
    cases = [(_release_farside_push_home(), 8.0, 1.0),        # pick+release+push+nest
             (_drop_at_the_goal('far'), 4.0, 1.0),            # pick+release+contact_push
             (_drop_at_the_goal('perp'), 2.0, 1.0)]           # pick+release only
    for frames, want_staged, want_sparse in cases:
        assert abs(_ladder_run(frames, 'staged')[0] - want_staged) < 1e-9
        assert abs(_ladder_run(frames, 'sparse')[0] - want_sparse) < 1e-9
    assert full_env.ladder_spec('staged')[0] == dict(picked=1.0, placed_v2=1.0,
                                                     contact_push=2.0, slide_success=4.0)
    assert full_env.ladder_extras('staged') == ({}, None)
    assert full_env.ladder_extras('sparse') == ({}, None)
    assert full_env.max_return('staged') == 8.0 and full_env.max_return('sparse') == 1.0
    # every ladder is distinguishable in the stamp, and far_release is part of it
    stamps = {L: full_env.ladder_stamp(L) for L in full_env.LADDERS}
    assert len(set(stamps.values())) == len(stamps), stamps
    assert full_env.ladder_stamp('nested_ramp', None, True) != full_env.ladder_stamp('nested_ramp')
    assert 'far_release=on' in full_env.ladder_stamp('nested_ramp', None, True)
    p = full_env.ladder_provenance('nested_ramp', None, True)
    assert p['far_release'] is True and p['terminal_stages'] == ['home', 'tipped']
    assert p['requires'] == dict(placed_v2='picked', farside='placed_v2',
                                 slide_event='farside', home='slide_event')
    assert p['ramp']['scale'] == 2.0 and p['ramp']['span'] == 0.05   # amendment (aa)
    assert p['return_clamp_required'] == 9.0
    assert full_env.ladder_provenance('nested_sparse')['return_clamp_required'] == 1.0
    print('10f. staged/sparse unchanged; every ladder and far_release distinct in the stamp  OK')


# ---------------------------------------------------------- the Lane-1 interface contract
def test_7_stage_tracker_interface():
    """The contract `full_env` codes against (brief "Lane-1 interface"). These assertions
    must hold for Lane 1's module too -- they are about the INTERFACE, not the constants.
    Anything below that depends on a threshold is exercised against the STUB and is
    labelled as such; the calibrated numbers live in NESTED_V2_PREDICATE_2026-09-10.md."""
    for k in ('NESTED_TOUCH_DIST', 'AT_REST_MM', 'AT_REST_FRAMES', 'PUSH_GAIN_MM', 'HELD_LEVER_M'):
        assert hasattr(SP, k), f'stage_predicates must export {k}'
    tr = SP.StageTracker(goal_xy=GOAL_XY, shelf_top_z=SHELF_TOP)
    out = tr.update(can_pos=(0.6, -0.1, 0.05), can_quat=UPRIGHT, goal_pos=GOAL_XY + (0.05,),
                    goal_quat=UPRIGHT, tool_xy=(0.4, -0.1), grip_cmd=1.0, picked=False,
                    can_goal_contact=False, gripper_goal_contact=False, placed_v2=False)
    for k in SP.FLAG_KEYS + SP.DIAG_KEYS:
        assert k in out, f'update() must return {k}'
    for k in SP.FLAG_KEYS:
        assert isinstance(out[k], bool), (k, type(out[k]))
    # tilt agrees with the env's own definition (the reason for the local copy is only that
    # this module must not import Genesis)
    from replay_harness import tilt_deg as env_tilt
    for q in (UPRIGHT, TIPPED, [0.9, 0.1, 0.2, 0.05]):
        assert abs(SP.tilt_from_quat(q) - env_tilt(q)) < 1e-9, q
    print('7. StageTracker interface + tilt parity with the env  OK')


def test_8_stub_predicates_behave_as_the_brief_says():
    """AGAINST THE STUB (uncalibrated). Two behaviours the brief calls out explicitly:
    a carry into the goal is not a push, and a push after release is."""
    if not getattr(SP, 'IS_STUB', False):
        # Lane 1's calibrated module replaced the stub at merge (2026-09-11); its own suite
        # (test_stage_predicates.py, 26 cases) covers these behaviours against real thresholds.
        print('8. stub-only test skipped: stage_predicates is the calibrated Lane-1 module')
        return
    goal = (0.672, -0.221, 0.05)
    # (a) carried in: tool lever < HELD_LEVER_M every frame -> never pushed, never nested
    tr = SP.StageTracker(goal_xy=goal[:2], shelf_top_z=SHELF_TOP)
    out = None
    for i in range(40):
        x = 0.80 - 0.004 * i                       # can walks toward the goal, in hand
        out = tr.update(can_pos=(x, goal[1], SHELF_TOP + 0.03), can_quat=UPRIGHT,
                        goal_pos=goal, goal_quat=UPRIGHT, tool_xy=(x, goal[1]),
                        grip_cmd=1.0, picked=True, can_goal_contact=False,
                        gripper_goal_contact=False, placed_v2=True)
    assert out['in_hand'] and not out['pushed'] and not out['nested_v2'], out
    # (b) released, then pushed home by a tool that stays a can-radius away
    tr = SP.StageTracker(goal_xy=goal[:2], shelf_top_z=SHELF_TOP)
    for i in range(40):
        x = 0.80 - 0.004 * i
        out = tr.update(can_pos=(x, goal[1], SHELF_TOP + 0.03), can_quat=UPRIGHT,
                        goal_pos=goal, goal_quat=UPRIGHT, tool_xy=(x + 0.05, goal[1]),
                        grip_cmd=0.4, picked=True, can_goal_contact=(x - goal[0]) < 0.08,
                        gripper_goal_contact=False, placed_v2=True)
    assert out['released'] and out['pushed'] and out['contact_push'], out
    for _ in range(SP.AT_REST_FRAMES):             # let it come to rest at the goal
        out = tr.update(can_pos=(goal[0] + 0.05, goal[1], SHELF_TOP + 0.03), can_quat=UPRIGHT,
                        goal_pos=goal, goal_quat=UPRIGHT, tool_xy=(goal[0] + 0.10, goal[1]),
                        grip_cmd=0.4, picked=True, can_goal_contact=True,
                        gripper_goal_contact=False, placed_v2=True)
    assert out['at_rest'] and out['nested_v2'] and out['slide_success'], out
    print('8. stub predicates: carry != push; release+push+rest -> nested_v2 + slide  OK')


def main():
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
    print('ALL OK')


if __name__ == '__main__':
    main()
