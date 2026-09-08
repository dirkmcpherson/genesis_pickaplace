"""Standalone policy eval for genesis_pick r2dreamer runs (dv3 genesis_eval.py equivalent).

Why a separate process: genesis allows ONE gs.init/world per process, so the
training process (eval_episode_num=0) can never evaluate in-process. This loads
a checkpoint (train.py's latest.pt: {"agent_state_dict", "optims_state_dict"})
into a FRESH process, builds a single in-process Genesis world (envs/genesis.py
adapter, scope from the run config -- 'pick'), rolls the policy out for N
episodes from DEMO ICs (FullTaskEnv.reset draws a success-labeled demo uid ->
that demo's recorded can placement; same reset distribution TRAINING uses), and
reports picked/tipped/timeout rates plus a top|wrist mp4 per episode.

Action selection mirrors Dreamer.act's own eval convention:
  --mode sample  -> action_dist.rsample()  (trainer's eval=False path; matches
                    the dv3 protocol headline numbers: eval SAMPLES)
  --mode mode    -> action_dist.mode       (trainer.eval's eval=True path)

Checkpoint caveat: train.py writes latest.pt ONLY at the end of the run
(trainer has no periodic save), so a still-running run dir has no latest.pt yet.

Usage:
  MUJOCO_GL=egl GENESIS_PICKAPLACE_ROOT=~/workspace/genesis_pickaplace \
    ./.venv/bin/python eval_genesis.py --checkpoint runs/<run>/latest.pt \
    --episodes 8 --max-steps 1200 [--mode sample] [--device cpu]
"""
import os

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("GENESIS_PICKAPLACE_ROOT", "/home/j/workspace/genesis_pickaplace")

import argparse
import json
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

ap = argparse.ArgumentParser()
ap.add_argument("--checkpoint", default=str(HERE / "runs/genesis_pick_demo_s0/latest.pt"),
                help="latest.pt path (or a run dir containing one)")
ap.add_argument("--config", default=None,
                help="hydra-resolved config.yaml; default <run_dir>/.hydra/config.yaml "
                     "(MUST match the checkpoint's model dims)")
ap.add_argument("--episodes", type=int, default=8)
ap.add_argument("--max-steps", type=int, default=1200,
                help="env-step horizon per episode (training time_limit is 1200)")
ap.add_argument("--mode", choices=["sample", "mode"], default="sample",
                help="sample = actor rsample (dv3 protocol default); mode = deterministic")
ap.add_argument("--device", default="cpu",
                help="net device; default cpu because training owns the GPU")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--out", default=None, help="output dir; default <run_dir>/policy_eval")
ap.add_argument("--scale", type=int, default=4, help="mp4 upscale factor (64px cams are tiny)")
ap.add_argument("--fps", type=float, default=0,
                help="video playback fps; 0 = real-time (30Hz sim / action_repeat, "
                     "i.e. 7.5 for repeat-4 -- frames are one per agent step)")
ap.add_argument("--ic-file", default=None,
                help="genesis_pickaplace baselines/eval_ics.json: use its IC sets instead "
                     "of random uid draws (final-RR one-harness protocol)")
ap.add_argument("--ic-skip", type=int, nargs="*", default=[], help="episode indices to SKIP and record as outcome 'hang' (counted as failure): deterministic simulator stalls on a specific IC (2026-09-04, rnd300 ep 269)")
ap.add_argument("--dump-entries", default=None, help="PHASE PLAN: write an entry-bank JSON (list) with the state BEFORE the decision on which the scope's success fired, per successful episode (pol-E bank)")
ap.add_argument("--entry-bank", default=None, help="PHASE PLAN: entry bank JSON for scope place/contact (REQUIRED there; the env's legacy default bank is the OLD world)")
ap.add_argument("--ic-set", choices=["sel", "hold", "rnd", "all"], default="sel",
                help="which IC set from --ic-file")
ap.add_argument("--ic-index", type=int, default=None,
                help="single IC index from the set (one episode); default = the whole set "
                     "(overrides --episodes)")
ap.add_argument("--torch-threads", type=int, default=4,
                help="cap torch CPU threads (training's 8 genesis workers share this box)")
ap.add_argument("--append-metrics", metavar="RUN_DIR", default=None,
                help="append the eval summary as one JSON line to "
                     "RUN_DIR/metrics.jsonl (keys eval/*, step = the checkpoint's "
                     "step). The run's wandb live-syncer then uploads it INTO the "
                     "TRAINING wandb run as a first-class curve (eval/picked vs "
                     "env_step) -- single wandb writer, no concurrent-resume "
                     "conflicts. Composes with --wandb (separate run keeps the "
                     "videos).")
ap.add_argument("--wandb", action="store_true",
                help="log summary metrics + up to 6 episode mp4s to wandb "
                     "(entity jambotime, project r2dreamer_genesis); never fatal")
args = ap.parse_args()

import torch  # noqa: E402

torch.set_num_threads(args.torch_threads)

from omegaconf import OmegaConf  # noqa: E402
from tensordict import TensorDict  # noqa: E402

import tools  # noqa: E402
from dreamer import Dreamer  # noqa: E402
from envs.genesis import STAGE_KEYS, GenesisPick  # noqa: E402

CKPT = pathlib.Path(args.checkpoint).expanduser().resolve()
if CKPT.is_dir():
    CKPT = CKPT / "latest.pt"
RUN_DIR = CKPT.parent
if not CKPT.exists():
    sys.exit(f"checkpoint not found: {CKPT}\n(train.py writes latest.pt only at run END; "
             f"a still-running run has no checkpoint yet)")
CFG_PATH = pathlib.Path(args.config) if args.config else RUN_DIR / ".hydra" / "config.yaml"
if not CFG_PATH.exists():
    sys.exit(f"config not found: {CFG_PATH} (pass --config)")
OUT = pathlib.Path(args.out) if args.out else RUN_DIR / "policy_eval"
OUT.mkdir(parents=True, exist_ok=True)

tools.set_seed_everywhere(args.seed)

cfg = OmegaConf.load(CFG_PATH)
cfg.device = args.device          # ${device} interpolations (model/rssm/heads) follow
cfg.model.compile = False         # update-path compile only; never used here
print(f"[eval] ckpt={CKPT}  config={CFG_PATH}")
print(f"[eval] device={args.device}  mode={args.mode}  episodes={args.episodes} "
      f"max_steps={args.max_steps}  scope={cfg.env.scope}  seed={args.seed}")

# ---- env: ONE in-process world (fresh process; never alongside a train run) ----
task = str(cfg.env.task).split("_", 1)[1]
# action_repeat from the RUN config (v2 runs train at 4): eval must hold each
# action the same number of sim steps or it evaluates a different MDP.
# --max-steps stays in SIM steps; the loop counts agent steps.
REPEAT = int(cfg.env.get("action_repeat", 1) or 1)
if not args.fps:
    args.fps = 30.0 / REPEAT   # real-time playback: one frame per agent step
# EVERY action-semantics param comes from the RUN's own config -- silent-default
# bug #7 (2026-08-11): this call passed only action_repeat, so delta_joint
# policies were evaluated in ABSOLUTE mode (a different MDP: delta outputs read
# as absolute joint targets -> lunge-tips + frozen-posture timeouts). All
# eval_genesis results for delta runs before this line are VOID.
if str(cfg.env.scope) in ("place", "contact", "carrycontact") and not args.entry_bank:
    sys.exit("FATAL: scope place/contact needs --entry-bank (no silent default bank)")
env = GenesisPick(task, size=tuple(cfg.env.size), seed=args.seed, scope=str(cfg.env.scope),
                  place_entry_bank=args.entry_bank,
                  action_repeat=REPEAT,
                  action_mode=str(cfg.env.get("action_mode", "absolute")),
                  delta_cap=cfg.env.get("delta_cap", None),
                  delta_leash_mult=cfg.env.get("delta_leash_mult", None),
                  reward_scale=float(cfg.env.get("reward_scale", 1.0)),
                  # WM fix stage 1 (2026-09-03): state-input runs need the same obs keys as training
                  state_obs=bool(cfg.env.get("state_obs", False)),
                  state_extra=cfg.env.get("state_extra", None))
print(f"[eval] action_mode={env._action_mode} delta_cap={getattr(env, '_delta_cap', None)} "
      f"state_obs={getattr(env, '_state_obs', False)} obs_keys={list(env.observation_space.spaces.keys())}")

# ---- agent: rebuild exactly as train.py does, then load the checkpoint ----
agent = Dreamer(cfg.model, env.observation_space, env.action_space).to(args.device)
ck = torch.load(CKPT, map_location=args.device, weights_only=False)
CKPT_STEP = ck.get("step")  # present in checkpoints saved after 2026-08-10; None before
missing, unexpected = agent.load_state_dict(ck["agent_state_dict"], strict=False)
print(f"[eval] loaded checkpoint (missing {len(missing)}, unexpected {len(unexpected)})")
assert not any(k.startswith(("actor.", "rssm.", "encoder.")) for k in missing), missing
agent.clone_and_freeze()          # act() runs on the _frozen_* clones -- resync post-load
agent.requires_grad_(False)
agent.eval()


def pack(obs, reward):
    """obs dict -> (B=1,) TensorDict, mirroring ParallelEnv.step + lift_dim.

    NO "action" key -- deliberately, and verified 2026-08-08 to match training
    exactly: trainer.begin also calls agent.act on a trans WITHOUT an action key
    (trans["action"] is assigned AFTER act, for replay storage only), the
    encoder consumes only cnn_keys='image' (obs_space has no action entry), and
    the RSSM's previous action is threaded through state["prev_action"], which
    act() returns as the action it just emitted (zeros only at t=0 /
    get_initial_state, and obs_step re-zeroes it wherever is_first is set).
    Runtime proof: test_action_conditioning.py (P1-P4 vs the real
    pick checkpoint: garbage action key in trans -> bit-identical act output).
    """
    d = {k: torch.as_tensor(np.asarray(v)[None]) for k, v in obs.items()}
    d["reward"] = torch.tensor([reward], dtype=torch.float32)
    td = TensorDict(d, batch_size=(1,), device="cpu")
    for k in td.keys():
        if td[k].ndim == 1:
            td[k] = td[k].unsqueeze(-1)
    return td.to(args.device)


def reset_to_uid(uid):
    """Adapter reset, pinned to an explicit demo uid / bank entry (reportable ICs). Returns (obs, (restored_uid, entry_frame)).

    EVERY scope pins (amendment (j), 2026-09-07; ADVERSARIAL_REVIEW_eval_env S1-3): the place scope used to call
    env._env.reset() unpinned, i.e. it drew from the WHOLE bank with replacement (~94 of 148 distinct entries per
    cell) and silently substituted any entry that failed to restore, while contact/carrycontact pinned -- the only
    difference behind the "restores in one scope but not the other" chase. Now a pinned entry that does not survive
    the restore raises (FullTaskEnv retries the same entry PLACE_MAX_TRIES times) -> recorded as restore_failed by
    the caller, never substituted; the restored uid is returned and asserted equal to the enumerated one."""
    if env._env is None:
        env._build()                              # gs.init + world build (once)
    _ret = env._env.reset(options={"uid": int(uid)})  # demo IC / bank entry of THAT uid only
    _rinfo = _ret[1] if (isinstance(_ret, tuple) and len(_ret) == 2 and isinstance(_ret[1], dict)) else {}
    _restored = (int(_rinfo["uid"]) if _rinfo.get("uid") is not None else None,
                 (int(_rinfo["entry_frame"]) if _rinfo.get("entry_frame") is not None else None))
    # delta_joint mode: the persistent joint target lives in the ADAPTER, and
    # this function bypasses adapter.reset -- re-seed it from measured qpos or
    # it carries over from the previous episode. No-op in absolute mode.
    getattr(env, "sync_delta_target", lambda: None)()
    obs = {"is_first": True, "is_last": False, "is_terminal": False, "image": env._image()}
    if getattr(env, "_state_obs", False):   # WM fix 2026-09-03: state-input runs (bypasses adapter.reset)
        _st = env._state_vec(env._env.genv._obs()["state"])
        env._last_state = _st
        obs["state"] = _st
    for k in STAGE_KEYS + ("task_success",):
        obs[f"log_{k}"] = np.float32(0.0)
    return obs, _restored


import cv2  # noqa: E402
import hashlib  # noqa: E402

BANK_VERSION = "physgrip_2026-09-07"   # stamped into every --dump-entries entry (physical grip units)
BANK_SHA256 = hashlib.sha256(open(args.entry_bank, "rb").read()).hexdigest() if args.entry_bank else None
BANK_VERSION_IN = None
if args.entry_bank:
    _bj = json.load(open(args.entry_bank)); _b0 = (next(iter(_bj.values())) if isinstance(_bj, dict) else _bj[0]) if _bj else {}
    BANK_VERSION_IN = _b0.get("bank_version") if isinstance(_b0, dict) else None
    _bg = [float(e["grip_cmd"]) for e in (_bj.values() if isinstance(_bj, dict) else _bj)]
    print(f"[eval] entry bank {args.entry_bank}: {len(_bg)} entries, bank_version={BANK_VERSION_IN!r}, grip_cmd range "
          f"{min(_bg):.3f}..{max(_bg):.3f}, sha256 {BANK_SHA256[:12]}", flush=True)
    if min(_bg) < 0.0:
        print("[eval] WARNING: bank carries grip_cmd < 0 (raw [-1,1] units; review S2-4) -- the restore clips it to 0 (fingers open)", flush=True)

print("[eval] building the Genesis world (~20-35 s)...", flush=True)
t0 = time.time()
if env._env is None:
    env._build()
print(f"[eval] world built in {time.time() - t0:.1f} s")
rng = np.random.RandomState(args.seed)
if args.ic_set == "all":
    # WM fix 2026-09-03 (user): FULL demo-IC set -- every success-labeled uid exactly once, in ascending
    # order (deterministic, no resampling). The training reset distribution is uniform over this set.
    ics = [int(u) for u in sorted(env._env.success_uids)]
    args.episodes = len(ics)
    print(f"[eval] FULL demo IC set: {len(ics)} uids (each once) {ics[:6]}...", flush=True)
elif args.ic_file:
    import json as _json
    _icj = _json.load(open(args.ic_file))
    _entries = _icj[args.ic_set]
    if args.ic_index is not None:
        _entries = [_entries[int(args.ic_index)]]
    ics = list(_entries)
    args.episodes = len(ics)
    print(f"[eval] ICs from {args.ic_file} set={args.ic_set}"
          + (f" index={args.ic_index}" if args.ic_index is not None else "")
          + f" -> {len(ics)} episode(s)  (--episodes/--seed IC draw overridden)")
else:
    ics = [int(u) for u in rng.choice(env._env.success_uids, size=args.episodes, replace=True)]


def reset_to_ic(ic):
    """uid int -> demo-IC reset (reset_to_uid); dict -> explicit can/goal pose
    (FullTaskEnv.reset_to, the rnd entries of eval_ics.json)."""
    if not isinstance(ic, dict):
        return reset_to_uid(int(ic))
    if env._env is None:
        env._build()
    kw = {k: ic[k] for k in ("can_pos", "can_quat", "goal_pos") if ic.get(k) is not None}
    env._env.reset_to(kw)
    getattr(env, "sync_delta_target", lambda: None)()
    obs = {"is_first": True, "is_last": False, "is_terminal": False, "image": env._image()}
    if getattr(env, "_state_obs", False):   # WM fix 2026-09-03: state-input runs (bypasses adapter.reset)
        _st = env._state_vec(env._env.genv._obs()["state"])
        env._last_state = _st
        obs["state"] = _st
    for k in STAGE_KEYS + ("task_success",):
        obs[f"log_{k}"] = np.float32(0.0)
    return obs, (None, None)


def _ic_label(ic, ep):
    return f"uid{int(ic)}" if not isinstance(ic, dict) else args.ic_set + str(args.ic_index if args.ic_index is not None else ep)

# Outcome taxonomy is scope-aware: in scope='place' the can STARTS held, so
# 'picked' is trivially granted on every episode (the pre-2026-08-08 eval
# reported place timeouts as picked 1.00). Place outcomes: placed_v2 (the
# scope's terminal success) / tipped / timeout.
SCOPE = str(cfg.env.scope)
success_key = {"place": "placed_v2", "contact": "contact", "carrycontact": "contact", "touchgoal": "touched_goal", "reach": "reached", "reach_goal": "reached_goal", "full": "nested"}.get(SCOPE, "picked")  # WM fix 2026-09-03: scope-aware (was picked for touchgoal); full -> nested (END-TO-END arm, 2026-09-05)
results = []
counts = {success_key: 0, "tipped": 0, "timeout": 0, "hang": 0, "restore_failed": 0}   # restore_failed = a bank entry that did not survive the entry restore in THIS process (history-dependent physics; carrycontact smoke 3290190, 2026-09-05) -- counted as a failure, never skipped silently
# amendment (j) 2026-09-07: nested_proxy == the old `nested` (training proxy: sticky contact + grip commanded open + both upright,
# terminating); nested_honest = genv._nested() after 100 settle steps at the episode end (scope=full only; False elsewhere);
# placed_v2 is now computed in scope=full too (full_env, logged only); `placed` is the STALE base-world band (0.12-0.18 m).
STAGES = ("picked", "placed", "placed_v2", "contact", "contact_push", "nested", "nested_proxy", "nested_honest", "slide_success")   # END-TO-END arm: success-by-stage (every stage granted during the episode), reported alongside the scope success; contact_push = stricter contact, logged only (2026-09-07, amendment (g))
stage_counts = {k: 0 for k in STAGES}   # hang = --ic-skip episodes (deterministic stalls), counted as failures
counts_honest = {"nested_honest": 0, "proxy_only": 0, "tipped": 0, "timeout": 0}   # scope=full outcome taxonomy under the honest predicate (amendment (j))
slide_routes = {"sustained": 0, "settle": 0}   # amendment (l): how each slide_success grant was earned
slide_fails = {}   # amendment (l): first clause that broke the window, per episode that did not earn it
pin_stats = {"n_enumerated": 0, "n_restored_match": 0, "n_restore_failed": 0, "n_hang": 0}   # amendment (j): every enumerated bank uid must be the restored one
cpush_diag = {"wrong_side": 0, "gripper_goal": 0, "push_not_contact": 0}   # contact_push (2026-09-07): contact-but-not-push episodes by reason; push-but-not-contact
ep_tiles = []   # (frames (T,64,128,3) RGB, outcome) per episode, for the grid video
_dump = []
for ep in range(args.episodes):
    t_ep = time.time()
    if ep in set(args.ic_skip):
        counts["hang"] += 1
        pin_stats["n_hang"] += 1; pin_stats["n_enumerated"] += int(not isinstance(ics[ep], dict))
        ep_stages = {k: False for k in STAGES}   # 2026-09-07: a hang is a failure at every stage (was: the previous episode's dict)
        results.append(dict(ep=ep, stages=ep_stages, uid=(int(ics[ep]) if not isinstance(ics[ep], dict) else None), ic=(ics[ep] if isinstance(ics[ep], dict) else None),
                            ic_set=(args.ic_set if (args.ic_file or args.ic_set == "all") else None), reward=0.0, seconds=0.0, video=None, outcome="hang", steps=0))
        print(f"ep{ep}: SKIPPED (known deterministic simulator stall) -> recorded as hang = failure", flush=True)
        continue
    _restored, _entry_frame = None, None
    try:
        obs, (_restored, _entry_frame) = reset_to_ic(ics[ep])
    except RuntimeError as _e:
        if "no entry survived restore" not in str(_e):
            raise
        counts["restore_failed"] += 1
        pin_stats["n_restore_failed"] += 1; pin_stats["n_enumerated"] += int(not isinstance(ics[ep], dict))
        results.append(dict(ep=ep, uid=(int(ics[ep]) if not isinstance(ics[ep], dict) else None), ic=(ics[ep] if isinstance(ics[ep], dict) else None),
                            ic_set=(args.ic_set if (args.ic_file or args.ic_set == "all") else None), reward=0.0, seconds=0.0, video=None, outcome="restore_failed", steps=0,
                            stages={k: False for k in STAGES}))
        print(f"ep{ep}: bank entry did not survive restore ({str(_e)[:80]}...) -> recorded as restore_failed = failure", flush=True)
        continue
    if not isinstance(ics[ep], dict):
        # amendment (j): the episode runs from EXACTLY the enumerated entry (no substitution, ever)
        pin_stats["n_enumerated"] += 1
        assert _restored == int(ics[ep]), f"ep{ep}: enumerated uid {int(ics[ep])} but the env restored uid {_restored} (substitution)"
        pin_stats["n_restored_match"] += 1
    state = agent.get_initial_state(1)
    trans = pack(obs, 0.0)
    frames = [obs["image"]]
    done, info, ep_reward, t = False, {}, 0.0, 0
    _prev_state = None
    while not done and t * REPEAT < args.max_steps:
        act, state = agent.act(trans, state, eval=(args.mode == "mode"))
        a = act[0].detach().cpu().numpy().astype(np.float32)
        _prev_state = np.asarray(obs.get("state"), np.float64).reshape(-1) if obs.get("state") is not None else None
        obs, reward, done, info = env.step(a)
        frames.append(obs["image"])
        trans = pack(obs, float(reward))
        ep_reward += float(reward)
        t += 1
    _outcome_honest = None
    if SCOPE in ("full", "contact", "carrycontact") and not info.get("end_of_episode"):
        # amendment (l) + guard fix: the episode ended on THIS loop's horizon (the adapter never saw `done`), so the
        # held continuation has not run yet. The guard keys off the explicit end_of_episode flag -- keying it off
        # info["slide_success"] was ALWAYS False, because genesis_can_env.step writes that key on every step, so the
        # settle was skipped for every timeout episode and slide_success was structurally 0 there.
        _es = env._env.genv.end_of_episode()
        info["slide_success"] = _es["slide_success"]; info["slide_route"] = _es["slide_route"]
        info["slide_fail_reason"] = _es["slide_fail_reason"]; info["slide_fail_frame"] = _es["slide_fail_frame"]
        info["end_of_episode"] = True
        if SCOPE == "full":
            info["nested_honest"] = _es["nested"]
    if info.get("slide_route"):
        slide_routes[info["slide_route"]] = slide_routes.get(info["slide_route"], 0) + 1
    elif info.get("slide_fail_reason"):
        slide_fails[info["slide_fail_reason"]] = slide_fails.get(info["slide_fail_reason"], 0) + 1
    if SCOPE == "full":
        # nested_honest (amendment (j), review S1-1): if the episode terminated inside env.step the adapter already ran the
        # settle; if it ended on THIS loop's horizon the adapter saw no `done`, so run the env's settled predicate now.
        # Either way exactly one 100-step settle per episode, after the last decision (steps/outcome/reward unchanged).
        if info.get("nested_honest") is None:
            info["nested_honest"] = bool(env._env.genv._nested())
        info["nested_proxy"] = bool("nested" in env._env._granted or info.get("nested"))
        _outcome_honest = ("nested_honest" if info["nested_honest"] else "proxy_only" if info["nested_proxy"]
                           else "tipped" if bool(info.get("tipped")) else "timeout")
        counts_honest[_outcome_honest] += 1
    success = success_key in env._env._granted or bool(info.get(success_key))
    tipped = bool(info.get("tipped"))
    outcome = success_key if success else ("tipped" if tipped else "timeout")   # scope=full: `nested` here is the PROXY (kept so per-episode outcome/steps reproduce the cells of record); see outcome_honest
    counts[outcome] += 1
    _gr = set(getattr(env._env, "_granted", set())) | {k for k in STAGES if bool(info.get(k))}
    ep_stages = {k: bool(k in _gr) for k in STAGES}
    for k in STAGES: stage_counts[k] += int(ep_stages[k])
    # contact_push diagnostics (2026-09-07): first-grant frames + WHY a `contact` episode fails the stricter test
    ep_cdiag = dict(contact_frame=info.get("contact_frame"), contact_push_frame=info.get("contact_push_frame"),
                    contact_gripper_goal=bool(info.get("contact_gripper_goal")), contact_farside=bool(info.get("contact_farside")),
                    contact_farside_wrist=bool(info.get("contact_farside_wrist")))   # wrist-based projection (withdrawn definition), for the tool-vs-wrist comparison
    if ep_stages["contact"] and not ep_stages["contact_push"]:
        _reason = "wrong_side" if not ep_cdiag["contact_farside"] else "gripper_goal"
        ep_cdiag["fail_reason"] = _reason; cpush_diag[_reason] += 1
    if ep_stages["contact_push"] and not ep_stages["contact"]: cpush_diag["push_not_contact"] += 1
    if args.dump_entries and success and _prev_state is not None and _prev_state.shape[0] >= 17:
        # PHASE PLAN pol-E bank: the state BEFORE the decision on which the grant fired (bank convention frame k)
        _sv = _prev_state
        _dump.append(dict(uid=(int(ics[ep]) if not isinstance(ics[ep], dict) else -1), ic_index=int(ep), frame=int(t - 1),
                          qpos=[float(x) for x in _sv[:6]],
                          # amendment (j) / review S2-4: PHYSICAL grip (the adapter's own map), the units the restore reads
                          grip_cmd=(env.grip_phys(a) if len(a) >= 7 else float(_sv[6])), grip_cmd_raw=(float(a[6]) if len(a) >= 7 else None),
                          grip_units="physical01", bank_version=BANK_VERSION,
                          grip_obs=float(_sv[6]), can_pos=[float(x) for x in _sv[8:11]], can_quat=[float(x) for x in _sv[11:15]],
                          goal_xy=[float(x) for x in _sv[15:17]], source=str(CKPT), scope=str(SCOPE), mode=str(args.mode)))
    dt = time.time() - t_ep

    s = args.scale
    vid = OUT / f"ep{ep}_{_ic_label(ics[ep], ep)}_{outcome}.mp4"
    h, w = 64 * s, 128 * s
    vw = cv2.VideoWriter(str(vid), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
    for f in frames:
        tile = np.hstack([f[..., :3], f[..., 3:]])[:, :, ::-1]   # top|wrist, BGR
        vw.write(cv2.resize(tile, (w, h), interpolation=cv2.INTER_NEAREST))
    vw.release()

    ep_tiles.append((np.stack([np.hstack([f[..., :3], f[..., 3:]])
                               for f in frames]), outcome))
    results.append(dict(ep=ep, stages=ep_stages, contact_diag=ep_cdiag, restored_uid=_restored, entry_frame=_entry_frame, outcome_honest=_outcome_honest,
                        slide_route=info.get("slide_route"), slide_fail_reason=info.get("slide_fail_reason"), slide_fail_frame=info.get("slide_fail_frame"),
                        uid=(int(ics[ep]) if not isinstance(ics[ep], dict) else None),
                        ic=(ics[ep] if isinstance(ics[ep], dict) else None), ic_set=(args.ic_set if (args.ic_file or args.ic_set == "all") else None), outcome=outcome, steps=t,
                        reward=ep_reward, seconds=round(dt, 1), video=str(vid)))
    print(f"ep{ep}: {_ic_label(ics[ep], ep)} {outcome} ({t} steps, r={ep_reward:.1f}, "
          f"{dt:.1f} s)", flush=True)

# ---- single grid video: every episode as one tile (top|wrist), outcome-labeled.
# Shorter episodes freeze on their last frame; success green / tipped red / timeout gray.
grid_path = None
if ep_tiles:
    T = max(len(fr) for fr, _ in ep_tiles)
    cols = min(4, len(ep_tiles))
    rows = -(-len(ep_tiles) // cols)
    s = args.scale
    th, tw = 64 * s, 128 * s
    OUTCOL = {success_key: (0, 200, 0), "tipped": (0, 0, 220), "timeout": (128, 128, 128)}
    grid_path = OUT / "rollouts_grid.mp4"
    gw = cv2.VideoWriter(str(grid_path), cv2.VideoWriter_fourcc(*"mp4v"),
                         args.fps, (cols * tw, rows * th))
    for t in range(T):
        canvas = np.zeros((rows * th, cols * tw, 3), np.uint8)
        for i, (fr, outcome) in enumerate(ep_tiles):
            tile = fr[min(t, len(fr) - 1)][:, :, ::-1]           # RGB->BGR
            tile = cv2.resize(tile, (tw, th), interpolation=cv2.INTER_NEAREST)
            cv2.putText(tile, outcome[:7], (2, 10), cv2.FONT_HERSHEY_PLAIN,
                        0.8, OUTCOL.get(outcome, (255, 255, 255)), 1, cv2.LINE_AA)
            r0, c0 = (i // cols) * th, (i % cols) * tw
            canvas[r0:r0 + th, c0:c0 + tw] = tile
        gw.write(canvas)
    gw.release()
    print(f"[eval] grid video ({len(ep_tiles)} eps tiled) -> {grid_path}", flush=True)

if args.dump_entries:
    json.dump(_dump, open(args.dump_entries, "w"), indent=1)
    print(f"[eval] dumped {len(_dump)} entries -> {args.dump_entries}", flush=True)
n = max(len(results), 1)
summary = dict(
    checkpoint=str(CKPT), episodes=len(results), mode=args.mode, seed=args.seed,
    max_steps=args.max_steps, ic_mode="demo", scope=SCOPE,
    **{success_key: counts[success_key] / n},
    tipped=counts["tipped"] / n,
    timeout=counts["timeout"] / n,
    restore_failed=counts["restore_failed"] / n,
    stages={k: stage_counts[k] / n for k in STAGES},   # END-TO-END arm: success-by-stage rates (2026-09-05)
    contact_push_diag=dict(n_contact=stage_counts["contact"], n_contact_push=stage_counts["contact_push"], **cpush_diag),   # 2026-09-07
    # amendment (j) 2026-09-07 -------------------------------------------------------------------------------------
    slide_success=(stage_counts["slide_success"] / n if SCOPE in ("full", "contact", "carrycontact") else None),   # amendment (l): statistic of record for the slide phase and end-to-end
    slide_routes=dict(slide_routes), slide_fails=dict(slide_fails),
    nested_proxy=(stage_counts["nested_proxy"] / n if SCOPE == "full" else None),
    nested_honest=(stage_counts["nested_honest"] / n if SCOPE == "full" else None),
    outcomes_honest=(dict(counts_honest) if SCOPE == "full" else None),
    eval_fixes="j", entries_pinned=True, pin_stats=dict(pin_stats),
    # HARDWARE PROVENANCE (2026-09-08): the CPU class is an axis of the result (EVAL_FIXES 7.2 -- a cross-class re-run
    # moved `contact` by 0.100), so it belongs IN the cell, not in a sidecar a reader has to know to look for.
    node=os.environ.get("SLURMD_NODENAME") or __import__("socket").gethostname(),
    cpu_model=next((l.split(":", 1)[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name")), "unknown"),
    ncpus_machine=sum(1 for l in open("/proc/cpuinfo") if l.startswith("processor")),
    slurm_job=os.environ.get("SLURM_JOB_ID"),
    bank_path=(str(args.entry_bank) if args.entry_bank else None), bank_sha256=BANK_SHA256, bank_version=BANK_VERSION_IN,
    world_shelf_top_z=float(getattr(env._env, "shelf_top_z", float("nan"))),
    stage_notes={"placed": "STALE base-world band BOX_TOP_Z+[0.01,0.07] = 0.12-0.18 m (unearnable in gc_kp4_riser3_shelf6; CONFOUNDS row 47) -- use placed_v2",
                 "placed_v2": "release predicate on the WORLD shelf band (shelf_top_z+[0.01,0.07]), sustained 10 frames; computed in every scope incl. full (logged only there)",
                 "nested": "TRAINING PROXY (== nested_proxy): sticky contact + grip commanded open + both upright; terminates the episode",
                 "nested_honest": "genv._nested(): 100 settle steps at the episode end, centre distance <= NESTED_TOUCH_DIST, picked, both upright (the DP/RLPD path's predicate); scope=full only",
                 "slide_success": "amendment (l): picked AND pick-can/goal solver contact AND grip commanded < 0.3 AND can in the shelf footprint with tilt < 20 deg, sustained 3 decisions (12 env frames). Both scopes where it is the statistic of record terminate at the first frame it could hold, so the window is evaluated over the first 12 frames of the post-episode settle with the last command held; route sustained|settle per episode"},
    mean_steps=float(np.mean([r["steps"] for r in results])),
    mean_reward=float(np.mean([r["reward"] for r in results])),
    per_episode=results,
)
# amendment (j): the enumerated bank uids ARE the restored ones (or restore_failed / hang), never a substitute
assert pin_stats["n_restored_match"] + pin_stats["n_restore_failed"] + pin_stats["n_hang"] == pin_stats["n_enumerated"], pin_stats
print(f"[eval] pinned entries: {pin_stats}", flush=True)
(OUT / "metrics.json").write_text(json.dumps(summary, indent=1))
if args.append_metrics:
    _line = {"step": int(CKPT_STEP) if CKPT_STEP is not None else -1,
             f"eval/{success_key}": summary[success_key],
             "eval/tipped": summary["tipped"], "eval/timeout": summary["timeout"],
             "eval/mean_steps": summary["mean_steps"],
             "eval/mode_flag": 1.0 if args.mode == "mode" else 0.0}
    with open(pathlib.Path(args.append_metrics) / "metrics.jsonl", "a") as _f:
        _f.write(json.dumps(_line) + "\n")   # O_APPEND: atomic for small lines
    print(f"[eval] appended eval/* line (step {_line['step']}) to {args.append_metrics}/metrics.jsonl")
print(f"\n[eval] {n} episodes ({args.mode}, demo ICs): "
      f"{success_key} {summary[success_key]:.2f}  tipped {summary['tipped']:.2f}  "
      f"timeout {summary['timeout']:.2f}  mean_steps {summary['mean_steps']:.0f}")
print(f"[eval] wrote {OUT}/metrics.json + {len(results)} mp4s")

# ---- wandb (opt-in; NEVER fatal to the eval -- metrics.json is already on disk) ----
if args.wandb:
    try:
        import wandb  # noqa: E402

        run_name = f"{RUN_DIR.name}-eval"
        if CKPT_STEP is not None:
            run_name += f"-step{int(CKPT_STEP)}"
        wb = wandb.init(
            entity="jambotime", project="r2dreamer_genesis", name=run_name,
            job_type="eval", tags=["eval", SCOPE],
            config=dict(checkpoint=str(CKPT), ckpt_step=CKPT_STEP, mode=args.mode,
                        seed=args.seed, episodes=len(results), max_steps=args.max_steps,
                        ic_mode="demo", scope=SCOPE, train_run=RUN_DIR.name),
        )
        scalars = {
            f"eval/{success_key}": summary[success_key],
            "eval/tipped": summary["tipped"],
            "eval/timeout": summary["timeout"],
            "eval/mean_steps": summary["mean_steps"],
            "eval/mean_reward": summary["mean_reward"],
            "eval/episodes": len(results),
        }
        wb.log(scalars)
        if grid_path is not None:
            try:
                wb.log({"video/rollouts_grid": wandb.Video(
                    str(grid_path), format="mp4",
                    caption=f"{len(results)} eps (top|wrist per tile): "
                            + " ".join(f"ep{r['ep']}={r['outcome']}" for r in results))})
            except Exception as e:  # a bad mp4 must not sink the rest
                print(f"[eval] wandb grid video upload failed: {e}")
        wb.summary.update(scalars)
        wb.finish()
        print(f"[eval] wandb run logged: {wb.url}")
    except Exception as e:
        print(f"[eval] wandb logging FAILED (eval results unaffected): {e}")
