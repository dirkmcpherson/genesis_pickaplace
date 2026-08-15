"""CELL B: harvest ManiSkill-quality demos for OUR pick task from the r2dreamer champion.

WHY (paper/RESULTS_MATRIX_2026-08-15.md, "Positive controls"): our RLPD shows a
stable ~1/3-seeds / ~6-picks-per-45 signature that four waves (density, diversity,
demo source, action reference) failed to move. Cell B discriminates the two
remaining explanations:
  * RLPD ignites reliably on CLEAN demos  -> the DATASET is the problem
    (human teleop tapes: long, slow, replay-gap-laden, one terminal reward).
  * Same ~1/3 signature on clean demos    -> METHOD/TASK is the problem.

WHAT "CLEAN" MEANS HERE (the four ManiSkill properties our human set lacks):
  1. in-sim, closed-loop      -> ZERO replay gap (the tape IS what the sim executed)
  2. native delta actions     -> recorded in the very space the learner acts in
  3. short / fast             -> a competent policy picks in ~10x fewer frames than
                                 the human demonstrator (human pick-phase median 660)
  4. clean successes only     -> every kept tape ends in a hardened pick

TEACHER: r2dreamer champion (0.91 sampled / 1.00 mode on demo ICs, hardened
predicate). It acts in DELTA-JOINT space at ACTION_REPEAT 4 with delta_cap 0.025 /
leash 5x -- read from its OWN run config, never hardcoded here. The policy-loading
block below is lifted from r2dreamer/eval_genesis.py (same checkpoint format, same
Dreamer rebuild, same act() convention); do not re-derive it.

RECORDING CONTRACT (mirrors baselines/rl/rerecord_delta_demos.py):
  states  (n,17) float32  obs BEFORE each SIM step   (collector convention)
  actions (n,7)  float32  the ABSOLUTE command the env EXECUTED that sim step
                          ([6 joint targets rad, grip 0..1])
Per SIM step, not per decision: the champion's delta target advances a*cap on EVERY
sim step inside its repeat-4 window, so a per-sim-step tape is a STRIDE-1 tape. The
standard stride-1 encoder (train_sacfd_full.delta_encode_transitions, delta_ref
='target') therefore applies downstream UNCHANGED -- it re-derives
a_arm = clip((cmd_t - cmd_{t-1})/cap, -1, 1), which is exactly the per-sim-step delta
the teacher commanded. Nothing about the teacher's repeat-4 leaks into the dataset.

TRUNCATION: the pick-phase rule, reused from baselines/make_pick_phase_datasets.py --
first frame with can_z > pick_z AND grip command > GRIP_CLOSED_FRAC, keep ONE frame
past it (end = k+2, because relabel_full scans frames [:-1]). Identical rule, identical
constants, on both demo sources -> the fairness requirement of the phase-1 comparison.

PROVENANCE: every npz carries the teacher checkpoint, both git hashes, the action
mode/repeat/ref/scale, and the IC uid. Filenames are ROLLOUT INDICES (>=100000, the
m1all_harvest convention) so this set can never be mistaken for a uid-keyed HUMAN set
-- the exact near-miss recorded in FABLE_HANDOFF_2026-08-13 3a.

NEGATIVE CONTROL (project standard): --teacher random rolls uniform [-1,1]^7 decisions
through the identical pipeline and MUST keep ~0. If it keeps demos, the keep predicate
is broken, not the teacher.

Usage (CPU; the venv must have tensordict/omegaconf -> r2dreamer's .venv):
  /home/j/workspace/r2dreamer/.venv/bin/python baselines/harvest_champion_demos.py \
      --outdir baselines/episodes_champion_pick --shard-idx 0 --shard-n 8
  ... --teacher random --outdir baselines/episodes_champion_negctl
  ... --merge --outdir baselines/episodes_champion_pick        # census, no sim
"""
import os

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("GENESIS_PICKAPLACE_ROOT", "/home/j/workspace/genesis_pickaplace")

import argparse
import glob
import json
import pathlib as pl
import subprocess
import sys
import time

import numpy as np

REPO = pl.Path(os.environ["GENESIS_PICKAPLACE_ROOT"])
R2D = pl.Path(os.environ.get("R2DREAMER_ROOT", "/home/j/workspace/r2dreamer"))
CHAMPION = R2D / "runs/pick_delta25d4_s0/CHAMPION_1576820.pt"

ap = argparse.ArgumentParser()
ap.add_argument("--checkpoint", default=str(CHAMPION))
ap.add_argument("--config", default=None,
                help="hydra config.yaml; default <ckpt_dir>/.hydra/config.yaml")
ap.add_argument("--outdir", default="baselines/episodes_champion_pick")
ap.add_argument("--full-outdir", default="baselines/episodes_champion_pick_full",
                help="ALSO write the UNTRUNCATED tape of every kept rollout here "
                     "(same stamps). Two uses: (a) the open-loop verify guard runs on "
                     "the full recording -- a pick-phase tape is marginal-terminal by "
                     "construction (cut 2 frames past the geometric grant, ~1-7 mm of "
                     "lift margin, see sacfd_delta_gate GATE_UIDS), so replaying it "
                     "measures the replay's TIME DILATION, not the tape's fidelity; "
                     "(b) a full-length set is what a future hold-reward arm needs. "
                     "The training tape is a bitwise PREFIX of the full one (asserted "
                     "by --merge). Empty string disables.")
ap.add_argument("--teacher", choices=["champion", "random"], default="champion",
                help="random = the NEGATIVE CONTROL teacher (must keep ~0)")
ap.add_argument("--mode", choices=["mode", "sample"], default="mode",
                help="mode = actor.mode (deterministic; the champion's 1.00 protocol). "
                     "sample = actor.rsample (its 0.91 protocol); REQUIRED for "
                     "--attempts > 1, because repeating a deterministic rollout on a "
                     "fixed IC reproduces the same tape and adds no data.")
ap.add_argument("--attempts", type=int, default=1,
                help="max rollouts per IC uid; stops at the first keep. >1 asserts "
                     "--mode sample.")
ap.add_argument("--max-sim-steps", type=int, default=1200,
                help="per-rollout horizon in SIM steps (the champion's eval protocol "
                     "of record; = 300 decisions at repeat 4)")
ap.add_argument("--uids-from", default="baselines/episodes_pick_phase",
                help="IC uid list source: the HUMAN pick-phase demo set, so the model "
                     "demos are harvested from the SAME ICs the human demos occupy "
                     "(matched-IC comparability). Intersected with the env's "
                     "resettable success_uids.")
ap.add_argument("--uids", type=int, nargs="*", default=None,
                help="explicit IC uid list, overriding --uids-from (pass-2 refills)")
ap.add_argument("--shard-idx", type=int, default=0)
ap.add_argument("--shard-n", type=int, default=1)
ap.add_argument("--rollout-base", type=int, default=100000,
                help="filename/uid base; each shard owns [base + 1000*shard_idx, ...) "
                     "so parallel shards never collide")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--device", default="cpu")
ap.add_argument("--torch-threads", type=int, default=2)
ap.add_argument("--merge", action="store_true",
                help="no sim: merge shard manifests + print the census")
args = ap.parse_args()

OUT = REPO / args.outdir
STAMP_KEYS = ("source", "teacher_ckpt", "teacher_kind", "act_mode", "delta_ref",
              "delta_scale", "teacher_action_repeat", "tape_stride", "ic_uid",
              "git_genesis", "git_r2dreamer", "schema")


def _git(path):
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=str(path),
                              capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return "unknown"


# --------------------------------------------------------------------------- merge
def merge_and_census():
    recs, cfgs = {}, []
    for m in sorted(glob.glob(str(OUT / "manifest_shard*.json"))):
        blob = json.loads(pl.Path(m).read_text())
        cfgs.append(blob["config"])
        for r in blob["records"]:
            recs[r["rollout"]] = r
    kept = [r for r in recs.values() if r["kept"]]
    files = sorted(glob.glob(str(OUT / "*.npz")))
    lens = []
    for f in files:
        d = np.load(f, allow_pickle=True)
        lens.append(int(d["n"]))
    (OUT / "manifest.json").write_text(json.dumps(
        dict(configs=cfgs, n_rollouts=len(recs), n_kept=len(kept),
             records=[recs[k] for k in sorted(recs)]), indent=1))
    print(f"[census] rollouts={len(recs)}  kept={len(kept)}  "
          f"yield={len(kept) / max(1, len(recs)):.2f}  files={len(files)}")
    if lens:
        lens = np.array(lens)
        print(f"[census] kept length (frames): min {lens.min()} / median "
              f"{int(np.median(lens))} / max {lens.max()} / mean {lens.mean():.0f}")
        print(f"[census] total frames {int(lens.sum())}")
    dec = [r["decisions"] for r in kept if r.get("decisions")]
    if dec:
        print(f"[census] teacher decisions to hardened pick: min {min(dec)} / "
              f"median {int(np.median(dec))} / max {max(dec)}")
    FULLD = (REPO / args.full_outdir) if args.full_outdir else None
    if FULLD is not None and FULLD.is_dir():
        n_pref = 0
        for f in files:
            g = FULLD / pl.Path(f).name
            if not g.exists():
                continue
            a, b = np.load(f, allow_pickle=True), np.load(g, allow_pickle=True)
            n = int(a["n"])
            assert np.array_equal(a["states"], b["states"][:n]), f
            assert np.array_equal(a["actions"], b["actions"][:n]), f
            n_pref += 1
        fl = np.array([int(np.load(g, allow_pickle=True)["n"])
                       for g in sorted(glob.glob(str(FULLD / "*.npz")))])
        print(f"[census] PREFIX CHECK: {n_pref}/{len(files)} training tapes are bitwise "
              f"prefixes of the full recordings")
        if len(fl):
            print(f"[census] FULL tape length: min {fl.min()} / median "
                  f"{int(np.median(fl))} / max {fl.max()} / total {int(fl.sum())}")
    outc = {}
    for r in recs.values():
        outc[r["outcome"]] = outc.get(r["outcome"], 0) + 1
    print(f"[census] outcomes {outc}")
    return


if args.merge:
    merge_and_census()
    sys.exit(0)

# --------------------------------------------------------------------- teacher setup
import torch  # noqa: E402

torch.set_num_threads(args.torch_threads)
from omegaconf import OmegaConf  # noqa: E402

sys.path.insert(0, str(R2D))
sys.path.insert(0, str(REPO / "baselines"))
sys.path.insert(0, str(REPO / "baselines" / "rl"))

CKPT = pl.Path(args.checkpoint).expanduser().resolve()
RUN_DIR = CKPT.parent
CFG_PATH = pl.Path(args.config) if args.config else RUN_DIR / ".hydra" / "config.yaml"
assert CKPT.exists(), CKPT
assert CFG_PATH.exists(), CFG_PATH

import tools  # noqa: E402  (r2dreamer)

tools.set_seed_everywhere(args.seed)
cfg = OmegaConf.load(CFG_PATH)
cfg.device = args.device
cfg.model.compile = False

# EVERY action-semantics parameter comes from the RUN's own config (eval_genesis's
# silent-default bug #7: passing only action_repeat evaluated delta policies in
# ABSOLUTE mode -- a different MDP). We additionally ASSERT the ones the downstream
# encoder depends on, because a mismatch here silently corrupts the dataset.
REPEAT = int(cfg.env.get("action_repeat", 1) or 1)
ACTION_MODE = str(cfg.env.get("action_mode", "absolute"))
DELTA_CAP = float(cfg.env.get("delta_cap", 0.04))
LEASH_MULT = float(cfg.env.get("delta_leash_mult", 3.0))
SCOPE = str(cfg.env.scope)
assert ACTION_MODE == "delta_joint", f"teacher is not a delta policy: {ACTION_MODE}"
assert SCOPE == "pick", f"cell B is pick scope; teacher scope={SCOPE}"

from envs.genesis import STAGE_KEYS, GenesisPick  # noqa: E402  (r2dreamer)

env = GenesisPick("pick", size=tuple(cfg.env.size), seed=args.seed, scope=SCOPE,
                  action_repeat=REPEAT, action_mode=ACTION_MODE,
                  delta_cap=DELTA_CAP, delta_leash_mult=LEASH_MULT,
                  reward_scale=float(cfg.env.get("reward_scale", 1.0)))
print(f"[harvest] teacher={args.teacher} mode={args.mode} repeat={REPEAT} "
      f"action_mode={ACTION_MODE} cap={DELTA_CAP} leash_mult={LEASH_MULT} "
      f"scope={SCOPE} max_sim_steps={args.max_sim_steps}", flush=True)

agent = None
if args.teacher == "champion":
    # ---- lifted from r2dreamer/eval_genesis.py (checkpoint parsing lives there) ----
    from tensordict import TensorDict  # noqa: E402
    from dreamer import Dreamer  # noqa: E402

    agent = Dreamer(cfg.model, env.observation_space, env.action_space).to(args.device)
    ck = torch.load(CKPT, map_location=args.device, weights_only=False)
    CKPT_STEP = ck.get("step")
    missing, unexpected = agent.load_state_dict(ck["agent_state_dict"], strict=False)
    assert not any(k.startswith(("actor.", "rssm.", "encoder.")) for k in missing), missing
    agent.clone_and_freeze()          # act() runs on the _frozen_* clones
    agent.requires_grad_(False)
    agent.eval()
    print(f"[harvest] champion loaded step={CKPT_STEP} "
          f"(missing {len(missing)}, unexpected {len(unexpected)})", flush=True)

    def pack(obs, reward):
        """obs dict -> (B=1,) TensorDict (verbatim from eval_genesis.pack)."""
        d = {k: torch.as_tensor(np.asarray(v)[None]) for k, v in obs.items()}
        d["reward"] = torch.tensor([reward], dtype=torch.float32)
        td = TensorDict(d, batch_size=(1,), device="cpu")
        for k in td.keys():
            if td[k].ndim == 1:
                td[k] = td[k].unsqueeze(-1)
        return td.to(args.device)
else:
    CKPT_STEP = None
    _rng = np.random.default_rng(args.seed + 7919)

# ---------------------------------------------------------------- env instrumentation
print("[harvest] building the Genesis world (~20-35 s)...", flush=True)
_t0 = time.time()
if env._env is None:
    env._build()
print(f"[harvest] world built in {time.time() - _t0:.1f} s", flush=True)

import pick_env  # noqa: E402  (genesis_pickaplace)

PICK_Z = float(env._env.pick_z)
GRIP_CLOSED_FRAC = pick_env.GRIP_CLOSED_FRAC
# make_pick_phase_datasets pins PICK_Z = 0.1505 as a module constant; assert the live
# world agrees rather than trusting either copy.
assert abs(PICK_Z - 0.1505) < 1e-6, PICK_Z

# Record by WRAPPING the inner FullTaskEnv.step -- never by re-implementing the delta
# integration. The adapter (r2dreamer/envs/genesis.py GenesisPick.step) integrates the
# persistent target and hands the inner env a NORMALIZED ABSOLUTE action; the inner env
# runs a_phys = denormalize_action(that). So denormalize_action(a_norm) IS the command
# the sim executed, per sim step, with no second implementation to drift.
_inner = env._env
_orig_step = _inner.step
_rec = dict(states=[], actions=[], s_prev=None, on=False)


def _recording_step(a_norm):
    if _rec["on"]:
        _rec["states"].append(np.asarray(_rec["s_prev"], dtype=np.float32))
        a_phys = pick_env.denormalize_action(a_norm)
        # cross-check against the adapter's own integrated target (float32 round-trip
        # through normalize/denormalize only; a mismatch means the adapter changed)
        assert np.max(np.abs(a_phys[:6] - np.asarray(env._dj_target))) < 1e-4, \
            (a_phys[:6], env._dj_target)
        _rec["actions"].append(np.asarray(a_phys, dtype=np.float32))
    out = _orig_step(a_norm)
    if _rec["on"]:
        _rec["s_prev"] = np.asarray(out[0], dtype=np.float32)
    return out


_inner.step = _recording_step


def reset_to_uid(uid):
    """eval_genesis.reset_to_uid, plus capture of the 17-dim reset state."""
    s0, _info = _inner.reset(options={"uid": int(uid)})
    env.sync_delta_target()          # adapter target must be re-seeded (bypassed reset)
    obs = {"is_first": True, "is_last": False, "is_terminal": False,
           "image": env._image()}
    for k in STAGE_KEYS + ("task_success",):
        obs[f"log_{k}"] = np.float32(0.0)
    return obs, np.asarray(s0, dtype=np.float32)


# ------------------------------------------------------------------------- IC uid set
if args.uids:
    uid_pool = [int(u) for u in args.uids]
    src_note = "explicit --uids"
else:
    uid_pool = sorted(int(pl.Path(p).stem)
                      for p in glob.glob(str(REPO / args.uids_from / "*.npz")))
    src_note = args.uids_from
resettable = set(int(u) for u in _inner.genv.placements
                 if _inner.genv.placements[u].get("label") == "success")
skipped = [u for u in uid_pool if u not in resettable]
uid_pool = [u for u in uid_pool if u in resettable]
print(f"[harvest] IC uids from {src_note}: {len(uid_pool)} resettable "
      f"({len(skipped)} not in the placements map: {skipped})", flush=True)
shard = uid_pool[args.shard_idx::args.shard_n]
assert args.attempts == 1 or args.mode == "sample", \
    "--attempts > 1 with --mode mode repeats a deterministic rollout; use --mode sample"

OUT.mkdir(parents=True, exist_ok=True)
FULL = (REPO / args.full_outdir) if args.full_outdir else None
if FULL is not None:
    FULL.mkdir(parents=True, exist_ok=True)
GIT_G, GIT_R = _git(REPO), _git(R2D)
STAMP = dict(source="r2dreamer_champion_harvest", teacher_ckpt=str(CKPT),
             teacher_kind=args.teacher, act_mode=args.mode, delta_ref="target",
             delta_scale=DELTA_CAP, teacher_action_repeat=REPEAT, tape_stride=1,
             git_genesis=GIT_G, git_r2dreamer=GIT_R,
             schema="states(n,17) actions(n,7 ABSOLUTE [6 rad, grip 0..1]) "
                    "uid=rollout_index ic_uid=IC label=success stage=picked")


# ------------------------------------------------------------------------- rollout
def roll(uid):
    """One closed-loop rollout. Returns (record, arrays|None)."""
    obs, s0 = reset_to_uid(uid)
    _rec["states"], _rec["actions"], _rec["s_prev"], _rec["on"] = [], [], s0, True
    if agent is not None:
        state = agent.get_initial_state(1)
        trans = pack(obs, 0.0)
    done, info, t = False, {}, 0
    t_ep = time.time()
    while not done and t * REPEAT < args.max_sim_steps:
        if agent is not None:
            act, state = agent.act(trans, state, eval=(args.mode == "mode"))
            a = act[0].detach().cpu().numpy().astype(np.float32)
        else:
            a = _rng.uniform(-1.0, 1.0, 7).astype(np.float32)
        obs, reward, done, info = env.step(a)
        if agent is not None:
            trans = pack(obs, float(reward))
        t += 1
    _rec["on"] = False
    granted = set(_inner._granted)
    picked = ("picked" in granted) or bool(info.get("picked"))
    tipped = bool(info.get("tipped"))
    outcome = "picked" if picked else ("tipped" if tipped else "timeout")
    S = np.stack(_rec["states"]).astype(np.float32) if _rec["states"] else None
    A = np.stack(_rec["actions"]).astype(np.float32) if _rec["actions"] else None
    rec = dict(ic_uid=int(uid), outcome=outcome, decisions=int(t),
               sim_steps=int(0 if S is None else len(S)),
               seconds=round(time.time() - t_ep, 1), kept=False)
    if not picked or S is None:
        return rec, None, None
    # --- pick-phase truncation, the SAME rule as make_pick_phase_datasets ----------
    grant = np.flatnonzero((S[:, 10] > PICK_Z) & (A[:, 6] > GRIP_CLOSED_FRAC))
    if not len(grant):
        rec["outcome"] = "picked_no_grant"   # hardened pick without the geometric proxy
        return rec, None, None
    k = int(grant[0])
    if k == 0:
        rec["outcome"] = "grant_at_frame0"   # relabel_full drops these
        return rec, None, None
    end = min(k + 2, len(S))
    rec.update(kept=True, grant_frame=k, n=int(end), n_full=int(len(S)))
    return rec, dict(states=S[:end], actions=A[:end], n=int(end),
                     label="success", stage="picked"), \
        dict(states=S, actions=A, n=int(len(S)), label="success", stage="picked")


records = []
n_kept = 0
base = args.rollout_base + 1000 * args.shard_idx
for i, uid in enumerate(shard):
    for attempt in range(args.attempts):
        rec, arrays, arrays_full = roll(uid)
        rec["attempt"] = attempt
        if arrays is not None:
            rollout_id = base + n_kept
            rec["rollout"] = rollout_id
            np.savez_compressed(OUT / f"{rollout_id}.npz", uid=int(rollout_id),
                                **arrays, **STAMP, ic_uid=int(uid))
            if FULL is not None:
                np.savez_compressed(FULL / f"{rollout_id}.npz", uid=int(rollout_id),
                                    **arrays_full, **STAMP, ic_uid=int(uid))
            n_kept += 1
        else:
            rec["rollout"] = -(base + i * 10 + attempt)   # negative = not serialized
        records.append(rec)
        print(f"[roll] ic_uid={uid} attempt={attempt} {rec['outcome']} "
              f"decisions={rec['decisions']} sim_steps={rec['sim_steps']} "
              f"kept={rec['kept']} n={rec.get('n', '-')} ({rec['seconds']}s)", flush=True)
        if rec["kept"]:
            break

cfg_rec = dict(vars(args)) | dict(repeat=REPEAT, delta_cap=DELTA_CAP,
                                  leash_mult=LEASH_MULT, pick_z=PICK_Z,
                                  git_genesis=GIT_G, git_r2dreamer=GIT_R,
                                  ckpt_step=CKPT_STEP, n_uids=len(shard))
(OUT / f"manifest_shard{args.shard_idx}of{args.shard_n}.json").write_text(
    json.dumps(dict(config=cfg_rec, records=records), indent=1))
print(f"[harvest] shard {args.shard_idx}/{args.shard_n} DONE: {n_kept}/{len(records)} "
      f"kept -> {OUT}", flush=True)
