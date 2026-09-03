#!/usr/bin/env python3
"""Interrogate a trained DreamerV3 checkpoint: WHY did it (not) learn, not whether.

Motivating question (2026-08 world-model investigation): does the genesis arm need
``action_repeat=4``, or can dreamer run at the native 30 Hz (``action_repeat=1``)?
"Did it learn" is answered by policy eval (``genesis_eval.py``). This tool answers the
next question -- *which* part of the agent is blind -- by pushing the RECORDED demo
tapes through the loaded world model and reading the heads directly:

  1. REWARD HEAD   -- can it even see the terminal grant? MAE on rewarded vs
                      unrewarded frames, the predicted value ON the terminal frame
                      (prior finding on this project: ~94-97 against a true 100), and
                      the false-positive rate on unrewarded frames.
  2. VALUE HEAD    -- how far BACK from the terminal does the critic's rise reach?
                      Value bucketed by decisions-to-terminal, plus
                      ``value_reach_decisions`` (the largest decisions-to-terminal at
                      which the smoothed value still exceeds a fraction of its
                      terminal value) reported BOTH in decisions and in SIM STEPS
                      (decisions x action_repeat), so repeat-1 and repeat-4 runs are
                      directly comparable. THIS is the repeat-1-vs-4 diagnostic: a
                      critic whose reach is ~60 decisions covers 60 sim steps at
                      repeat 1 but 240 at repeat 4.
  3. IMAGINATION   -- from latents sampled at several decisions-to-terminal buckets,
                      roll the ACTOR + model forward ``imag_horizon`` steps through
                      ``ImagBehavior._imagine`` / ``._compute_target`` (the real code
                      paths, not a reimplementation) and report the mean imagined
                      lambda-return and the fraction of imagined rollouts that ever
                      predict a reward.
  4. REPLAY REWARD DENSITY -- parse the run's ``metrics.jsonl`` for the
                      reward-frames-per-batch metric (project reference:
                      ``train/data/reward_frames``). If that metric is ABSENT the tool
                      says so explicitly and lists the reward-ish keys it did find --
                      it never substitutes a different metric.
  5. JSON dump of everything + a compact printed summary, so several checkpoints
     (repeat 1 vs repeat 4, different demo sources) can be diffed in a table.

--------------------------------------------------------------------------------
HONESTY / TEST COVERAGE -- READ BEFORE TRUSTING A NUMBER
--------------------------------------------------------------------------------
*** NOTHING HERE HAS BEEN RUN AGAINST A REAL dv3 CHECKPOINT. ***
There is no dv3 checkpoint on the laptop this was written on. Everything was written
against the actual sources in ``~/workspace/dreamerv3-torch-genesis`` (dreamer.py,
models.py, networks.py, tools.py, genesis_eval.py, convert_genesis_demos_repeat.py,
envs/genesis.py) and exercised by ``--self-test``, which builds a *tiny* Dreamer with
the REAL genesis config shapes (64x64x6 image obs, 7-dim actions, discrete RSSM,
symlog_disc reward/critic heads), writes synthetic demo tapes in the exact format
``convert_genesis_demos_repeat.py`` emits, saves a checkpoint in the exact format
``dreamer.py`` writes, and then runs every analysis path end to end.

The self-test roster (14 checks): full run on latest.pt with a chunked RSSM carry;
archived ckpt_<step>.pt giving identical numbers; the metrics.jsonl metric-ABSENT path;
run_config action_repeat mismatch REFUSING and its explicit override; missing
run_config refusing; demo repeat.json stride mismatch and terminal-reward-scale
mismatch refusing; the pretrain-cache (wm-only) checkpoint format; the demo/checkpoint
shape cross-check catching a wrong action dim; an unknown --configs name; the
value_reach and spearman maths on a hand-computable curve; a terminal-free demo dir
refusing; and the whole reward/value aggregation + printer driven by a HEALTHY
synthetic profile (96-at-terminal reward head, value rising over the last 40
decisions -> value_reach 36 decisions = 144 sim steps at repeat 4).

What the self-test therefore does NOT prove:
  - that a REAL checkpoint's ``agent_state_dict`` loads with zero missing/unexpected
    keys against the obs/act spaces this tool reconstructs (it loads them by shape,
    and a shape error WOULD raise -- but a silently *renamed* module would not);
  - that the numbers mean what the docstring says for a trained model (an untrained
    tiny model has a flat value head by construction);
  - anything about wall-clock cost on a real 1200-decision pixel tape.
Every place where an API had to be guessed rather than read is marked ``# API-GUESS:``
in the source (``grep -n API-GUESS`` before the first real run). At time of writing:
  1. ``RUN_CONFIG_DIVIDED_KEYS`` -- that dreamer.main() divides time_limit/steps/
     eval_every/log_every by action_repeat BEFORE dumping run_config.json. Depends on
     the order of two blocks in main(); if wrong, those four keys false-alarm.
  2. ``*_task_behaviors.<task>.actor.mean_layer.weight`` -- the state-dict key of the
     actor's output layer, used to cross-check the action dim. Missing key = the
     cross-check is skipped with a warning, not a wrong answer.
  3. ``*encoder._cnn.layers.0.weight`` -- likewise for the image channel count.
  4. ``ImagBehavior._imagine`` accepting a ``(1, N, ...)`` start-state layout.
  5. ``ImagBehavior._compute_target`` returning ``target`` as the list that
     ``tools.lambda_return`` unbinds, re-stacked with ``torch.stack(target, dim=1)``
     into ``(H-1, N, 1)`` -- the same line Dreamer's own actor loss uses.
Not guesses (read directly from source): the checkpoint dict layout and
``load_state_dict(strict=False)`` path (dreamer.py ~686 / genesis_eval.py), the demo
npz schema and ``repeat.json`` stamp (convert_genesis_demos_repeat.py), the obs/act
spaces (envs/genesis.py), and that ``ckpt_<step>.pt`` is a byte-copy of ``latest.pt``
(cluster/sbatch_r2dreamer.sh ~463 does ``cp latest.pt ckpt_${STEP}.pt``).

DELIBERATE DEVIATION from genesis_eval.py: that script builds the real Genesis env to
get ``observation_space`` / ``action_space``. This tool does NOT -- Genesis is heavy,
allows one world per process, and none of these diagnostics touch the simulator. The
spaces are reconstructed from ``config`` by mirroring ``envs/genesis.py``
(``GenesisPickPlace.observation_space`` / ``.action_space``) and are then CROSS-CHECKED
against (a) the demo tapes' own shapes and (b) the checkpoint's own tensor shapes. If
that reconstruction is wrong, ``load_state_dict`` raises on a size mismatch rather than
loading a subtly different agent. Checkpoint loading itself mirrors genesis_eval.py
exactly (``torch.load(..., weights_only=False)`` -> ``load_state_dict(strict=False)``).

CONVENTIONS honoured (see AUDIT_REQUEST_Fable.md / the silent-default bug family):
  - No silent defaults. The run's ``run_config.json`` is REQUIRED and every
    action-semantics key it stamps is asserted against the config rebuilt from
    ``--configs``; a mismatch REFUSES with a diff (override only with the explicit
    ``--allow-config-mismatch``). The demo dir's ``repeat.json`` stride and terminal
    reward are asserted the same way ``dreamer.py`` asserts them.
  - Everything resolved is printed before any number is produced.

USAGE
  python analysis/dv3_interrogate.py \
      --checkpoint <logdir>/latest.pt \
      --configs genesis_pixels genesis_pick_msrecipe genesis_final_rr \
      --demodir <...>/demonstrations/genesis_final_dH_r4 \
      --n-episodes 8 --device cpu --out /tmp/interrogate_dH_r4.json

  python analysis/dv3_interrogate.py --self-test      # no checkpoint needed
"""
import argparse
import json
import os
import pathlib
import shutil
import sys
import tempfile
import time

import numpy as np

# ---------------------------------------------------------------------------
# torch.compile: dreamer decorates RSSM.obs_step / img_step / preprocess /
# lambda_return with @torch.compile(disable=STATIC_CONSTANTS.DISABLE_COMPILE), which
# defaults to ENABLED. On CPU the dynamo warm-up dominates a short diagnostic run and
# recompiles on every ragged chunk length. We cannot flip STATIC_CONSTANTS (frozen
# dataclass in read-only source), so we disable dynamo via env var -- which must be
# set BEFORE torch is imported. Numerically compile is a no-op, but the choice is
# printed, never silent.
# ---------------------------------------------------------------------------
_COMPILE_CHOICE = None


def _resolve_compile_flag(argv):
    mode = "auto"
    device = "cpu"
    for i, a in enumerate(argv):
        if a == "--disable-torch-compile" and i + 1 < len(argv):
            mode = argv[i + 1]
        elif a.startswith("--disable-torch-compile="):
            mode = a.split("=", 1)[1]
        elif a == "--device" and i + 1 < len(argv):
            device = argv[i + 1]
        elif a.startswith("--device="):
            device = a.split("=", 1)[1]
    if mode not in ("auto", "yes", "no"):
        raise SystemExit(f"--disable-torch-compile must be auto|yes|no, got {mode!r}")
    disable = (mode == "yes") or (mode == "auto" and str(device).startswith("cpu"))
    return mode, disable


_COMPILE_MODE, _COMPILE_DISABLE = _resolve_compile_flag(sys.argv[1:])
if _COMPILE_DISABLE:
    os.environ["TORCHDYNAMO_DISABLE"] = "1"

import torch  # noqa: E402

if _COMPILE_DISABLE:
    try:
        import torch._dynamo as _dynamo
        _dynamo.config.disable = True
    except Exception as _e:                                    # pragma: no cover
        print(f"[warn] could not force-disable dynamo: {_e}", flush=True)


DEFAULT_DREAMER_ROOT = os.environ.get(
    "DREAMER_ROOT", os.path.expanduser("~/workspace/dreamerv3-torch-genesis"))

# genesis_eval.py copies exactly these keys out of run_config.json, for exactly the
# reason its comment gives (action semantics must match or you measure a different
# agent). We assert them instead of silently adopting them.
RUN_CONFIG_KEYS = (
    "task", "genesis_cartesian_control", "genesis_pixels", "genesis_scope",
    "genesis_workspace_limit", "size", "time_limit", "action_repeat",
    "genesis_joint_action_mode", "genesis_delta_cap",
    "genesis_delta_leash_mult", "genesis_reward_scale",
)
# API-GUESS: dreamer.main() does `config.<k> //= config.action_repeat` for these
# BEFORE it dumps run_config.json (division at dreamer.py ~430, dump at ~445), so the
# stamped value is the DIVIDED one and comparing raw would false-alarm on every run.
# This depends on the ORDER of two blocks in main(); if a future dreamer.py dumps the
# config earlier, these four keys start reporting spurious mismatches.
RUN_CONFIG_DIVIDED_KEYS = ("time_limit", "steps", "eval_every", "log_every")

DTT_BUCKETS = ((0, 0), (1, 2), (3, 5), (6, 10), (11, 20), (21, 50), (51, 100),
               (101, 10 ** 9))


def bucket_label(lo, hi):
    if lo == hi:
        return str(lo)
    if hi >= 10 ** 9:
        return f"{lo}+"
    return f"{lo}-{hi}"


BUCKET_LABELS = [bucket_label(lo, hi) for lo, hi in DTT_BUCKETS]


def bucket_of(d):
    for i, (lo, hi) in enumerate(DTT_BUCKETS):
        if lo <= d <= hi:
            return i
    raise AssertionError(d)


# ===========================================================================
# dreamer source import
# ===========================================================================
def import_dreamer(dreamer_root):
    """Import the dv3 modules the same way genesis_eval.py does (sys.path.insert)."""
    root = str(pathlib.Path(dreamer_root).expanduser().resolve())
    if not os.path.isdir(root):
        raise SystemExit(f"--dreamer-root {root} does not exist")
    if not os.path.isfile(os.path.join(root, "dreamer.py")):
        raise SystemExit(f"--dreamer-root {root} has no dreamer.py")
    if root not in sys.path:
        sys.path.insert(0, root)
    try:
        import gym  # noqa: F401
    except ImportError:
        # dreamer.py -> envs.wrappers -> `import gym`. The training env has gym; a
        # laptop analysis venv may only have gymnasium. We never CONSTRUCT a wrapper
        # here (no env is built), so aliasing is safe -- and it is announced.
        import gymnasium as _gymnasium
        sys.modules["gym"] = _gymnasium
        print("[env] `gym` not installed -- aliased to `gymnasium` for the import of "
              "envs.wrappers (no env is ever constructed by this tool)", flush=True)
    import ruamel.yaml as ruamel_yaml
    import dreamer as D
    import models
    import networks
    import tools
    return dict(root=root, D=D, models=models, networks=networks, tools=tools,
                yaml=ruamel_yaml)


def recursive_update(base, update):
    """Byte-identical to the merge in genesis_eval.py / dreamer.py __main__: nested
    dicts (encoder/decoder/actor/critic) MERGE, they do not replace."""
    for key, value in update.items():
        if isinstance(value, dict) and key in base and isinstance(base[key], dict):
            recursive_update(base[key], value)
        else:
            base[key] = value


def build_config(dv3, config_names, overrides=None):
    yaml = dv3["yaml"]
    cfg_path = pathlib.Path(dv3["root"]) / "configs.yaml"
    configs = yaml.YAML(typ="safe", pure=True).load(cfg_path.read_text())
    missing = [n for n in config_names if n not in configs]
    if missing:
        raise SystemExit(
            f"unknown --configs name(s) {missing} in {cfg_path}. Available: "
            f"{sorted(configs.keys())}. (A run trained on the CLUSTER may use a "
            f"config block that this checkout of dreamerv3-torch-genesis does not "
            f"have -- point --dreamer-root at the checkout the run used.)")
    cfg = {}
    for name in ["defaults"] + list(config_names):
        recursive_update(cfg, configs[name])
    if overrides:
        recursive_update(cfg, overrides)
    return cfg


# ===========================================================================
# obs / act spaces -- MIRROR of envs/genesis.py (no Genesis world is built)
# ===========================================================================
def build_spaces(cfg):
    """Mirror GenesisPickPlace.observation_space / .action_space + NormalizeActions.

    envs/genesis.py, verbatim behaviour:
      obs: image (H,W,6 if pixels else 3) uint8, is_first/is_last/is_terminal bool,
           reward float32; 'state' (17 joint / 18 cartesian) ONLY when NOT pixels.
      act: Box(-1,1,(7,)) for joint or cartesian abs6/delta6, else (5,).
      make_env then wraps with NormalizeActions -> Box(-1,1) (already the case here),
      TimeLimit / SelectAction / UUID (none of which touch either space).
    """
    import gym  # already importable (real or aliased) by import_dreamer
    task = str(cfg["task"])
    if not task.startswith("genesis"):
        raise SystemExit(
            f"dv3_interrogate only reconstructs spaces for genesis tasks; got "
            f"task={task!r}. Add a mirror for that suite before using this tool.")
    pixels = bool(cfg.get("genesis_pixels", False))
    cartesian = "cartesian" in task
    size = tuple(cfg["size"])
    spaces = {
        "image": gym.spaces.Box(0, 255, (*size, 6 if pixels else 3), dtype=np.uint8),
        "is_first": gym.spaces.Box(0, 1, (), dtype=bool),
        "is_last": gym.spaces.Box(0, 1, (), dtype=bool),
        "is_terminal": gym.spaces.Box(0, 1, (), dtype=bool),
        "reward": gym.spaces.Box(-np.inf, np.inf, (), dtype=np.float32),
    }
    if not pixels:
        spaces["state"] = gym.spaces.Box(
            -np.inf, np.inf, (18 if cartesian else 17,), dtype=np.float32)
    obs_space = gym.spaces.Dict(spaces)
    ctrl = str(cfg.get("genesis_cartesian_control", "delta"))
    adim = 7 if (not cartesian or ctrl in ("abs6", "delta6")) else 5
    act_space = gym.spaces.Box(-1.0, 1.0, (adim,), dtype=np.float32)
    act_space.discrete = False
    return obs_space, act_space


# ===========================================================================
# checkpoint
# ===========================================================================
def load_checkpoint_blob(path, device):
    """`latest.pt` and the archived `ckpt_<step>.pt` are the SAME format --
    cluster/sbatch_r2dreamer.sh line ~463 archives with `cp latest.pt ckpt_<step>.pt`.
    The pretrain cache (`*_wm.pt` / `*_ac.pt`, dreamer.py ~line 823) is a DIFFERENT
    format and is handled explicitly rather than failing obscurely."""
    p = pathlib.Path(path).expanduser()
    if not p.is_file():
        raise SystemExit(f"--checkpoint {p} does not exist")
    blob = torch.load(p, map_location=device, weights_only=False)
    if not isinstance(blob, dict):
        raise SystemExit(f"{p} did not deserialize to a dict (got {type(blob)})")
    keys = sorted(blob.keys())
    if "agent_state_dict" in blob:
        kind = "agent"
    elif "wm_state_dict" in blob:
        kind = "pretrain_cache_wm"
    elif "ac_state_dict" in blob:
        kind = "pretrain_cache_ac"
    else:
        raise SystemExit(
            f"{p} has none of agent_state_dict / wm_state_dict / ac_state_dict "
            f"(top-level keys: {keys}). This is not a dreamer checkpoint.")
    return blob, kind, keys


def state_dict_lookup(sd, suffix):
    hits = [k for k in sd if k.endswith(suffix)]
    return (hits[0], sd[hits[0]]) if hits else (None, None)


def build_agent(dv3, cfg, obs_space, act_space, device, task_name):
    """Mirror genesis_eval.py: Namespace(config) -> D.Dreamer(...) -> .to(device) ->
    requires_grad_(False). The Logger goes to a THROWAWAY dir: tools.Logger opens a
    SummaryWriter, and this tool must never write event files into a run's logdir."""
    D, tools = dv3["D"], dv3["tools"]
    cfg = dict(cfg)
    cfg["device"] = device
    cfg["num_actions"] = int(act_space.shape[0])
    config = argparse.Namespace(**cfg)
    tb_dir = pathlib.Path(tempfile.mkdtemp(prefix="dv3_interrogate_tb_"))
    config.logdir = tb_dir
    config.traindir = tb_dir / "train_eps"
    config.evaldir = tb_dir / "eval_eps"
    logger = tools.Logger(tb_dir, 0)          # Dreamer.__init__ reads logger.step only
    agent = D.Dreamer(obs_space, act_space, config, logger=logger,
                      dataset=None).to(device)
    agent.requires_grad_(False)
    agent.eval()
    beh_keys = list(agent._task_behaviors.keys())
    if task_name not in beh_keys:
        shutil.rmtree(tb_dir, ignore_errors=True)
        raise SystemExit(f"--task-name {task_name!r} not in the rebuilt agent's "
                         f"task behaviors {beh_keys}")
    return agent, config, tb_dir


def load_into_agent(agent, blob, kind, task_name):
    report = {"checkpoint_kind": kind}
    if kind == "agent":
        missing, unexpected = agent.load_state_dict(blob["agent_state_dict"],
                                                    strict=False)
        report["has_optims_state"] = "optims_state_dict" in blob
    elif kind == "pretrain_cache_wm":
        missing, unexpected = agent._wm.load_state_dict(blob["wm_state_dict"],
                                                        strict=False)
        report["has_optims_state"] = "wm_optims" in blob
        report["warning"] = ("pretrain-cache WORLD MODEL only: the actor and critic "
                             "in this report are RANDOMLY INITIALIZED. Sections "
                             "VALUE and IMAGINATION are meaningless.")
        print("\n*** " + report["warning"] + " ***\n", flush=True)
    else:
        beh = agent._task_behaviors[task_name]
        missing, unexpected = beh.load_state_dict(blob["ac_state_dict"], strict=False)
        report["has_optims_state"] = "ac_optims" in blob
        report["warning"] = ("pretrain-cache ACTOR-CRITIC only: the world model in "
                             "this report is RANDOMLY INITIALIZED. Sections REWARD "
                             "and IMAGINATION are meaningless.")
        print("\n*** " + report["warning"] + " ***\n", flush=True)
    report["missing_keys"] = len(missing)
    report["unexpected_keys"] = len(unexpected)
    report["missing_keys_sample"] = list(missing[:8])
    report["unexpected_keys_sample"] = list(unexpected[:8])
    print(f"[ckpt] loaded ({kind}): missing {len(missing)}, unexpected "
          f"{len(unexpected)}", flush=True)
    if missing:
        print(f"[ckpt]   missing e.g. {list(missing[:5])}", flush=True)
    if unexpected:
        print(f"[ckpt]   unexpected e.g. {list(unexpected[:5])}", flush=True)
    return report


# ===========================================================================
# config / stride agreement -- refuse on mismatch
# ===========================================================================
def check_run_config(cfg, run_config_path, allow_mismatch):
    """Assert the run's stamped action semantics against the config rebuilt from
    --configs. genesis_eval.py adopts run_config silently; we DIFF first."""
    p = pathlib.Path(run_config_path)
    saved = json.loads(p.read_text())
    mismatches, checked = [], {}
    ar_cfg = int(cfg["action_repeat"])
    for k in RUN_CONFIG_KEYS:
        if k not in saved:
            continue
        want = cfg.get(k, "<absent-from-configs>")
        if k in RUN_CONFIG_DIVIDED_KEYS and isinstance(want, (int, float)):
            want = int(want) // ar_cfg        # dreamer.main() divides before dumping
        got = saved[k]
        same = (list(got) == list(want)) if isinstance(want, (list, tuple)) \
            else (got == want)
        checked[k] = {"run_config": got, "from_configs": want, "match": bool(same)}
        if not same:
            mismatches.append(f"  {k}: run_config={got!r} but --configs gives {want!r}")
    if mismatches:
        msg = ("CONFIG MISMATCH between the run's stamped run_config.json and the "
               "config rebuilt from --configs:\n" + "\n".join(mismatches) +
               f"\n  ({p})\nThis is the action-mode-sidecar failure family: "
               "interrogating a checkpoint under the wrong action semantics/stride "
               "measures a different agent than the one trained. Pass the --configs "
               "the run actually used, or --allow-config-mismatch to proceed anyway.")
        if not allow_mismatch:
            raise SystemExit(msg)
        print("\n*** OVERRIDDEN " + msg + "\n", flush=True)
    return {"path": str(p), "checked": checked,
            "mismatches": mismatches, "overridden": bool(mismatches and allow_mismatch),
            "stamped_action_repeat": saved.get("action_repeat"),
            "stamped_demodir": saved.get("demodir"),
            "stamped_steps": saved.get("steps"),
            "stamped_task": saved.get("task")}


def check_demo_stamp(cfg, demodir, allow_mismatch):
    """Mirror dreamer.py's demo/online TIME-BASE check (lines ~493-520), including the
    'no repeat.json means stride-1 by provenance' rule."""
    demodir = pathlib.Path(demodir).expanduser()
    if not demodir.is_dir():
        raise SystemExit(f"--demodir {demodir} does not exist")
    stamp = demodir / "repeat.json"
    ar = int(cfg["action_repeat"])
    out = {"demodir": str(demodir), "repeat_json": None}
    if stamp.exists():
        meta = json.loads(stamp.read_text())
        out["repeat_json"] = meta
        demo_repeat = int(meta["action_repeat"])
        if demo_repeat != ar:
            msg = (f"demo stride mismatch: {demodir} was encoded at action_repeat="
                   f"{demo_repeat} but the config says action_repeat={ar}. dreamer.py "
                   f"CRASHES on this for a reason -- the tapes encode a different "
                   f"time base than the model was trained on.")
            if not allow_mismatch:
                raise SystemExit(msg)
            print("*** OVERRIDDEN " + msg, flush=True)
        if "terminal_reward" in meta:
            env_scale = float(cfg.get("genesis_reward_scale", 1.0))
            if float(meta["terminal_reward"]) != env_scale:
                msg = (f"demo terminal reward {meta['terminal_reward']} != config "
                       f"genesis_reward_scale {env_scale} ({demodir}). The reward-head "
                       f"section would be comparing two different reward scales.")
                if not allow_mismatch:
                    raise SystemExit(msg)
                print("*** OVERRIDDEN " + msg, flush=True)
    elif str(cfg["task"]).startswith("genesis") and ar != 1:
        msg = (f"{demodir} has no repeat.json stride stamp, so it is stride-1 by "
               f"provenance, but action_repeat={ar}.")
        if not allow_mismatch:
            raise SystemExit(msg)
        print("*** OVERRIDDEN " + msg, flush=True)
    return out


def cross_check_shapes(agent, obs_space, act_space, episodes, task_name):
    """The reconstructed spaces are only trustworthy if the checkpoint's own tensors
    agree with them. A size mismatch would already have raised inside load_state_dict;
    these checks catch the cases that would NOT raise."""
    out = {}
    sd = agent.state_dict()
    key, w = state_dict_lookup(sd, f"_task_behaviors.{task_name}.actor.mean_layer.weight")
    if w is not None:
        # API-GUESS: module path of the actor's output layer
        # (ImagBehavior.actor -> networks.MLP.mean_layer). Read from models.py /
        # networks.py, but never verified against a real state_dict's key names.
        out["actor_out_dim"] = int(w.shape[0])
        assert int(w.shape[0]) == int(act_space.shape[0]), (
            f"checkpoint actor outputs {w.shape[0]} dims but the reconstructed action "
            f"space is {act_space.shape[0]}-dim ({key})")
    else:
        out["actor_out_dim"] = None
        print("[warn] could not find '*actor.mean_layer.weight' in the state dict -- "
              "action-dim cross-check SKIPPED", flush=True)
    key, w = state_dict_lookup(sd, "encoder._cnn.layers.0.weight")
    if w is not None:
        # API-GUESS: first conv of networks.ConvEncoder is layers[0] with weight
        # (depth, in_ch, k, k). Read from networks.py; not verified on a real ckpt.
        out["encoder_in_channels"] = int(w.shape[1])
        want = int(obs_space["image"].shape[-1])
        assert int(w.shape[1]) == want, (
            f"checkpoint image encoder takes {w.shape[1]} channels but the "
            f"reconstructed image obs has {want} ({key})")
    else:
        out["encoder_in_channels"] = None
        print("[warn] could not find '*encoder._cnn.layers.0.weight' in the state "
              "dict -- image-channel cross-check SKIPPED", flush=True)
    adims = sorted({int(ep["action"].shape[-1]) for ep in episodes.values()})
    assert adims == [int(act_space.shape[0])], (
        f"demo tapes carry action dims {adims} but the action space is "
        f"{act_space.shape[0]}-dim -- wrong demo set for this checkpoint")
    ishapes = sorted({tuple(ep["image"].shape[1:]) for ep in episodes.values()})
    want = tuple(obs_space["image"].shape)
    assert ishapes == [want], (
        f"demo tapes carry image shapes {ishapes} but the obs space image is {want}")
    out["demo_action_dim"] = adims[0]
    out["demo_image_shape"] = list(want)
    return out


# ===========================================================================
# episode selection + world-model pass
# ===========================================================================
def select_episodes(all_eps, n_terminal, n_nonterminal):
    term, nonterm = [], []
    for k in sorted(all_eps.keys()):
        ep = all_eps[k]
        for req in ("image", "action", "reward", "is_first", "is_terminal"):
            if req not in ep:
                raise SystemExit(f"demo episode {k} is missing key {req!r} "
                                 f"(has {sorted(ep.keys())})")
        (term if bool(np.any(ep["is_terminal"])) else nonterm).append(k)
    chosen = term[:n_terminal] + nonterm[:n_nonterminal]
    if not term:
        raise SystemExit(
            "no demo episode in --demodir has is_terminal=True anywhere. The reward "
            "and value sections need a terminal to measure decisions-to-terminal. "
            "(convert_genesis_demos_repeat.py writes is_terminal only on PICK tapes; "
            "a no-pick-only directory cannot answer this question.)")
    return chosen, {"n_terminal_available": len(term),
                    "n_nonterminal_available": len(nonterm),
                    "selected": chosen,
                    "selected_terminal": [k for k in chosen if k in term],
                    "selected_nonterminal": [k for k in chosen if k in nonterm]}


@torch.no_grad()
def wm_pass_with_states(agent, wm_keys, ep, device, chunk):
    """Encode the recorded observations and run the POSTERIOR rollout with the
    RECORDED actions -- exactly WorldModel._train's observe() path, minus the losses.
    Long tapes are chunked with the posterior carried across chunk boundaries
    (RSSM.observe accepts `state`), so a 1200-decision pixel tape fits on CPU.
    Returns (feat, posterior-state-dict, T)."""
    wm = agent._wm
    T = int(len(ep["reward"]))
    state = None
    feats, posts = [], []
    for s in range(0, T, chunk):
        e = min(s + chunk, T)
        batch = {k: np.asarray(ep[k][s:e])[None] for k in wm_keys}
        data = wm.preprocess(batch)
        embed = wm.encoder(data)
        post, _ = wm.dynamics.observe(embed, data["action"], data["is_first"], state)
        state = {k: v[:, -1] for k, v in post.items()}
        feats.append(wm.dynamics.get_feat(post))
        posts.append({k: v for k, v in post.items()})
    feat = torch.cat(feats, dim=1)
    post_all = {k: torch.cat([p[k] for p in posts], dim=1) for k in posts[0]}
    return feat, post_all, int(feat.shape[1])


# ===========================================================================
# analyses
# ===========================================================================
def spearman(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    if len(a) < 3:
        return None
    ra = np.argsort(np.argsort(a)).astype(float)
    rb = np.argsort(np.argsort(b)).astype(float)
    ra -= ra.mean()
    rb -= rb.mean()
    den = np.sqrt((ra ** 2).sum() * (rb ** 2).sum())
    return float((ra * rb).sum() / den) if den > 0 else None


def value_reach(per_dtt_sum, per_dtt_n, v_term, frac, window, min_count):
    """`value_reach_decisions` = the largest decisions-to-terminal at which the
    smoothed mean value still exceeds frac * value(terminal).
    `value_reach_contiguous_decisions` = the largest d such that EVERY data-bearing
    decisions-to-terminal <= d clears the bar (the honest 'how far back does the rise
    reach' number; the unconstrained one can be tripped by a single noisy bucket)."""
    if v_term is None or not np.isfinite(v_term) or v_term <= 0:
        return {"value_reach_decisions": None,
                "value_reach_contiguous_decisions": None,
                "threshold": None,
                "reason": ("terminal value is not positive "
                           f"({v_term!r}) -- a 'fraction of terminal value' "
                           "threshold is undefined; read the bucket means instead")}
    maxd = max(per_dtt_sum.keys())
    means = np.full(maxd + 1, np.nan)
    counts = np.zeros(maxd + 1, dtype=int)
    for d in per_dtt_sum:
        counts[d] = per_dtt_n[d]
        means[d] = per_dtt_sum[d] / per_dtt_n[d]
    half = max(int(window) // 2, 0)
    sm = np.full(maxd + 1, np.nan)
    for d in range(maxd + 1):
        seg = means[max(0, d - half): min(maxd + 1, d + half + 1)]
        seg = seg[~np.isnan(seg)]
        if len(seg):
            sm[d] = seg.mean()
    thresh = frac * v_term
    ok = (~np.isnan(sm)) & (sm >= thresh) & (counts >= min_count)
    reach = int(np.max(np.flatnonzero(ok))) if ok.any() else None
    cont = None
    for d in range(maxd + 1):
        if counts[d] < min_count:
            continue                       # no data at this exact d -- not a failure
        if ok[d]:
            cont = d
        else:
            break
    return {"value_reach_decisions": reach,
            "value_reach_contiguous_decisions": cont,
            "threshold": float(thresh),
            "max_decisions_to_terminal_observed": int(maxd),
            "smoothing_window": int(window),
            "min_frames_per_decision_bin": int(min_count),
            "reason": None}


class FrameStats:
    """Per-frame accumulator for sections 1 and 2.

    Deliberately separated from the model forward pass: it takes plain arrays, so the
    self-test can drive it with a HEALTHY synthetic profile (reward head ~96 at the
    terminal, value rising toward it) and check that the reported numbers -- MAE split,
    bucket means, value_reach in decisions AND sim steps -- come out right. Driving it
    from an untrained tiny model can only ever exercise the flat/degenerate branch.
    """

    def __init__(self):
        self.per_ep = []
        self.err_rewarded, self.err_unrewarded, self.fp_preds = [], [], []
        self.term_pred, self.term_true = [], []
        nb = len(DTT_BUCKETS)
        self.b_v_sum, self.b_v_n = np.zeros(nb), np.zeros(nb, dtype=int)
        self.b_r_sum = np.zeros(nb)
        self.nt_v_sum, self.nt_v_n = np.zeros(nb), np.zeros(nb, dtype=int)
        self.per_dtt_sum, self.per_dtt_n = {}, {}
        self.all_dtt, self.all_val = [], []

    def add_episode(self, key, r_pred, v_pred, c_pred, r_true, is_term, seconds=0.0,
                    on_terminal_frame=None):
        """on_terminal_frame(bucket_index, frame_index) is called for every frame at or
        before the terminal -- the imagination pool uses it to grab start latents."""
        T = len(r_true)
        term_idx = int(np.flatnonzero(is_term)[0]) if is_term.any() else None
        rewarded = r_true > 0
        err = np.abs(r_pred - r_true)
        if rewarded.any():
            self.err_rewarded.extend(err[rewarded].tolist())
        if (~rewarded).any():
            self.err_unrewarded.extend(err[~rewarded].tolist())
            self.fp_preds.extend(r_pred[~rewarded].tolist())
        rec = {"episode": key, "frames": int(T), "seconds": round(seconds, 2),
               "has_terminal": term_idx is not None, "terminal_index": term_idx,
               "recorded_total_reward": float(r_true.sum()),
               "value_mean": float(v_pred.mean()), "value_max": float(v_pred.max()),
               "cont_mean": float(c_pred.mean())}
        if term_idx is not None:
            rec["reward_pred_at_terminal"] = float(r_pred[term_idx])
            rec["reward_true_at_terminal"] = float(r_true[term_idx])
            rec["value_at_terminal"] = float(v_pred[term_idx])
            rec["cont_at_terminal"] = float(c_pred[term_idx])
            self.term_pred.append(float(r_pred[term_idx]))
            self.term_true.append(float(r_true[term_idx]))
            for t in range(term_idx + 1):
                d = term_idx - t
                b = bucket_of(d)
                self.b_v_sum[b] += v_pred[t]
                self.b_v_n[b] += 1
                self.b_r_sum[b] += r_pred[t]
                self.per_dtt_sum[d] = self.per_dtt_sum.get(d, 0.0) + float(v_pred[t])
                self.per_dtt_n[d] = self.per_dtt_n.get(d, 0) + 1
                self.all_dtt.append(d)
                self.all_val.append(float(v_pred[t]))
                if on_terminal_frame is not None:
                    on_terminal_frame(b, t)
        else:
            end = T - 1
            for t in range(T):
                b = bucket_of(end - t)
                self.nt_v_sum[b] += v_pred[t]
                self.nt_v_n[b] += 1
        self.per_ep.append(rec)
        return rec

    @staticmethod
    def _m(x):
        return float(np.mean(x)) if len(x) else None

    def reward_section(self, reward_scale, fp_threshold):
        fp_thresh = fp_threshold if fp_threshold is not None else 0.1 * reward_scale
        fp = np.asarray(self.fp_preds) if self.fp_preds else np.zeros(0)
        tp = self.term_pred
        return {
            "reward_scale_from_config": float(reward_scale),
            "n_rewarded_frames": len(self.err_rewarded),
            "n_unrewarded_frames": len(self.err_unrewarded),
            "mae_rewarded_frames": self._m(self.err_rewarded),
            "mae_unrewarded_frames": self._m(self.err_unrewarded),
            "pred_at_terminal_mean": self._m(tp),
            "pred_at_terminal_min": float(np.min(tp)) if tp else None,
            "pred_at_terminal_max": float(np.max(tp)) if tp else None,
            "true_at_terminal_mean": self._m(self.term_true),
            "pred_at_terminal_per_episode": tp,
            "false_positive_threshold": float(fp_thresh),
            "false_positive_pred_mean": float(fp.mean()) if fp.size else None,
            "false_positive_pred_max": float(fp.max()) if fp.size else None,
            "false_positive_pred_p99": (float(np.percentile(fp, 99)) if fp.size
                                        else None),
            "false_positive_frac_over_threshold": (float((fp > fp_thresh).mean())
                                                   if fp.size else None),
        }

    def value_section(self, args, action_repeat):
        nb = len(DTT_BUCKETS)
        v_term = (self.b_v_sum[0] / self.b_v_n[0]) if self.b_v_n[0] else None
        out = {
            "buckets": BUCKET_LABELS,
            "value_mean_by_dtt_bucket": [
                float(self.b_v_sum[i] / self.b_v_n[i]) if self.b_v_n[i] else None
                for i in range(nb)],
            "n_frames_by_dtt_bucket": self.b_v_n.tolist(),
            "reward_pred_mean_by_dtt_bucket": [
                float(self.b_r_sum[i] / self.b_v_n[i]) if self.b_v_n[i] else None
                for i in range(nb)],
            "value_at_terminal": float(v_term) if v_term is not None else None,
            "value_reach_fraction_of_terminal": float(args.value_reach_frac),
            "spearman_value_vs_decisions_to_terminal": spearman(self.all_dtt,
                                                                self.all_val),
            "nonterminal_control": {
                "note": ("episodes with NO terminal (no-pick demos): value bucketed by "
                         "decisions-to-END-OF-TAPE. A critic that rises here too is "
                         "responding to tape position, not to the reward."),
                "value_mean_by_bucket": [
                    float(self.nt_v_sum[i] / self.nt_v_n[i]) if self.nt_v_n[i] else None
                    for i in range(nb)],
                "n_frames_by_bucket": self.nt_v_n.tolist()},
        }
        curve_cap = int(getattr(args, "value_curve_max_decisions", 400))
        out["value_curve_by_decision"] = [
            {"decisions_to_terminal": d,
             "sim_steps_to_terminal": d * int(action_repeat),
             "value_mean": self.per_dtt_sum[d] / self.per_dtt_n[d],
             "n_frames": self.per_dtt_n[d]}
            for d in sorted(self.per_dtt_sum) if d <= curve_cap]
        out.update(value_reach(self.per_dtt_sum, self.per_dtt_n, v_term,
                               args.value_reach_frac, args.reach_window,
                               args.reach_min_count))
        for name in ("value_reach_decisions", "value_reach_contiguous_decisions"):
            d = out[name]
            out[name.replace("decisions", "sim_steps")] = (
                None if d is None else int(d) * int(action_repeat))
        out["action_repeat"] = int(action_repeat)
        return out


@torch.no_grad()
def analyse_episodes(agent, cfg, episodes, keys, device, chunk, args, action_repeat):
    wm = agent._wm
    beh = agent._task_behaviors[args.task_name]
    wm_keys = ("image", "action", "reward", "is_first", "is_terminal")
    reward_scale = float(cfg.get("genesis_reward_scale", 1.0))
    stats = FrameStats()
    # dtt bucket -> list of single-frame posterior states (imagination start latents).
    # Only the SELECTED frames are kept: caching a whole 1200-frame posterior per
    # episode is ~12 MB/episode for the real (32x32 stoch, 512 deter) RSSM.
    imag_pool = {i: [] for i in range(len(DTT_BUCKETS))}

    for k in keys:
        ep = episodes[k]
        t0 = time.time()
        feat, post, T = wm_pass_with_states(agent, wm_keys, ep, device, chunk)
        r_pred = wm.heads["reward"](feat).mode().squeeze(0).squeeze(-1).cpu().numpy()
        v_pred = beh.value(feat).mode().squeeze(0).squeeze(-1).cpu().numpy()
        c_pred = wm.heads["cont"](feat).mean.squeeze(0).squeeze(-1).cpu().numpy()

        def grab(b, t, _post=post):
            if len(imag_pool[b]) < args.imag_per_bucket:
                imag_pool[b].append(
                    {key: _post[key][:, t:t + 1].clone() for key in _post})

        rec = stats.add_episode(
            k, r_pred, v_pred, c_pred,
            np.asarray(ep["reward"], dtype=np.float64),
            np.asarray(ep["is_terminal"], dtype=bool),
            seconds=time.time() - t0, on_terminal_frame=grab)
        del post, feat
        print(f"[wm] {k}: T={T} terminal={rec['terminal_index']} "
              f"r_pred@term={rec.get('reward_pred_at_terminal', float('nan')):.2f} "
              f"v@term={rec.get('value_at_terminal', float('nan')):.2f} "
              f"({rec['seconds']}s)", flush=True)

    return (stats.per_ep,
            stats.reward_section(reward_scale, args.fp_threshold),
            stats.value_section(args, action_repeat),
            imag_pool)


@torch.no_grad()
def analyse_imagination(agent, cfg, imag_pool, args):
    """Roll the ACTOR + model forward from real posterior latents, through
    ImagBehavior._imagine and ._compute_target -- the same calls Dreamer._train makes,
    with the same reward objective lambda (heads['reward'](get_feat(s)).mode())."""
    wm = agent._wm
    beh = agent._task_behaviors[args.task_name]
    horizon = int(cfg["imag_horizon"])
    reward_scale = float(cfg.get("genesis_reward_scale", 1.0))
    thresh = args.imag_reward_threshold if args.imag_reward_threshold is not None \
        else 0.5 * reward_scale
    out = {"imag_horizon": horizon,
           "reward_reached_threshold": float(thresh),
           "threshold_note": ("an imagined rollout 'reaches a predicted reward' when "
                              "max_h reward_head(imagined feat) exceeds this"),
           "buckets": BUCKET_LABELS, "by_bucket": []}
    for bi, label in enumerate(BUCKET_LABELS):
        picks = imag_pool[bi]
        if not picks:
            out["by_bucket"].append({"bucket": label, "n_starts": 0})
            continue
        # API-GUESS: ImagBehavior._imagine flattens the first TWO dims of `start`
        # (`x.reshape([-1] + list(x.shape[2:]))`), so any (B, K, ...) layout works.
        # We use (1, N, ...) -- N independent start latents. Read from models.py.
        start = {k: torch.cat([pk[k] for pk in picks], dim=1) for k in picks[0]}
        feats, states, actions, ents = beh._imagine(start, beh.actor, horizon)
        reward = wm.heads["reward"](wm.dynamics.get_feat(states)).mode()   # (H,N,1)
        target, weights, base = beh._compute_target(feats, states, reward)
        # API-GUESS: _compute_target returns `target` as the LIST that
        # tools.lambda_return produces (torch.unbind over the batch axis); Dreamer's
        # own actor loss re-assembles it with exactly this stack, giving (H-1, N, 1).
        # Traced through static_scan_for_lambda_return, not verified on real numbers.
        tgt = torch.stack(target, dim=1)                                   # (H-1,N,1)
        disc_ret = (weights * reward).sum(0).squeeze(-1)                   # (N,)
        rmax = reward.max(dim=0).values.squeeze(-1)                        # (N,)
        out["by_bucket"].append({
            "bucket": label,
            "n_starts": int(reward.shape[1]),
            "imag_lambda_return_from_start_mean": float(tgt[0].mean().item()),
            "imag_lambda_return_all_steps_mean": float(tgt.mean().item()),
            "imag_discounted_reward_sum_mean": float(disc_ret.mean().item()),
            "imag_reward_max_mean": float(rmax.mean().item()),
            "imag_reward_max_max": float(rmax.max().item()),
            "frac_rollouts_reaching_reward": float((rmax > thresh).float().mean()),
            "value_at_start_mean": float(beh.value(feats[0]).mode().mean().item()),
            "actor_entropy_mean": float(ents.mean().item()),
            "cont_mean": float(wm.heads["cont"](
                wm.dynamics.get_feat(states)).mean.mean().item()),
        })
    return out


def analyse_metrics_jsonl(logdir, pattern, max_points):
    """Item 4. Report the reward-frames-per-batch trajectory. If the metric is not in
    metrics.jsonl, SAY SO -- never substitute a different metric."""
    out = {"pattern": pattern, "present": False, "logdir": str(logdir)}
    if logdir is None:
        out["reason"] = "--logdir not passed and could not be inferred"
        return out
    p = pathlib.Path(logdir).expanduser() / "metrics.jsonl"
    out["metrics_jsonl"] = str(p)
    if not p.is_file():
        out["reason"] = f"{p} does not exist"
        return out
    rows, all_keys = [], set()
    bad = 0
    with p.open() as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except Exception:
                bad += 1
                continue
            all_keys.update(rows[-1].keys())
    out["n_rows"] = len(rows)
    out["n_unparseable_rows"] = bad
    pat = pattern.lower()
    hits = sorted(k for k in all_keys if pat in k.lower())
    out["matching_keys"] = hits
    if not hits:
        out["present"] = False
        out["reason"] = (
            f"NO key in {p} contains {pattern!r}. The reward-frames-per-batch metric "
            f"this run logs (if any) is NOT this one -- reporting nothing rather than "
            f"substituting a different metric.")
        out["reward_like_keys_available"] = sorted(
            k for k in all_keys if "reward" in k.lower())
        out["all_keys_sample"] = sorted(all_keys)[:60]
        return out
    out["present"] = True
    series = {}
    for key in hits:
        pts = [(int(r["step"]), float(r[key])) for r in rows
               if key in r and "step" in r and r[key] is not None]
        pts.sort()
        vals = np.asarray([v for _, v in pts], dtype=float)
        stride = max(1, int(np.ceil(len(pts) / max(1, max_points))))
        series[key] = {
            "n_points": len(pts),
            "first": {"step": pts[0][0], "value": pts[0][1]} if pts else None,
            "last": {"step": pts[-1][0], "value": pts[-1][1]} if pts else None,
            "mean": float(vals.mean()) if vals.size else None,
            "min": float(vals.min()) if vals.size else None,
            "max": float(vals.max()) if vals.size else None,
            "trajectory_stride": stride,
            "trajectory": [{"step": s, "value": v} for s, v in pts[::stride]],
        }
    out["series"] = series
    return out


# ===========================================================================
# printing
# ===========================================================================
def print_summary(res):
    def f(x, nd=3):
        return "n/a" if x is None else (f"{x:.{nd}f}" if isinstance(x, float) else str(x))

    m, r, v, im = res["meta"], res["reward_head"], res["value_head"], res["imagination"]
    print("\n" + "=" * 78)
    print(f"dv3 INTERROGATION  {m['checkpoint']}")
    print("=" * 78)
    print(f"configs           : {' '.join(m['configs'])}")
    print(f"task / scope      : {m['task']} / {m.get('genesis_scope')}")
    print(f"action_repeat     : {m['action_repeat']}   (reward scale "
          f"{m['genesis_reward_scale']}, imag_horizon {m['imag_horizon']})")
    print(f"demodir           : {m['demodir']}")
    print(f"episodes          : {m['n_episodes_terminal']} terminal + "
          f"{m['n_episodes_nonterminal']} non-terminal")
    print("-" * 78)
    print("1. REWARD HEAD")
    print(f"   MAE rewarded frames    : {f(r['mae_rewarded_frames'])}   "
          f"(n={r['n_rewarded_frames']})")
    print(f"   MAE unrewarded frames  : {f(r['mae_unrewarded_frames'])}   "
          f"(n={r['n_unrewarded_frames']})")
    print(f"   pred @ TERMINAL frame  : {f(r['pred_at_terminal_mean'], 2)} "
          f"[{f(r['pred_at_terminal_min'], 2)} .. {f(r['pred_at_terminal_max'], 2)}] "
          f"vs true {f(r['true_at_terminal_mean'], 2)}")
    print(f"   false positives        : mean {f(r['false_positive_pred_mean'])} "
          f"max {f(r['false_positive_pred_max'], 2)} "
          f"frac>{f(r['false_positive_threshold'], 2)}: "
          f"{f(r['false_positive_frac_over_threshold'], 4)}")
    print("-" * 78)
    print("2. VALUE HEAD  (value vs decisions-to-terminal)")
    hdr = "   bucket   " + "".join(f"{b:>9}" for b in v["buckets"])
    print(hdr)
    print("   value    " + "".join(
        ("      n/a" if x is None else f"{x:9.2f}")
        for x in v["value_mean_by_dtt_bucket"]))
    print("   frames   " + "".join(f"{n:9d}" for n in v["n_frames_by_dtt_bucket"]))
    print("   ctl(noT) " + "".join(
        ("      n/a" if x is None else f"{x:9.2f}")
        for x in v["nonterminal_control"]["value_mean_by_bucket"]))
    print(f"   value @ terminal       : {f(v['value_at_terminal'], 2)}")
    print(f"   spearman(value, -dtt)  : "
          f"{f(None if v['spearman_value_vs_decisions_to_terminal'] is None else -v['spearman_value_vs_decisions_to_terminal'])}")
    if v.get("reason"):
        print(f"   value_reach            : NOT COMPUTED -- {v['reason']}")
    else:
        print(f"   value_reach (>= {f(v['value_reach_fraction_of_terminal'], 2)} x "
              f"terminal, thresh {f(v['threshold'], 2)}):")
        print(f"       any        : {v['value_reach_decisions']} decisions = "
              f"{v['value_reach_sim_steps']} sim steps")
        print(f"       contiguous : {v['value_reach_contiguous_decisions']} decisions "
              f"= {v['value_reach_contiguous_sim_steps']} sim steps")
    print("-" * 78)
    print(f"3. IMAGINATION  (actor+model, horizon {im['imag_horizon']}, "
          f"reward-reached threshold {f(im['reward_reached_threshold'], 2)})")
    print("   bucket      n   lam-return   disc-rew   max-rew   frac-reach   V(start)")
    for b in im["by_bucket"]:
        if not b["n_starts"]:
            print(f"   {b['bucket']:>7}   0        --         --        --          "
                  f"--           --")
            continue
        print(f"   {b['bucket']:>7} {b['n_starts']:3d} "
              f"{b['imag_lambda_return_from_start_mean']:12.3f} "
              f"{b['imag_discounted_reward_sum_mean']:10.3f} "
              f"{b['imag_reward_max_mean']:9.3f} "
              f"{b['frac_rollouts_reaching_reward']:12.3f} "
              f"{b['value_at_start_mean']:10.3f}")
    print("-" * 78)
    rd = res["replay_reward_density"]
    print("4. REPLAY REWARD DENSITY (metrics.jsonl)")
    if not rd.get("present"):
        print(f"   ABSENT: {rd.get('reason')}")
        if rd.get("reward_like_keys_available"):
            print(f"   reward-ish keys present instead: "
                  f"{rd['reward_like_keys_available']}")
    else:
        for key, s in rd["series"].items():
            print(f"   {key}: n={s['n_points']} first={s['first']['value']:.3f}"
                  f"@{s['first']['step']} last={s['last']['value']:.3f}"
                  f"@{s['last']['step']} mean={s['mean']:.3f} max={s['max']:.3f}")
    print("=" * 78 + "\n")


# ===========================================================================
# main
# ===========================================================================
def build_argparser():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", help="<logdir>/latest.pt or <logdir>/ckpt_<step>.pt")
    ap.add_argument("--configs", nargs="*",
                    help="config overlay names the RUN used, e.g. genesis_pixels "
                         "genesis_pick_msrecipe genesis_final_rr")
    ap.add_argument("--demodir", help="dreamer-format demo dir (with repeat.json)")
    ap.add_argument("--n-episodes", type=int, default=8,
                    help="number of TERMINAL (rewarded) demo episodes to analyse")
    ap.add_argument("--n-nonterminal-episodes", type=int, default=2,
                    help="extra no-terminal (no-pick) episodes used as the value "
                         "negative control; 0 to skip")
    ap.add_argument("--out", help="path to write the JSON dump")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--logdir", default=None,
                    help="run logdir for metrics.jsonl / run_config.json "
                         "(default: the checkpoint's parent directory)")
    ap.add_argument("--dreamer-root", default=DEFAULT_DREAMER_ROOT)
    ap.add_argument("--task-name", default="default",
                    help="key in agent._task_behaviors (multitask runs)")
    ap.add_argument("--chunk-frames", type=int, default=128,
                    help="frames per RSSM.observe chunk (posterior carries across)")
    ap.add_argument("--imag-per-bucket", type=int, default=8,
                    help="imagination start latents sampled per dtt bucket")
    ap.add_argument("--imag-reward-threshold", type=float, default=None,
                    help="an imagined rollout 'reaches reward' above this "
                         "(default: 0.5 x genesis_reward_scale, printed)")
    ap.add_argument("--fp-threshold", type=float, default=None,
                    help="reward-head false-positive threshold on unrewarded frames "
                         "(default: 0.1 x genesis_reward_scale, printed)")
    ap.add_argument("--value-reach-frac", type=float, default=0.10)
    ap.add_argument("--value-curve-max-decisions", type=int, default=400,
                    help="cap on the per-decision value curve written to the JSON")
    ap.add_argument("--reach-window", type=int, default=5)
    ap.add_argument("--reach-min-count", type=int, default=2)
    ap.add_argument("--metric-pattern", default="reward_frames",
                    help="substring searched in metrics.jsonl keys (item 4)")
    ap.add_argument("--metric-max-points", type=int, default=60)
    ap.add_argument("--allow-config-mismatch", action="store_true",
                    help="proceed despite a run_config.json / repeat.json disagreement "
                         "(prints the diff loudly)")
    ap.add_argument("--allow-missing-run-config", action="store_true",
                    help="proceed when the logdir has no run_config.json -- the "
                         "action_repeat/action-semantics assertions are then SKIPPED")
    ap.add_argument("--disable-torch-compile", choices=["auto", "yes", "no"],
                    default="auto", help="auto = yes on cpu (parsed before torch "
                                         "import; see module docstring)")
    ap.add_argument("--self-test", action="store_true",
                    help="build a tiny synthetic Dreamer + demo dir and exercise "
                         "every code path (no real checkpoint needed)")
    ap.add_argument("--self-test-dir", default=None,
                    help="where to build the self-test fixture (default: a tempdir)")
    return ap


def run_interrogation(args, dv3, cfg_overrides=None):
    t_start = time.time()
    tools = dv3["tools"]

    ckpt = pathlib.Path(args.checkpoint).expanduser()
    logdir = pathlib.Path(args.logdir).expanduser() if args.logdir else ckpt.parent
    print(f"[resolve] dreamer-root  : {dv3['root']}")
    print(f"[resolve] checkpoint    : {ckpt}  "
          f"({ckpt.stat().st_size / 1e6:.1f} MB, mtime "
          f"{time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(ckpt.stat().st_mtime))})")
    print(f"[resolve] logdir        : {logdir}")
    print(f"[resolve] configs       : {' '.join(args.configs)}")
    print(f"[resolve] device        : {args.device}")
    print(f"[resolve] torch.compile : disabled={_COMPILE_DISABLE} "
          f"(--disable-torch-compile {_COMPILE_MODE})")

    cfg = build_config(dv3, args.configs, cfg_overrides)
    print(f"[resolve] action_repeat : {cfg['action_repeat']} (from --configs)")

    rc_path = logdir / "run_config.json"
    if not rc_path.exists() and (logdir.parent / "run_config.json").exists():
        rc_path = logdir.parent / "run_config.json"     # genesis_eval.py does the same
    if rc_path.exists():
        rc = check_run_config(cfg, rc_path, args.allow_config_mismatch)
        print(f"[resolve] run_config    : {rc_path} -- "
              f"{'MATCHES' if not rc['mismatches'] else 'MISMATCH (overridden)'}")
    elif args.allow_missing_run_config:
        rc = {"path": None, "checked": {}, "mismatches": [],
              "skipped": "no run_config.json; --allow-missing-run-config given"}
        print("[resolve] run_config    : ABSENT -- action-semantics assertions SKIPPED")
    else:
        raise SystemExit(
            f"no run_config.json in {logdir} or {logdir.parent}. That file is the only "
            f"record of the action semantics/stride this checkpoint was trained under, "
            f"and interrogating under the wrong ones measures a different agent. Pass "
            f"--logdir explicitly, or --allow-missing-run-config to proceed blind.")

    demo_stamp = check_demo_stamp(cfg, args.demodir, args.allow_config_mismatch)
    action_repeat = int(cfg["action_repeat"])
    if demo_stamp["repeat_json"]:
        print(f"[resolve] demo stride   : {demo_stamp['repeat_json']['action_repeat']} "
              f"(terminal_reward "
              f"{demo_stamp['repeat_json'].get('terminal_reward')})")
    if rc.get("stamped_demodir") and \
            os.path.basename(str(rc["stamped_demodir"]).rstrip("/")) != \
            os.path.basename(str(args.demodir).rstrip("/")):
        print(f"[WARN] --demodir {args.demodir} is NOT the dir this run trained on "
              f"({rc['stamped_demodir']}). That is legitimate for a probe, but the "
              f"reward/value numbers are then OFF-TRAINING-DISTRIBUTION.", flush=True)

    obs_space, act_space = build_spaces(cfg)
    print(f"[resolve] obs space     : "
          f"{ {k: tuple(s.shape) for k, s in obs_space.spaces.items()} }")
    print(f"[resolve] act space     : {tuple(act_space.shape)}")

    blob, kind, top_keys = load_checkpoint_blob(ckpt, args.device)
    print(f"[resolve] ckpt format   : {kind} (top-level keys {top_keys})")
    agent, config, tb_dir = build_agent(dv3, cfg, obs_space, act_space, args.device,
                                        args.task_name)
    load_report = load_into_agent(agent, blob, kind, args.task_name)

    all_eps = tools.load_episodes(pathlib.Path(args.demodir).expanduser(),
                                  limit=None,
                                  use_depth=bool(cfg.get("use_depth", False)))
    if not all_eps:
        raise SystemExit(f"--demodir {args.demodir} contains no loadable *.npz episodes")
    keys, census = select_episodes(all_eps, args.n_episodes,
                                   args.n_nonterminal_episodes)
    print(f"[resolve] demo episodes : {len(all_eps)} available; using "
          f"{len(census['selected_terminal'])} terminal + "
          f"{len(census['selected_nonterminal'])} non-terminal")
    shape_report = cross_check_shapes(agent, obs_space, act_space,
                                      {k: all_eps[k] for k in keys}, args.task_name)
    print(f"[resolve] shape checks  : {shape_report}")

    per_ep, reward_section, value_section, imag_pool = analyse_episodes(
        agent, cfg, all_eps, keys, args.device, args.chunk_frames, args, action_repeat)
    imag_section = analyse_imagination(agent, cfg, imag_pool, args)
    metrics_section = analyse_metrics_jsonl(logdir, args.metric_pattern,
                                            args.metric_max_points)

    res = {
        "meta": {
            "tool": "analysis/dv3_interrogate.py",
            "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "dreamer_root": dv3["root"],
            "checkpoint": str(ckpt),
            "checkpoint_bytes": ckpt.stat().st_size,
            "checkpoint_mtime": ckpt.stat().st_mtime,
            "logdir": str(logdir),
            "configs": list(args.configs),
            "device": args.device,
            "task": cfg["task"],
            "genesis_scope": cfg.get("genesis_scope"),
            "genesis_joint_action_mode": cfg.get("genesis_joint_action_mode"),
            "genesis_cartesian_control": cfg.get("genesis_cartesian_control"),
            "genesis_reward_scale": cfg.get("genesis_reward_scale", 1.0),
            "action_repeat": action_repeat,
            "imag_horizon": cfg["imag_horizon"],
            "discount": cfg["discount"],
            "demodir": str(args.demodir),
            "n_episodes_terminal": len(census["selected_terminal"]),
            "n_episodes_nonterminal": len(census["selected_nonterminal"]),
            "torch_compile_disabled": bool(_COMPILE_DISABLE),
            "elapsed_seconds": None,
        },
        "run_config_check": rc,
        "demo_stamp": demo_stamp,
        "checkpoint_load": load_report,
        "shape_cross_check": shape_report,
        "episode_census": census,
        "per_episode": per_ep,
        "reward_head": reward_section,
        "value_head": value_section,
        "imagination": imag_section,
        "replay_reward_density": metrics_section,
    }
    res["meta"]["elapsed_seconds"] = round(time.time() - t_start, 1)
    shutil.rmtree(tb_dir, ignore_errors=True)
    print_summary(res)
    if args.out:
        out = pathlib.Path(args.out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(res, indent=1, default=str))
        print(f"wrote {out}", flush=True)
    return res


# ===========================================================================
# SELF-TEST
# ===========================================================================
SELF_TEST_SHRINK = {
    # Tiny model, REAL genesis obs/action shapes. Everything that changes a code PATH
    # (dyn_discrete, symlog_disc heads, BlockGRUCell via dyn_gru_blocks, cnn encoder,
    # reward_EMA, slow critic target) is left at the genesis values.
    "dyn_hidden": 32, "dyn_deter": 32, "dyn_stoch": 4, "dyn_discrete": 4,
    "dyn_gru_blocks": 8, "units": 32, "imag_horizon": 5,
    "precision": 32,                 # amp autocast is cuda-only
    "encoder": {"cnn_depth": 4, "mlp_units": 32, "mlp_layers": 1},
    "decoder": {"cnn_depth": 4, "mlp_units": 32, "mlp_layers": 1},
    "actor": {"layers": 1}, "critic": {"layers": 1},
    "reward_head": {"layers": 1}, "cont_head": {"layers": 1},
    "video_pred_log": False, "colorful_dimension": False, "debug": False,
}
SELF_TEST_CONFIGS = ["genesis_pixels", "genesis_pick_msrecipe"]


def _write_synth_demo(dst, uid, T, adim, img_shape, terminal, terminal_reward, rng):
    img = rng.integers(0, 256, size=(T,) + tuple(img_shape), dtype=np.uint8)
    act = rng.uniform(-1, 1, size=(T, adim)).astype(np.float32)
    act[0] = 0.0                                   # dreamer convention: index 0 is reset
    rew = np.zeros(T, np.float32)
    is_term = np.zeros(T, bool)
    if terminal:
        rew[-1] = float(terminal_reward)
        is_term[-1] = True
    is_first = np.zeros(T, bool); is_first[0] = True
    is_last = np.zeros(T, bool); is_last[-1] = True
    np.savez_compressed(
        dst / f"genesis-{uid:04d}-{T}.npz",
        image=img, action=act, reward=rew,
        discount=(1.0 - is_term.astype(np.float32)),
        is_first=is_first, is_last=is_last, is_terminal=is_term,
        logprob=np.zeros(T, np.float32))


def self_test(args, dv3):
    print("\n" + "#" * 78)
    print("# SELF-TEST: tiny Dreamer with REAL genesis shapes, synthetic tapes")
    print("#" * 78 + "\n", flush=True)
    root = pathlib.Path(args.self_test_dir).expanduser() if args.self_test_dir \
        else pathlib.Path(tempfile.mkdtemp(prefix="dv3_interrogate_selftest_"))
    root.mkdir(parents=True, exist_ok=True)
    logdir = root / "run"
    demodir = root / "demos"
    logdir.mkdir(exist_ok=True)
    demodir.mkdir(exist_ok=True)
    rng = np.random.default_rng(0)

    cfg = build_config(dv3, SELF_TEST_CONFIGS, SELF_TEST_SHRINK)
    ar = int(cfg["action_repeat"])
    scale = float(cfg["genesis_reward_scale"])
    obs_space, act_space = build_spaces(cfg)
    adim = int(act_space.shape[0])
    img_shape = tuple(obs_space["image"].shape)
    print(f"[self-test] action_repeat={ar} reward_scale={scale} adim={adim} "
          f"image={img_shape} imag_horizon={cfg['imag_horizon']}")

    # --- synthetic demo dir in convert_genesis_demos_repeat.py's exact format ------
    for i, T in enumerate((37, 45, 29)):
        _write_synth_demo(demodir, 100 + i, T, adim, img_shape, True, scale, rng)
    for i, T in enumerate((33, 41)):
        _write_synth_demo(demodir, 200 + i, T, adim, img_shape, False, scale, rng)
    (demodir / "repeat.json").write_text(json.dumps(dict(
        action_repeat=ar, action_encoding="delta_joint",
        terminal_reward=scale, scope=cfg.get("genesis_scope"),
        generator="dv3_interrogate.py --self-test", n_pick=3, n_nopick=2), indent=1))

    # --- run_config.json exactly as dreamer.main() dumps it (post-division keys) ---
    rc = {k: cfg[k] for k in RUN_CONFIG_KEYS if k in cfg}
    for k in RUN_CONFIG_DIVIDED_KEYS:
        if k in cfg and isinstance(cfg[k], (int, float)):
            rc[k] = int(cfg[k]) // ar
    rc["demodir"] = str(demodir)
    (logdir / "run_config.json").write_text(json.dumps(rc, indent=1, default=str))

    # --- metrics.jsonl: one WITH the metric, one WITHOUT (both branches of item 4) -
    with (logdir / "metrics.jsonl").open("w") as f:
        for s in range(0, 100001, 10000):
            f.write(json.dumps({"step": s, "train_return": 0.01 * s / 1e4,
                                "train/data/reward_frames": 2.0 + s / 1e5,
                                "reward_loss": 1.0}) + "\n")
    nometric = root / "run_nometric"
    nometric.mkdir(exist_ok=True)
    with (nometric / "metrics.jsonl").open("w") as f:
        for s in range(0, 30001, 10000):
            f.write(json.dumps({"step": s, "train_return": 0.0,
                                "reward_loss": 1.0}) + "\n")

    # --- checkpoints in dreamer.py's exact save formats --------------------------
    agent, _config, tb = build_agent(dv3, cfg, obs_space, act_space, args.device,
                                     "default")
    tools = dv3["tools"]
    torch.save({"agent_state_dict": agent.state_dict(),
                "optims_state_dict": tools.recursively_collect_optim_state_dict(agent)},
               logdir / "latest.pt")
    shutil.copy(logdir / "latest.pt", logdir / "ckpt_120000.pt")   # sbatch archive form
    torch.save({"wm_state_dict": agent._wm.state_dict(),
                "wm_optims": tools.recursively_collect_optim_state_dict(agent._wm)},
               root / "cache_wm.pt")
    shutil.rmtree(tb, ignore_errors=True)
    del agent

    checks = []

    def check(name, fn):
        try:
            fn()
            checks.append((name, "PASS", ""))
            print(f"  [PASS] {name}", flush=True)
        except Exception as e:                                     # noqa: BLE001
            checks.append((name, "FAIL", f"{type(e).__name__}: {e}"))
            print(f"  [FAIL] {name}: {type(e).__name__}: {e}", flush=True)
            raise

    def base_args(**kw):
        a = argparse.Namespace(**vars(args))
        a.checkpoint = str(logdir / "latest.pt")
        a.configs = SELF_TEST_CONFIGS
        a.demodir = str(demodir)
        a.logdir = str(logdir)
        a.n_episodes = 3
        a.n_nonterminal_episodes = 2
        a.imag_per_bucket = 2
        a.chunk_frames = 16                    # forces the multi-chunk carry path
        a.out = str(root / "out.json")
        a.self_test = False
        for k, v in kw.items():
            setattr(a, k, v)
        return a

    print("\n-- T1: full run on latest.pt (chunked observe, all four sections) --")
    res = {}

    def t1():
        res["r"] = run_interrogation(base_args(), dv3, cfg_overrides=SELF_TEST_SHRINK)
        r = res["r"]
        assert r["reward_head"]["n_rewarded_frames"] == 3, \
            r["reward_head"]["n_rewarded_frames"]
        assert r["reward_head"]["pred_at_terminal_mean"] is not None
        assert r["reward_head"]["mae_unrewarded_frames"] is not None
        assert r["reward_head"]["false_positive_pred_max"] is not None
        v = r["value_head"]
        assert len(v["value_mean_by_dtt_bucket"]) == len(DTT_BUCKETS)
        assert v["n_frames_by_dtt_bucket"][0] == 3, v["n_frames_by_dtt_bucket"]
        assert sum(v["nonterminal_control"]["n_frames_by_bucket"]) == 33 + 41
        assert v["action_repeat"] == ar
        if v["value_reach_decisions"] is not None:
            assert v["value_reach_sim_steps"] == v["value_reach_decisions"] * ar
        assert v["spearman_value_vs_decisions_to_terminal"] is not None
        im = r["imagination"]
        assert im["imag_horizon"] == cfg["imag_horizon"]
        got = [b for b in im["by_bucket"] if b["n_starts"]]
        assert len(got) >= 5, [b["bucket"] for b in got]
        for b in got:
            assert 0.0 <= b["frac_rollouts_reaching_reward"] <= 1.0
            assert np.isfinite(b["imag_lambda_return_from_start_mean"])
            assert np.isfinite(b["imag_discounted_reward_sum_mean"])
        rd = r["replay_reward_density"]
        assert rd["present"] and "train/data/reward_frames" in rd["series"], rd
        s = rd["series"]["train/data/reward_frames"]
        assert s["n_points"] == 11 and s["first"]["step"] == 0, s
        assert json.loads(pathlib.Path(res["r"]["meta"]["checkpoint"]).parent
                          .joinpath("run_config.json").read_text())
        assert pathlib.Path(root / "out.json").is_file()
        json.loads(pathlib.Path(root / "out.json").read_text())   # dump is valid JSON
    check("T1 full run on latest.pt", t1)

    print("\n-- T2: archived ckpt_<step>.pt loads identically --")

    def t2():
        r2 = run_interrogation(base_args(checkpoint=str(logdir / "ckpt_120000.pt"),
                                         out=str(root / "out2.json")),
                               dv3, cfg_overrides=SELF_TEST_SHRINK)
        assert r2["checkpoint_load"]["checkpoint_kind"] == "agent"
        assert r2["checkpoint_load"]["missing_keys"] == 0, \
            r2["checkpoint_load"]["missing_keys_sample"]
        assert r2["checkpoint_load"]["unexpected_keys"] == 0, \
            r2["checkpoint_load"]["unexpected_keys_sample"]
        a = np.asarray(res["r"]["value_head"]["value_mean_by_dtt_bucket"], float)
        b = np.asarray(r2["value_head"]["value_mean_by_dtt_bucket"], float)
        assert np.allclose(a, b, atol=1e-4, equal_nan=True), (a, b)
    check("T2 ckpt_<step>.pt == latest.pt", t2)

    print("\n-- T3: metrics.jsonl WITHOUT the metric reports absence, not a stand-in --")

    def t3():
        m = analyse_metrics_jsonl(nometric, "reward_frames", 60)
        assert m["present"] is False
        assert "NO key" in m["reason"], m
        assert m["matching_keys"] == []
        assert "reward_loss" in m["reward_like_keys_available"], m
        m2 = analyse_metrics_jsonl(root / "does_not_exist", "reward_frames", 60)
        assert m2["present"] is False and "does not exist" in m2["reason"]
        m3 = analyse_metrics_jsonl(None, "reward_frames", 60)
        assert m3["present"] is False
    check("T3 metric-absent path", t3)

    print("\n-- T4: action_repeat mismatch in run_config.json REFUSES --")

    def t4():
        bad = root / "run_badrepeat"
        bad.mkdir(exist_ok=True)
        rcb = dict(rc); rcb["action_repeat"] = ar + 1
        (bad / "run_config.json").write_text(json.dumps(rcb, indent=1, default=str))
        shutil.copy(logdir / "latest.pt", bad / "latest.pt")
        try:
            run_interrogation(base_args(checkpoint=str(bad / "latest.pt"),
                                        logdir=str(bad), out=None),
                              dv3, cfg_overrides=SELF_TEST_SHRINK)
        except SystemExit as e:
            assert "CONFIG MISMATCH" in str(e) and "action_repeat" in str(e), str(e)
            return
        raise AssertionError("mismatched action_repeat did NOT refuse")
    check("T4 config mismatch refuses", t4)

    print("\n-- T5: --allow-config-mismatch overrides T4 --")

    def t5():
        bad = root / "run_badrepeat"
        r5 = run_interrogation(base_args(checkpoint=str(bad / "latest.pt"),
                                         logdir=str(bad), out=None,
                                         allow_config_mismatch=True),
                               dv3, cfg_overrides=SELF_TEST_SHRINK)
        assert r5["run_config_check"]["overridden"] is True
    check("T5 mismatch override", t5)

    print("\n-- T6: missing run_config.json refuses unless allowed --")

    def t6():
        nore = root / "run_norc"
        nore.mkdir(exist_ok=True)
        shutil.copy(logdir / "latest.pt", nore / "latest.pt")
        try:
            run_interrogation(base_args(checkpoint=str(nore / "latest.pt"),
                                        logdir=str(nore), out=None),
                              dv3, cfg_overrides=SELF_TEST_SHRINK)
        except SystemExit as e:
            assert "run_config.json" in str(e), str(e)
        else:
            raise AssertionError("missing run_config did NOT refuse")
        r6 = run_interrogation(base_args(checkpoint=str(nore / "latest.pt"),
                                         logdir=str(nore), out=None,
                                         allow_missing_run_config=True),
                               dv3, cfg_overrides=SELF_TEST_SHRINK)
        assert r6["run_config_check"].get("skipped")
    check("T6 missing run_config", t6)

    print("\n-- T7: demo stride mismatch (repeat.json) REFUSES --")

    def t7():
        d2 = root / "demos_wrongstride"
        shutil.copytree(demodir, d2, dirs_exist_ok=True)
        meta = json.loads((d2 / "repeat.json").read_text())
        meta["action_repeat"] = ar + 3
        (d2 / "repeat.json").write_text(json.dumps(meta, indent=1))
        try:
            run_interrogation(base_args(demodir=str(d2), out=None), dv3,
                              cfg_overrides=SELF_TEST_SHRINK)
        except SystemExit as e:
            assert "demo stride mismatch" in str(e), str(e)
            return
        raise AssertionError("demo stride mismatch did NOT refuse")
    check("T7 demo stride refuses", t7)

    print("\n-- T8: demo terminal-reward scale mismatch REFUSES --")

    def t8():
        d3 = root / "demos_wrongscale"
        shutil.copytree(demodir, d3, dirs_exist_ok=True)
        meta = json.loads((d3 / "repeat.json").read_text())
        meta["terminal_reward"] = scale * 2
        (d3 / "repeat.json").write_text(json.dumps(meta, indent=1))
        try:
            run_interrogation(base_args(demodir=str(d3), out=None), dv3,
                              cfg_overrides=SELF_TEST_SHRINK)
        except SystemExit as e:
            assert "terminal reward" in str(e), str(e)
            return
        raise AssertionError("terminal-reward mismatch did NOT refuse")
    check("T8 terminal-reward refuses", t8)

    print("\n-- T9: pretrain-cache (wm-only) checkpoint format is handled --")

    def t9():
        r9 = run_interrogation(base_args(checkpoint=str(root / "cache_wm.pt"),
                                         out=None), dv3,
                               cfg_overrides=SELF_TEST_SHRINK)
        assert r9["checkpoint_load"]["checkpoint_kind"] == "pretrain_cache_wm"
        assert "RANDOMLY INITIALIZED" in r9["checkpoint_load"]["warning"]
    check("T9 pretrain-cache format", t9)

    print("\n-- T10: wrong-shaped demos are caught, not silently analysed --")

    def t10():
        d4 = root / "demos_wrongadim"
        d4.mkdir(exist_ok=True)
        shutil.copy(demodir / "repeat.json", d4 / "repeat.json")
        _write_synth_demo(d4, 900, 30, adim + 1, img_shape, True, scale, rng)
        try:
            run_interrogation(base_args(demodir=str(d4), out=None), dv3,
                              cfg_overrides=SELF_TEST_SHRINK)
        except AssertionError as e:
            assert "action dims" in str(e), str(e)
            return
        raise AssertionError("wrong action dim was NOT caught")
    check("T10 shape cross-check", t10)

    print("\n-- T11: unknown --configs name lists what IS available --")

    def t11():
        try:
            build_config(dv3, ["genesis_pixels", "no_such_config_block"])
        except SystemExit as e:
            assert "unknown --configs" in str(e) and "Available" in str(e)
            return
        raise AssertionError("unknown config name was NOT caught")
    check("T11 unknown config name", t11)

    print("\n-- T12: value_reach maths (synthetic curve, not the model) --")

    def t12():
        s = {d: (100.0 if d <= 20 else 1.0) for d in range(0, 60)}
        n = {d: 5 for d in range(0, 60)}
        out = value_reach(s, n, 100.0, 0.10, 1, 2)
        assert out["value_reach_contiguous_decisions"] == 20, out
        assert out["value_reach_decisions"] == 20, out
        assert out["threshold"] == 10.0
        flat = value_reach({d: 0.0 for d in range(5)}, {d: 5 for d in range(5)},
                           0.0, 0.1, 1, 2)
        assert flat["value_reach_decisions"] is None and flat["reason"], flat
        assert abs(spearman([1, 2, 3, 4], [1, 2, 3, 4]) - 1.0) < 1e-9
        assert abs(spearman([1, 2, 3, 4], [4, 3, 2, 1]) + 1.0) < 1e-9
    check("T12 value_reach maths", t12)

    print("\n-- T13: no-terminal-only demo dir refuses (cannot measure dtt) --")

    def t13():
        d5 = root / "demos_noterm"
        d5.mkdir(exist_ok=True)
        shutil.copy(demodir / "repeat.json", d5 / "repeat.json")
        _write_synth_demo(d5, 800, 25, adim, img_shape, False, scale, rng)
        try:
            run_interrogation(base_args(demodir=str(d5), out=None), dv3,
                              cfg_overrides=SELF_TEST_SHRINK)
        except SystemExit as e:
            assert "is_terminal=True" in str(e), str(e)
            return
        raise AssertionError("terminal-free demo dir was NOT caught")
    check("T13 terminal-free demodir", t13)

    print("\n-- T14: HEALTHY synthetic head outputs through the real aggregation --")

    def t14():
        # The tiny untrained model can only exercise the flat/degenerate branch, so
        # drive the PRODUCTION aggregation (FrameStats) with a profile that looks like
        # a critic that learned: value rises linearly over the last 40 decisions,
        # reward head reads 96 on the terminal against a true 100 (the shape of this
        # project's prior finding). Every number below is hand-computable.
        st = FrameStats()
        for i, T in enumerate((120, 140, 100, 160)):
            term = T - 1
            d = np.arange(T)[::-1].astype(float)          # decisions-to-terminal
            v = np.where(d <= 40, 100.0 * (1.0 - d / 40.0), 0.0) + 0.05
            r_pred = np.full(T, 0.3); r_pred[term] = 96.0
            r_true = np.zeros(T); r_true[term] = 100.0
            is_term = np.zeros(T, bool); is_term[term] = True
            st.add_episode(f"synthetic-{i}", r_pred, v, np.full(T, 0.99), r_true,
                           is_term)
        st.add_episode("synthetic-noterm", np.full(80, 0.3), np.full(80, 0.05),
                       np.full(80, 0.99), np.zeros(80), np.zeros(80, bool))
        a = base_args()
        rs = st.reward_section(100.0, None)
        vs = st.value_section(a, ar)
        assert abs(rs["pred_at_terminal_mean"] - 96.0) < 1e-9, rs
        assert abs(rs["true_at_terminal_mean"] - 100.0) < 1e-9, rs
        assert abs(rs["mae_rewarded_frames"] - 4.0) < 1e-9, rs
        assert abs(rs["mae_unrewarded_frames"] - 0.3) < 1e-9, rs
        assert rs["n_rewarded_frames"] == 4 and rs["n_unrewarded_frames"] == 596, rs
        assert abs(rs["false_positive_pred_max"] - 0.3) < 1e-9, rs
        assert abs(rs["false_positive_threshold"] - 10.0) < 1e-9, rs
        assert rs["false_positive_frac_over_threshold"] == 0.0, rs
        assert abs(vs["value_at_terminal"] - 100.05) < 1e-9, vs
        assert abs(vs["threshold"] - 10.005) < 1e-9, vs
        # v(d) crosses 0.1 x terminal at d=36 once the window-5 smoother is applied
        assert vs["value_reach_decisions"] == 36, vs
        assert vs["value_reach_contiguous_decisions"] == 36, vs
        assert vs["value_reach_sim_steps"] == 36 * ar, vs
        assert vs["value_reach_contiguous_sim_steps"] == 36 * ar, vs
        assert vs["spearman_value_vs_decisions_to_terminal"] < -0.5, vs
        # bucket 0 is the terminal frame itself; the 101+ bucket is deep past the rise
        assert abs(vs["value_mean_by_dtt_bucket"][0] - 100.05) < 1e-9, vs
        assert abs(vs["value_mean_by_dtt_bucket"][-1] - 0.05) < 1e-9, vs
        assert vs["n_frames_by_dtt_bucket"][0] == 4, vs
        assert sum(vs["nonterminal_control"]["n_frames_by_bucket"]) == 80, vs
        # and the printer must not crash on a fully-populated healthy report
        print_summary({"meta": {"checkpoint": "synthetic", "configs": ["synthetic"],
                                "task": "genesis_pickplace", "genesis_scope": "pick",
                                "action_repeat": ar, "genesis_reward_scale": 100.0,
                                "imag_horizon": 5, "demodir": "synthetic",
                                "n_episodes_terminal": 4,
                                "n_episodes_nonterminal": 1},
                       "reward_head": rs, "value_head": vs,
                       "imagination": {"imag_horizon": 5,
                                       "reward_reached_threshold": 50.0,
                                       "buckets": BUCKET_LABELS,
                                       "by_bucket": [{"bucket": b, "n_starts": 0}
                                                     for b in BUCKET_LABELS]},
                       "replay_reward_density": {"present": False,
                                                 "reason": "synthetic"}})
    check("T14 healthy-profile aggregation", t14)

    print("\n" + "#" * 78)
    npass = sum(1 for _, s, _ in checks if s == "PASS")
    print(f"# SELF-TEST: {npass}/{len(checks)} checks passed")
    print(f"# fixture: {root}")
    print("#" * 78 + "\n")
    return 0 if npass == len(checks) else 1


def main():
    args = build_argparser().parse_args()
    dv3 = import_dreamer(args.dreamer_root)
    if args.self_test:
        return self_test(args, dv3)
    missing = [n for n in ("checkpoint", "configs", "demodir", "out")
               if not getattr(args, n)]
    if missing:
        raise SystemExit(f"missing required argument(s): "
                         f"{' '.join('--' + m for m in missing)} "
                         f"(or pass --self-test)")
    run_interrogation(args, dv3)
    return 0


if __name__ == "__main__":
    sys.exit(main())
