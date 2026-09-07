"""hdf5 + arm manifest -> the three learner formats (paper/ROBOMIMIC_PLAN_2026-09-05.md §3).

One row per decision at 20 Hz (the file's rows ARE decisions). Cut every tape at k = first row with
rewards > 0: rows 0..k, reward +1 exactly once at k, terminal at k, next_obs from the file's next_obs group.
No-success tapes (MG failures, Can-Paired bad half) keep reward 0 and are NOT terminal at their last row
(the value bootstraps). State = the 23-dim study obs (robo_common.STATE_KEYS order); actions verbatim.

Outputs under <data_root>/arms/<ARM>/:
  rlpd/transitions.npz     obs (N,23) act (N,7) rew (N,) next_obs (N,23) done (N,) f32 + tape_id (N,) -- DemoData tuples
  r2d/robosuite-<i>-<T>.npz  to_dreamer_native.py layout: image zeros (T,16,16,3) u8, state (T,23), action backward-
                           shifted (action[0]=0), reward (+1 once), discount, is_first/is_last/is_terminal, logprob;
                           T = rows + 1 (obs before each decision + the final next_obs); repeat.json
                           {action_repeat 1, terminal_reward 1, src_sha, state_dim 23, ...}
  lerobot/                 LeRobotDataset v3 (fps 20): observation.state (9), observation.environment_state (14), action (7)
Each target's manifest.json carries row counts + the source sha.

  $LAB/robo_venv/bin/python baselines/robomimic/convert_arms.py --arm PH200 --targets rlpd,r2d,lerobot
"""
import argparse
import datetime
import hashlib
import json
import os
import pathlib as pl
import sys

import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import HDF5, DATA_ROOT, STATE_KEYS, STATE_DIM, PROPRIO_DIM, ACT_DIM, CONTROL_HZ, state_from_hdf5_group, sha256_file  # noqa: E402

IMG_HW = (16, 16)   # placeholder image for the state-only WM (encoder/decoder cnn_keys '$^'); tiny to keep the CPU replay small
TASK = "pick the can and place it in its target bin"


def load_tapes(hdf5, demos):
    """-> list of dict(obs (T,23), act (T,7), rew (T,), next_obs (T,23), done (T,), success bool, k)."""
    import h5py
    out = []
    with h5py.File(hdf5, "r") as f:
        for d in demos:
            g = f["data"][d]
            act = np.asarray(g["actions"], dtype=np.float32); n = act.shape[0]
            rew_file = np.asarray(g["rewards"], dtype=np.float32).reshape(-1)
            obs = state_from_hdf5_group(g["obs"], n=n); nobs = state_from_hdf5_group(g["next_obs"], n=n)
            succ = bool(rew_file.sum() > 0)
            if succ:
                k = int(np.argmax(rew_file > 0)); obs, nobs, act = obs[: k + 1], nobs[: k + 1], act[: k + 1]
            else:
                k = None
            T = act.shape[0]
            rew = np.zeros(T, np.float32); done = np.zeros(T, np.float32)
            if succ:
                rew[-1] = 1.0; done[-1] = 1.0
            assert np.abs(act).max() <= 1.0 + 1e-6, (d, np.abs(act).max())
            assert np.isfinite(obs).all() and np.isfinite(nobs).all()
            out.append(dict(demo=d, obs=obs, act=act, rew=rew, next_obs=nobs, done=done, success=succ, k=k))
    return out


def write_rlpd(tapes, dst, man):
    dst.mkdir(parents=True, exist_ok=True)
    obs = np.concatenate([t["obs"] for t in tapes]); act = np.concatenate([t["act"] for t in tapes])
    rew = np.concatenate([t["rew"] for t in tapes]); nobs = np.concatenate([t["next_obs"] for t in tapes])
    done = np.concatenate([t["done"] for t in tapes]); tid = np.concatenate([np.full(len(t["rew"]), i, np.int32) for i, t in enumerate(tapes)])
    np.savez_compressed(dst / "transitions.npz", obs=obs, act=act, rew=rew, next_obs=nobs, done=done, tape_id=tid)
    m = dict(man, format="rlpd_transitions", n_transitions=int(len(rew)), n_rewarded=int((rew > 0).sum()), n_terminal=int(done.sum()),
             sha256=sha256_file(dst / "transitions.npz"))
    (dst / "manifest.json").write_text(json.dumps(m, indent=1))
    print(f"[rlpd] {man['arm']}: {len(rew)} transitions, {m['n_rewarded']} rewarded, {m['n_terminal']} terminal -> {dst}")
    return m


def write_r2d(tapes, dst, man, img_hw=IMG_HW):
    dst.mkdir(parents=True, exist_ok=True)
    lens = []; total_r = 0.0
    for i, t in enumerate(tapes):
        n = len(t["rew"]); T = n + 1
        state = np.concatenate([t["obs"], t["next_obs"][-1:]]).astype(np.float32)
        action = np.concatenate([np.zeros((1, ACT_DIM), np.float32), t["act"]])
        reward = np.concatenate([[0.0], t["rew"]]).astype(np.float32)
        is_terminal = np.zeros(T, bool); is_terminal[-1] = bool(t["done"][-1] > 0)
        is_first = np.zeros(T, bool); is_first[0] = True
        is_last = np.zeros(T, bool); is_last[-1] = True
        ep = dict(image=np.zeros((T,) + tuple(img_hw) + (3,), np.uint8), state=state, action=action, reward=reward,
                  discount=(1.0 - is_terminal.astype(np.float32)), is_first=is_first, is_last=is_last, is_terminal=is_terminal,
                  logprob=np.zeros(T, np.float32))
        np.savez_compressed(dst / f"robosuite-{i:06d}-{T}.npz", **ep)
        lens.append(T); total_r += float(reward.sum())
    meta = dict(man, format="r2d_native", action_repeat=1, contract="robomimic_v1", terminal_reward=1.0, with_state=True, state_dim=STATE_DIM,
                state_only=True, image_hw=list(img_hw), image_channels=3, scope="can", n_written=len(tapes), total_reward=total_r,
                decisions_min=int(min(lens)), decisions_median=int(np.median(lens)), decisions_max=int(max(lens)),
                total_rows=int(sum(lens)), src_sha=man["hdf5_sha256"], generator="baselines/robomimic/convert_arms.py",
                created=datetime.datetime.now().isoformat(timespec="seconds"))
    (dst / "repeat.json").write_text(json.dumps(meta, indent=1))
    print(f"[r2d] {man['arm']}: {len(tapes)} tapes, rows {meta['total_rows']}, total reward {total_r:.0f} -> {dst}")
    return meta


def write_lerobot(tapes, dst, man):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    if dst.exists() and any(dst.iterdir()):
        raise SystemExit(f"FATAL: {dst} exists and is not empty (delete it to rebuild)")
    features = {
        "observation.state": {"dtype": "float32", "shape": (PROPRIO_DIM,), "names": None},
        "observation.environment_state": {"dtype": "float32", "shape": (STATE_DIM - PROPRIO_DIM,), "names": None},
        "action": {"dtype": "float32", "shape": (ACT_DIM,), "names": None},
    }
    ds = LeRobotDataset.create(repo_id=f"local/robomimic_can_{man['arm']}", fps=CONTROL_HZ, root=dst, features=features, use_videos=False)
    n_frames = 0
    for t in tapes:
        for i in range(len(t["rew"])):
            ds.add_frame({"observation.state": t["obs"][i][:PROPRIO_DIM], "observation.environment_state": t["obs"][i][PROPRIO_DIM:],
                          "action": t["act"][i], "task": TASK})
            n_frames += 1
        ds.save_episode()
    ds.finalize()
    import pyarrow.parquet as pq, glob
    info = json.loads((dst / "meta" / "info.json").read_text())
    rows = sum(pq.read_table(f).num_rows for f in glob.glob(str(dst / "meta" / "episodes" / "**" / "*.parquet"), recursive=True))
    assert rows == info["total_episodes"] == len(tapes), (rows, info["total_episodes"], len(tapes))
    assert info["total_frames"] == n_frames, (info["total_frames"], n_frames)
    m = dict(man, format="lerobot_v3", fps=CONTROL_HZ, n_episodes=len(tapes), n_frames=n_frames, proprio_dim=PROPRIO_DIM)
    (dst / "robomimic_source.json").write_text(json.dumps(m, indent=1))
    print(f"[lerobot] {man['arm']}: {len(tapes)} episodes, {n_frames} frames (finalized, metadata verified) -> {dst}")
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", required=True)
    ap.add_argument("--targets", default="rlpd,r2d,lerobot")
    ap.add_argument("--arms-root", default=str(DATA_ROOT / "arms"))
    args = ap.parse_args()
    root = pl.Path(args.arms_root) / args.arm
    man = json.loads((root / "manifest.json").read_text())
    tapes = load_tapes(man["hdf5"], man["demos"])
    if "extra" in man:
        tapes += load_tapes(man["extra"]["hdf5"], man["extra"]["demos"])
    rows = sum(len(t["rew"]) for t in tapes); succ = sum(t["success"] for t in tapes)
    assert rows == man["rows_after_cut"], (rows, man["rows_after_cut"])
    print(f"[convert] {args.arm}: {len(tapes)} tapes ({succ} success), {rows} rows after the cut (manifest agrees)")
    base = dict(arm=args.arm, hdf5=man["hdf5"], hdf5_sha256=man["hdf5_sha256"], n_tapes=len(tapes), n_success=int(succ), rows=int(rows),
                state_keys=list(STATE_KEYS), cut_rule=man["cut_rule"], manifest=str(root / "manifest.json"))
    for tgt in args.targets.split(","):
        {"rlpd": write_rlpd, "r2d": write_r2d, "lerobot": write_lerobot}[tgt](tapes, root / tgt, base)


if __name__ == "__main__":
    main()
