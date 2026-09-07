"""The shared 50-state evaluation bank (paper/ROBOMIMIC_PLAN_2026-09-05.md §2).

bank_can50.npz: for k in 0..K-1: np.random.seed(1000 + k); env.reset(); entry = env.get_state()
(robosuite's UniformRandomSampler + robot init noise draw from the GLOBAL np.random, so the seed fixes
the placement). Stored: states (K, D), models (K,) xml str, ep_meta (K,) str, can_xy0 (K, 2) = object[0:2]
at t0, seeds. Every evaluator restores entries with reset_to({model, states}) and records the file's sha256.

  $LAB/robo_venv/bin/python baselines/robomimic/make_bank.py --out $LAB/robomimic_data/bank_can50.npz
"""
import argparse
import json
import pathlib as pl
import sys

import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import HDF5, BANK_PATH, CAN_POS, make_env, state_from_obs, sha256_file, reset_env_to, reset_env  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hdf5", default=str(HDF5["ph"]), help="env_meta source (any Can file; identical env_kwargs)")
    ap.add_argument("--k", type=int, default=50)
    ap.add_argument("--seed-base", type=int, default=1000)
    ap.add_argument("--out", default=str(BANK_PATH))
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    out = pl.Path(args.out)
    if out.exists() and not args.force:
        sys.exit(f"FATAL: {out} exists (the bank is generated ONCE; --force to regenerate)")
    env, meta = make_env(args.hdf5)
    states, models, metas, xy, seeds = [], [], [], [], []
    for k in range(args.k):
        s = reset_env(env, seed=args.seed_base + k)
        st = env.get_state()
        states.append(np.asarray(st["states"], dtype=np.float64)); models.append(st["model"]); metas.append(st.get("ep_meta", ""))
        xy.append(s[CAN_POS][:2].copy()); seeds.append(args.seed_base + k)
    states = np.stack(states)
    # round-trip checks on the PHYSICAL dims (eef pose, gripper, world can pose; the can->eef block is an observation
    # artefact at t=0 -- robo_common fact (2)): restore twice == identical; restore == reseeded reset.
    phys = np.r_[0:9, 16:23]
    r1 = reset_env_to(env, {"states": states[0], "model": models[0]}); r2 = reset_env_to(env, {"states": states[0], "model": models[0]})
    s_again = reset_env(env, seed=args.seed_base)
    e_rr = float(np.linalg.norm(r1[phys] - r2[phys])); e_rs = float(np.linalg.norm(r1[phys] - s_again[phys]))
    print(f"[bank] restore-twice error {e_rr:.2e}; restore-vs-reseed (physical dims) {e_rs:.2e}; can->eef block restore-vs-reseed {float(np.linalg.norm(r1[9:16] - s_again[9:16])):.2e} (artefact dims)")
    assert e_rr < 1e-9 and e_rs < 1e-6, (e_rr, e_rs)
    # distinct placements
    xy = np.stack(xy)
    dmin = min(np.linalg.norm(xy[i] - xy[j]) for i in range(len(xy)) for j in range(i + 1, len(xy)))
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, states=states, models=np.array(models, dtype=object), ep_meta=np.array(metas, dtype=object),
                        can_xy0=xy, seeds=np.array(seeds), env_meta=json.dumps(meta), source_hdf5=str(args.hdf5))
    sha = sha256_file(out)
    (out.with_suffix(".json")).write_text(json.dumps(dict(path=str(out), sha256=sha, k=args.k, seed_base=args.seed_base,
                                                          state_dim=int(states.shape[1]), can_xy0=xy.tolist(),
                                                          min_pairwise_can_dist=float(dmin), source_hdf5=str(args.hdf5),
                                                          env_name=meta["env_name"], env_version=meta.get("env_version")), indent=1))
    print(f"[bank] wrote {out} K={args.k} state_dim={states.shape[1]} sha256={sha[:16]}... can x [{xy[:,0].min():.3f},{xy[:,0].max():.3f}] "
          f"y [{xy[:,1].min():.3f},{xy[:,1].max():.3f}] min pairwise dist {dmin*100:.2f} cm")


if __name__ == "__main__":
    main()
