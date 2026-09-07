"""Amendment A4 (plan §A4, 2026-09-07): re-execution PAIRS for the action-process control.

For a source arm (MG200s or MH200) build TWO arms with one pipeline: a CONTROL (the tape's original actions) and a
TREATMENT (modified actions), each replayed open-loop in the installed env from reset_to({model xml, states[0]}) of
the original tape for the tape's own length T, recording fresh observations every step; a tape is kept iff
is_success() fires within T; rows are cut at that first success (+1 once, terminal). Both arms are then restricted to
the tapes that succeed in BOTH re-executions (identical tape sets). Treatments:
  ema       arm dims a'_t = beta*a'_{t-1} + (1-beta)*a_t (a'_0 = a_0), ONE beta for the whole arm bisected so the arm's
            mean |a'_t - a'_{t-1}| (within tapes, arm dims) equals --target-roughness; gripper binarised at 0 (+1 if g > 0).
  ema_mag   ema, then each tape's arm dims rescaled so its mean |a'| equals the tape's original mean |a| (clipped)  [R1m]
  rough     a'_t = clip(a_t + U(-eps, eps) per arm dim); eps bisected so mean |a'_t - a'_{t-1}| equals --target-roughness;
            gripper unchanged (seeded rng).
Outputs: <arms_root>/<NAME>/{manifest.json, rlpd/transitions.npz, rlpd/manifest.json} for both arms (RLPD format of
convert_arms.py: obs/act/rew/next_obs/done/tape_id), yields + beta/eps + action stats in every manifest.

  $LAB/robo_venv/bin/python baselines/robomimic/build_reexec_arms.py --src MG200s --treat ema --target-roughness 0.0431 --names MG200s_re MG200s_sm
  $LAB/robo_venv/bin/python baselines/robomimic/build_reexec_arms.py --src MH200 --treat rough --target-roughness 0.2741 --names MH200_re MH200_rough
"""
import argparse
import json
import pathlib as pl
import sys
import time

import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import DATA_ROOT, ACT_DIM, make_env, state_from_obs, reset_env_to, sha256_file  # noqa: E402


def load_src(arm, arms_root):
    import h5py
    man = json.loads((arms_root / arm / "manifest.json").read_text())
    tapes = []
    with h5py.File(man["hdf5"], "r") as f:
        for d in man["demos"]:
            g = f["data"][d]
            tapes.append(dict(demo=d, states0=np.asarray(g["states"][0]), model=g.attrs["model_file"],
                              acts=np.asarray(g["actions"], np.float32), T=int(g.attrs["num_samples"]),
                              k_file=int(np.argmax(np.asarray(g["rewards"]) > 0))))
    return man, tapes


def roughness(act_list):
    d = np.concatenate([np.abs(a[1:, :6] - a[:-1, :6]).reshape(-1) for a in act_list if len(a) > 1])
    return float(d.mean())


def mean_abs(act_list):
    return float(np.concatenate([np.abs(a[:, :6]).reshape(-1) for a in act_list]).mean())


def apply_ema(acts, beta, mag_restore=False):
    out = acts.copy(); a = acts[:, :6]; s = np.empty_like(a); s[0] = a[0]
    for t in range(1, len(a)):
        s[t] = beta * s[t - 1] + (1.0 - beta) * a[t]
    if mag_restore:
        m0 = np.abs(a).mean(); m1 = np.abs(s).mean()
        if m1 > 1e-8:
            s = np.clip(s * (m0 / m1), -1.0, 1.0)
    out[:, :6] = s
    out[:, 6] = np.where(acts[:, 6] > 0, 1.0, -1.0)
    return out.astype(np.float32)


def apply_rough(acts, eps, rng):
    out = acts.copy()
    out[:, :6] = np.clip(acts[:, :6] + rng.uniform(-eps, eps, size=acts[:, :6].shape), -1.0, 1.0)
    return out.astype(np.float32)


def bisect(fn, target, lo, hi, iters=40):
    """fn monotone in x on [lo, hi]; returns x with fn(x) ~= target."""
    flo, fhi = fn(lo), fn(hi)
    inc = fhi > flo
    for _ in range(iters):
        mid = 0.5 * (lo + hi); fm = fn(mid)
        if (fm < target) == inc:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def reexecute(env, tape, acts, cap=None):
    s = reset_env_to(env, {"states": tape["states0"], "model": tape["model"]})
    obs, nobs, act = [], [], []
    T = len(acts) if cap is None else min(len(acts), cap)
    for t in range(T):
        o, r, done, info = env.step(acts[t]); s2 = state_from_obs(o)
        obs.append(s); nobs.append(s2); act.append(acts[t]); s = s2
        if bool(env.is_success()["task"]):
            n = t + 1
            rew = np.zeros(n, np.float32); rew[-1] = 1.0; dn = np.zeros(n, np.float32); dn[-1] = 1.0
            return dict(success=True, k=t, obs=np.stack(obs), next_obs=np.stack(nobs), act=np.stack(act), rew=rew, done=dn)
    return dict(success=False, k=None, obs=np.stack(obs), next_obs=np.stack(nobs), act=np.stack(act),
                rew=np.zeros(T, np.float32), done=np.zeros(T, np.float32))


def write_arm(name, src_man, tapes, results, keep, arms_root, extra):
    d = arms_root / name; (d / "rlpd").mkdir(parents=True, exist_ok=True)
    kept = [(t, results[i]) for i, t in enumerate(tapes) if keep[i]]
    obs = np.concatenate([r["obs"] for _, r in kept]); act = np.concatenate([r["act"] for _, r in kept])
    rew = np.concatenate([r["rew"] for _, r in kept]); nobs = np.concatenate([r["next_obs"] for _, r in kept])
    done = np.concatenate([r["done"] for _, r in kept]); tid = np.concatenate([np.full(len(r["rew"]), i, np.int32) for i, (_, r) in enumerate(kept)])
    np.savez_compressed(d / "rlpd" / "transitions.npz", obs=obs, act=act, rew=rew, next_obs=nobs, done=done, tape_id=tid)
    stats = [dict(demo=t["demo"], T=int(t["T"]), success=True, k_first_success=int(r["k"]), rows_kept=int(len(r["rew"])), k_file=int(t["k_file"])) for t, r in kept]
    man = dict(arm=name, src_arm=src_man["arm"], src="reexec_" + src_man["src"], hdf5=src_man["hdf5"], hdf5_sha256=src_man["hdf5_sha256"],
               n_tapes=len(kept), n_success=len(kept), rows_after_cut=int(len(rew)),
               cut_rule="RE-EXECUTED open-loop from reset_to({model, states[0]}) for the tape's own length T; kept iff success within T; rows 0..k of the re-execution, +1 once at k, terminal at k",
               state_keys=src_man["state_keys"], state_dim=23, proprio_dim=9, demos=[t["demo"] for t, _ in kept], stats=stats,
               action_stats=dict(mean_abs_arm=round(mean_abs([r["act"] for _, r in kept]), 4), mean_abs_delta_arm=round(roughness([r["act"] for _, r in kept]), 4),
                                 grip_binary_frac=round(float(np.mean(np.isclose(np.abs(act[:, 6]), 1.0, atol=1e-6))), 4)), **extra)
    (d / "manifest.json").write_text(json.dumps(man, indent=1))
    rm = dict(arm=name, format="rlpd_transitions", n_transitions=int(len(rew)), n_rewarded=int((rew > 0).sum()), n_terminal=int(done.sum()),
              sha256=sha256_file(d / "rlpd" / "transitions.npz"), hdf5=src_man["hdf5"], hdf5_sha256=src_man["hdf5_sha256"], n_tapes=len(kept), rows=int(len(rew)),
              state_keys=src_man["state_keys"], cut_rule=man["cut_rule"], manifest=str(d / "manifest.json"))
    (d / "rlpd" / "manifest.json").write_text(json.dumps(rm, indent=1))
    print(f"[arm] {name}: {len(kept)} tapes, {len(rew)} rows, {int((rew > 0).sum())} rewarded | |a| {man['action_stats']['mean_abs_arm']} |da| {man['action_stats']['mean_abs_delta_arm']} grip_bin {man['action_stats']['grip_binary_frac']} -> {d}", flush=True)
    return man


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, choices=["MG200s", "MH200"])
    ap.add_argument("--treat", required=True, choices=["ema", "ema_mag", "rough"])
    ap.add_argument("--target-roughness", type=float, required=True)
    ap.add_argument("--names", nargs=2, required=True, metavar=("CONTROL", "TREATMENT"))
    ap.add_argument("--arms-root", default=str(DATA_ROOT / "arms"), help="OUTPUT root for the two new arms")
    ap.add_argument("--src-root", default=str(DATA_ROOT / "arms"), help="root holding the SOURCE arm's manifest")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--limit", type=int, default=None, help="first N tapes (smoke)")
    ap.add_argument("--fixed-param", type=float, default=None, help="skip the bisection: use this beta (ema) / eps (rough) directly")
    ap.add_argument("--ladder", default=None, help="comma list of beta/eps values: report control + treatment yields on the first --limit tapes for each, write NO arms")
    args = ap.parse_args()
    root = pl.Path(args.arms_root); t0 = time.time()
    src_man, tapes = load_src(args.src, pl.Path(args.src_root))
    if args.limit:
        tapes = tapes[: args.limit]
    orig = [t["acts"] for t in tapes]
    r_orig, m_orig = roughness(orig), mean_abs(orig)
    print(f"[build] {args.src}: {len(tapes)} tapes, original |a| {m_orig:.4f} |da| {r_orig:.4f}; target |da| {args.target_roughness}", flush=True)
    rng = np.random.default_rng(args.seed)
    if args.ladder:
        # yield ladder (A4 addendum): re-execute the CONTROL once, then each treatment strength on the same tapes; no arms written
        env, _ = make_env(src_man["hdf5"])
        res_c = [reexecute(env, t, orig[i]) for i, t in enumerate(tapes)]; yc = sum(r["success"] for r in res_c)
        print(f"[ladder] control yield {yc}/{len(tapes)}", flush=True)
        base = [rng.uniform(-1.0, 1.0, size=a[:, :6].shape) for a in orig]
        for v in [float(x) for x in args.ladder.split(",")]:
            if args.treat in ("ema", "ema_mag"):
                mod = [apply_ema(a, v, mag_restore=(args.treat == "ema_mag")) for a in orig]
            else:
                mod = [np.concatenate([np.clip(a[:, :6] + v * n, -1, 1), a[:, 6:7]], axis=1).astype(np.float32) for a, n in zip(orig, base)]
            res_t = [reexecute(env, t, mod[i]) for i, t in enumerate(tapes)]; yt = sum(r["success"] for r in res_t)
            both = sum(a["success"] and b["success"] for a, b in zip(res_c, res_t))
            print(f"[ladder] {args.treat} param {v:.4f}: |a| {mean_abs(mod):.4f} |da| {roughness(mod):.4f} -> treatment yield {yt}/{len(tapes)} both {both}/{len(tapes)}", flush=True)
        return
    if args.treat in ("ema", "ema_mag"):
        beta = args.fixed_param if args.fixed_param is not None else bisect(lambda b: roughness([apply_ema(a, b) for a in orig]), args.target_roughness, 0.0, 0.9999)
        mod = [apply_ema(a, beta, mag_restore=(args.treat == "ema_mag")) for a in orig]; param = dict(beta=beta)
    else:
        # fixed noise draw per tape for the bisection (same rng state each evaluation), then the final draw with the found eps
        base = [rng.uniform(-1.0, 1.0, size=a[:, :6].shape) for a in orig]

        def rough_with(eps):
            return [np.concatenate([np.clip(a[:, :6] + eps * n, -1, 1), a[:, 6:7]], axis=1).astype(np.float32) for a, n in zip(orig, base)]
        eps = args.fixed_param if args.fixed_param is not None else bisect(lambda e: roughness(rough_with(e)), args.target_roughness, 0.0, 2.0)
        mod = rough_with(eps); param = dict(eps=eps, noise_seed=args.seed)
    print(f"[build] treatment {args.treat}: {param} -> modified |a| {mean_abs(mod):.4f} |da| {roughness(mod):.4f} grip_bin {np.mean([np.mean(np.isclose(np.abs(a[:, 6]), 1.0)) for a in mod]):.3f}", flush=True)
    env, _ = make_env(src_man["hdf5"])
    res_c, res_t = [], []
    for i, t in enumerate(tapes):
        rc = reexecute(env, t, orig[i]); rt = reexecute(env, t, mod[i]); res_c.append(rc); res_t.append(rt)
        if i % 20 == 0 or i == len(tapes) - 1:
            print(f"[reexec] {i + 1}/{len(tapes)} control succ {sum(r['success'] for r in res_c)} treat succ {sum(r['success'] for r in res_t)} "
                  f"both {sum(a['success'] and b['success'] for a, b in zip(res_c, res_t))} ({time.time() - t0:.0f}s)", flush=True)
    yc = sum(r["success"] for r in res_c); yt = sum(r["success"] for r in res_t)
    keep = [a["success"] and b["success"] for a, b in zip(res_c, res_t)]; nboth = sum(keep)
    print(f"[yield] {args.src}: control {yc}/{len(tapes)} treatment({args.treat}) {yt}/{len(tapes)} BOTH {nboth}/{len(tapes)}", flush=True)
    extra = dict(amendment="A4", treatment=args.treat, treatment_params=param, target_roughness=args.target_roughness, src_action_stats=dict(mean_abs_arm=round(m_orig, 4), mean_abs_delta_arm=round(r_orig, 4)),
                 yield_control=int(yc), yield_treatment=int(yt), yield_both=int(nboth), n_source_tapes=len(tapes), pair=list(args.names),
                 k_shift_control=[(r["k"] - t["k_file"]) for t, r in zip(tapes, res_c) if r["success"]],
                 generator="baselines/robomimic/build_reexec_arms.py", seed=args.seed)
    if nboth == 0:
        sys.exit("FATAL: no tape succeeds in both re-executions")
    write_arm(args.names[0], src_man, tapes, res_c, keep, root, dict(extra, role="control"))
    write_arm(args.names[1], src_man, tapes, res_t, keep, root, dict(extra, role="treatment"))
    print(f"[build] done in {time.time() - t0:.0f}s", flush=True)


if __name__ == "__main__":
    main()
