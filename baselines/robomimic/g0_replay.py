"""G0 install/compatibility gate (paper/ROBOMIMIC_PLAN_2026-09-05.md §5).

Replay the first N PH tapes' recorded actions open-loop from reset_to(states[0]) (model xml + flattened
state, the robomimic playback convention) in the INSTALLED env and compare with the file:
  * final WORLD can position (Can_pos = object[7:10] of the last row's next_obs; robo_common.CAN_POS) within --tol m (registered: 0.01),
  * the same success flag (file: rewards.sum() > 0; env: is_success()["task"] at any step).
PASS = both hold on >= --need of N tapes (registered: 4/5). Also reports (diagnostic only, not a gate
clause) the per-step single-step restore error: reset_to(states[t]) + step(actions[t]) vs next_obs[t],
which isolates a physics/version mismatch from open-loop chaos.

  $LAB/robo_venv/bin/python baselines/robomimic/g0_replay.py --hdf5 $LAB/robomimic_data/v1.5/can/ph/low_dim_v15.hdf5 --n 5
"""
import argparse
import json
import pathlib as pl
import sys

import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import HDF5, CAN_POS, make_env, reset_env_to, state_from_obs, state_from_hdf5_group, sha256_file  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hdf5", default=str(HDF5["ph"]))
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--need", type=int, default=4)
    ap.add_argument("--tol", type=float, default=0.01, help="final can-position tolerance (m)")
    ap.add_argument("--out", default=None, help="json report path (default <data_root>/g0_report.json)")
    ap.add_argument("--demos", default=None, help="comma list of demo keys to replay instead of the first --n (e.g. successful MG rollouts)")
    args = ap.parse_args()
    import h5py
    import robosuite, robomimic, mujoco
    env, meta = make_env(args.hdf5)
    print(f"[g0] robosuite {robosuite.__version__} robomimic {robomimic.__version__} mujoco {mujoco.__version__} | "
          f"env {meta['env_name']} file env_version {meta.get('env_version')} | sha256 {sha256_file(args.hdf5)[:16]}...", flush=True)
    rows = []
    with h5py.File(args.hdf5, "r") as f:
        demos = args.demos.split(",") if args.demos else sorted(f["data"].keys(), key=lambda s: int(s.split("_")[1]))[: args.n]
        for ep in demos:
            g = f["data"][ep]
            states = np.asarray(g["states"]); acts = np.asarray(g["actions"], dtype=np.float32)
            rew = np.asarray(g["rewards"]).reshape(-1)
            nobs = state_from_hdf5_group(g["next_obs"], n=len(acts))
            obs0 = state_from_hdf5_group(g["obs"], n=len(acts))
            file_success = bool(rew.sum() > 0)
            entry = {"states": states[0], "model": g.attrs["model_file"]}
            # ---- open-loop replay from state 0 ----
            s = reset_env_to(env, entry)
            d0 = float(np.linalg.norm(s[np.r_[0:9, 16:23]] - obs0[0][np.r_[0:9, 16:23]]))   # physical dims (the t=0 can->eef block is an artefact in the files)
            env_success = False; k_env = None; trace = []; eef_states = []
            for t in range(len(acts)):
                o, r, done, info = env.step(acts[t]); s = state_from_obs(o)
                trace.append(float(np.linalg.norm(s[CAN_POS] - nobs[t][CAN_POS])))   # WORLD can xyz = object[7:10] (robo_common layout)
                eef_states.append(s.copy())
                if not env_success and bool(env.is_success()["task"]):
                    env_success = True; k_env = t
            final_err = float(np.linalg.norm(s[CAN_POS] - nobs[-1][CAN_POS]))
            eef_err = float(np.linalg.norm(s[:3] - nobs[-1][:3]))
            k_file = int(np.argmax(rew > 0)) if file_success else None
            # can error at the file's first-success row k (before the post-success free-fall rows; diagnostic)
            can_err_at_k = float(trace[k_file]) if k_file is not None else None
            eef_trace = [float(np.linalg.norm(st_[:3] - nobs[t][:3])) for t, st_ in enumerate(eef_states)]
            # ---- single-step restore check (diagnostic): reset_to(states[t]) + step(actions[t]) vs next_obs[t],
            # POSITIONS ONLY (eef_pos, can xyz, gripper_qpos) -- eef_quat has a sign ambiguity (q == -q) that inflates a
            # full-state norm to ~2.0 without any physical difference (seen on demo_4, 2026-09-06). ----
            step_errs = {"eef_pos": [], "can_pos": [], "grip": []}
            for t in range(0, len(acts), max(1, len(acts) // 24)):
                env.reset_to({"states": states[t]})
                o, r, done, info = env.step(acts[t]); s1 = state_from_obs(o)
                step_errs["eef_pos"].append(float(np.linalg.norm(s1[:3] - nobs[t][:3])))
                step_errs["can_pos"].append(float(np.linalg.norm(s1[CAN_POS] - nobs[t][CAN_POS])))
                step_errs["grip"].append(float(np.linalg.norm(s1[7:9] - nobs[t][7:9])))
            ss = {k: dict(max=float(max(v)), median=float(np.median(v))) for k, v in step_errs.items()}
            ok = (final_err <= args.tol) and (env_success == file_success)
            rows.append(dict(demo=ep, T=int(len(acts)), file_success=file_success, env_success=env_success,
                             k_success_file=k_file, k_success_env=k_env, final_can_err_m=final_err, can_err_at_k_m=can_err_at_k,
                             final_eef_err_m=eef_err, obs0_err=d0, max_can_err_along_replay=float(max(trace)),
                             max_eef_err_along_replay=float(max(eef_trace)), can_err_trace_cm=[round(x * 100, 2) for x in trace],
                             eef_err_trace_cm=[round(x * 100, 2) for x in eef_trace], single_step=ss, pass_=bool(ok)))
            print(f"[g0] {ep}: T={len(acts)} file_success={file_success} (k={k_file}) env_success={env_success} (k={k_env}) "
                  f"final can err {final_err*100:.2f} cm (at k {can_err_at_k*100 if can_err_at_k is not None else float('nan'):.2f} cm; eef final {eef_err*100:.2f} cm; "
                  f"max along replay can {max(trace)*100:.2f} / eef {max(eef_trace)*100:.2f} cm; obs0 err {d0:.1e}) | single-step (pos-only) "
                  f"eef max {ss['eef_pos']['max']*1000:.2f} mm med {ss['eef_pos']['median']*1000:.2f} mm, can max {ss['can_pos']['max']*1000:.2f} mm med {ss['can_pos']['median']*1000:.2f} mm -> {'PASS' if ok else 'FAIL'}", flush=True)
    n_ok = sum(r["pass_"] for r in rows)
    verdict = "PASS" if n_ok >= args.need else "FAIL"
    n_flag = sum(r["env_success"] == r["file_success"] for r in rows); n_k = sum((r["can_err_at_k_m"] or 9) <= args.tol for r in rows)
    print(f"[g0] diagnostics: success flag agrees on {n_flag}/{len(rows)}; can within {args.tol} m AT the first-success row k on {n_k}/{len(rows)}; "
          f"registered clause (final row, all post-success rows replayed): {n_ok}/{len(rows)}", flush=True)
    rep = dict(gate="G0", hdf5=args.hdf5, hdf5_sha256=sha256_file(args.hdf5), n=len(rows), need=args.need, tol_m=args.tol,
               n_pass=n_ok, verdict=verdict, n_success_flag_agree=n_flag, n_can_within_tol_at_k=n_k, robosuite=robosuite.__version__, robomimic=robomimic.__version__,
               mujoco=mujoco.__version__, file_env_version=meta.get("env_version"), rows=rows)
    out = pl.Path(args.out) if args.out else pl.Path(args.hdf5).resolve().parents[3] / "g0_report.json"
    out.write_text(json.dumps(rep, indent=1))
    print(f"G0-RESULT verdict={verdict} pass={n_ok}/{len(rows)} need={args.need} tol={args.tol} -> {out}", flush=True)
    sys.exit(0 if verdict == "PASS" else 3)


if __name__ == "__main__":
    main()
