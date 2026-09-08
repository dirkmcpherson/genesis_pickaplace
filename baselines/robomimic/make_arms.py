"""Arm manifests + mask copies (paper/ROBOMIMIC_PLAN_2026-09-05.md §3).

Arms (tape lists are FIXED here, seeded, and written to <data_root>/arms/<ARM>/manifest.json):
  PH200   all 200 PH demos
  MH200   33/34 per operator (6 operators, seed 0) = 200 of the 300 MH demos (mask keys per operator in the file)
  MG200s  200 of the 718 successful MG rollouts (reward.sum() > 0), uniform without replacement, seed 0;
          the manifest lists the chosen demo indices and their 300-block histogram (no per-checkpoint key exists)
  MH300   all 300 MH demos                                   (secondary, all-data)
  MGall   all 3,900 MG rollouts incl. failures               (secondary, all-data)
  PH200pb the 200 PH demos + the 100 FAILED Can-Paired tapes (secondary; needs the paired file)

Also writes <src>_masked.hdf5 COPIES of each source file with a `mask/<ARM>` group (robomimic's
train.hdf5_filter_key) so the BC-RNN control trains on exactly the same tapes; the pristine downloads are
never modified (both sha256 recorded).

  $LAB/robo_venv/bin/python baselines/robomimic/make_arms.py
"""
import argparse
import json
import pathlib as pl
import shutil
import sys

import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import HDF5, DATA_ROOT, sha256_file  # noqa: E402

ARMS = ("PH200", "MH200", "MG200s", "MH300", "MGall", "PH200pb", "MG718s", "MH80")


def demo_stats(f, names):
    out = []
    for d in names:
        g = f["data"][d]; rew = np.asarray(g["rewards"]).reshape(-1)
        succ = bool(rew.sum() > 0); k = int(np.argmax(rew > 0)) if succ else None
        out.append(dict(demo=d, T=int(g.attrs["num_samples"]), success=succ, k_first_success=k,
                        rows_kept=(k + 1 if succ else int(g.attrs["num_samples"]))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=str(DATA_ROOT / "arms"))
    ap.add_argument("--no-masked-copies", action="store_true")
    ap.add_argument("--row-target", type=int, default=16501, help="A6 MH80 row target (default = MG200s's row count)")
    ap.add_argument("--only", default=None, help="write ONLY this arm's manifest/mask (others left untouched); e.g. MG718s (amendment A2)")
    args = ap.parse_args()
    import h5py
    out = pl.Path(args.out); out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    shas = {k: (sha256_file(p) if p.exists() else None) for k, p in HDF5.items()}
    arms = {}
    # --- PH200 / PH200pb ---
    with h5py.File(HDF5["ph"], "r") as f:
        ph = sorted(f["data"].keys(), key=lambda s: int(s.split("_")[1]))
        assert len(ph) == 200, len(ph)
        arms["PH200"] = dict(src="ph", demos=ph, stats=demo_stats(f, ph))
    if HDF5["paired"].exists():
        with h5py.File(HDF5["paired"], "r") as f:
            pd_ = sorted(f["data"].keys(), key=lambda s: int(s.split("_")[1]))
            st = demo_stats(f, pd_)
            bad = [s["demo"] for s in st if not s["success"]]
            assert len(bad) == 100, len(bad)
            arms["PH200pb"] = dict(src="ph", demos=ph, stats=arms["PH200"]["stats"],
                                   extra=dict(src="paired", demos=bad, stats=[s for s in st if not s["success"]]),
                                   note="PH200 + the 100 failed Can-Paired tapes (same-operator human failures; reward 0, not terminal)")
    # --- MH200 / MH300 ---
    with h5py.File(HDF5["mh"], "r") as f:
        mh = sorted(f["data"].keys(), key=lambda s: int(s.split("_")[1]))
        assert len(mh) == 300, len(mh)
        ops = {}
        for key in f["mask"].keys():
            if key.startswith("worse_") or key.startswith("okay_") or key.startswith("better_"):
                names = [s.decode() if isinstance(s, bytes) else str(s) for s in np.asarray(f["mask"][key])]
                if len(names) == 50:
                    ops[key] = sorted(names, key=lambda s: int(s.split("_")[1]))
        assert len(ops) == 6, sorted(f["mask"].keys())
        quota = [34, 34, 33, 33, 33, 33]           # 200 over 6 operators
        pick = []
        for (op, names), q in zip(sorted(ops.items()), quota):
            pick += sorted(rng.choice(names, size=q, replace=False).tolist(), key=lambda s: int(s.split("_")[1]))
        pick = sorted(pick, key=lambda s: int(s.split("_")[1]))
        assert len(pick) == 200 and len(set(pick)) == 200
        arms["MH200"] = dict(src="mh", demos=pick, stats=demo_stats(f, pick), per_operator={op: q for (op, _), q in zip(sorted(ops.items()), quota)})
        arms["MH300"] = dict(src="mh", demos=mh, stats=demo_stats(f, mh))
        # amendment A6 (2026-09-08): row-matched human arm. Uniform permutation (its own generator, so MH200/MG200s
        # draws above are byte-identical to the 09-06 build), prefix minimising |cum_rows - target|.
        st200 = {s2["demo"]: s2 for s2 in arms["MH200"]["stats"]}
        order = np.random.default_rng(args.seed).permutation(len(pick))
        rows = np.array([st200[pick[i]]["rows_kept"] for i in order]); cum = np.cumsum(rows)
        k = int(np.argmin(np.abs(cum - args.row_target))) + 1
        keep = sorted([pick[i] for i in order[:k]], key=lambda s2: int(s2.split("_")[1]))
        arms["MH80"] = dict(src="mh", demos=keep, stats=[st200[d] for d in keep],
                            note=f"amendment A6: MH200 row-matched to {args.row_target} rows; uniform seed-{args.seed} permutation, "
                                 f"prefix minimising |cum_rows - target| -> {k} tapes / {int(cum[k-1])} rows (err {int(abs(cum[k-1]-args.row_target))})",
                            row_target=int(args.row_target), rows_selected=int(cum[k - 1]))
    # --- MG200s / MGall ---
    with h5py.File(HDF5["mg"], "r") as f:
        mg = sorted(f["data"].keys(), key=lambda s: int(s.split("_")[1]))
        assert len(mg) == 3900, len(mg)
        st = demo_stats(f, mg)
        succ = [s["demo"] for s in st if s["success"]]
        assert len(succ) == 718, len(succ)
        pick = sorted(rng.choice(succ, size=200, replace=False).tolist(), key=lambda s: int(s.split("_")[1]))
        blocks = np.bincount([(int(d.split("_")[1]) - 1) // 300 for d in pick], minlength=13).tolist()   # MG keys are demo_1..demo_3900 (no demo_0; checked 2026-09-06)
        stmap = {s["demo"]: s for s in st}
        arms["MG200s"] = dict(src="mg", demos=pick, stats=[stmap[d] for d in pick], block300_histogram=blocks,
                              note="200 of the 718 successful rollouts, uniform without replacement; no per-checkpoint key in the file")
        arms["MGall"] = dict(src="mg", demos=mg, stats=st)
        # amendment A2 (2026-09-07): every successful MG rollout (718; MG200s is a subset) -- the RLPD quantity control
        blocks718 = np.bincount([(int(d.split("_")[1]) - 1) // 300 for d in succ], minlength=13).tolist()
        arms["MG718s"] = dict(src="mg", demos=succ, stats=[stmap[d] for d in succ], block300_histogram=blocks718,
                              note="amendment A2 (2026-09-07): every successful MG rollout; MG200s is a subset; quantity control for RLPD")
    # --- write manifests ---
    if args.only:
        arms = {args.only: arms[args.only]}
    for arm, a in arms.items():
        d = out / arm; d.mkdir(exist_ok=True)
        rows = int(sum(s["rows_kept"] for s in a["stats"])) + (int(sum(s["rows_kept"] for s in a["extra"]["stats"])) if "extra" in a else 0)
        man = dict(arm=arm, src=a["src"], hdf5=str(HDF5[a["src"]]), hdf5_sha256=shas[a["src"]], seed=args.seed,
                   n_tapes=len(a["demos"]) + (len(a["extra"]["demos"]) if "extra" in a else 0),
                   n_success=int(sum(s["success"] for s in a["stats"])), rows_after_cut=rows,
                   cut_rule="rows 0..k inclusive, k = first row with rewards > 0; +1 once at k, terminal at k; no-success tapes: reward 0, not terminal",
                   state_keys=["robot0_eef_pos", "robot0_eef_quat", "robot0_gripper_qpos", "object"], state_dim=23, proprio_dim=9,
                   demos=a["demos"], stats=a["stats"], **{k: v for k, v in a.items() if k not in ("src", "demos", "stats")})
        if "extra" in a:
            man["extra"]["hdf5"] = str(HDF5["paired"]); man["extra"]["hdf5_sha256"] = shas["paired"]
        (d / "manifest.json").write_text(json.dumps(man, indent=1))
        lens = [s["rows_kept"] for s in a["stats"]]
        print(f"[arms] {arm}: {man['n_tapes']} tapes ({man['n_success']} success) rows_after_cut {rows} tape len min/med/max {min(lens)}/{int(np.median(lens))}/{max(lens)}"
              + (f" blocks {a['block300_histogram']}" if "block300_histogram" in a else "") + (f" per_op {a['per_operator']}" if "per_operator" in a else ""))
    # --- masked copies for robomimic's own trainer ---
    if not args.no_masked_copies:
        import h5py
        for src in ("ph", "mh", "mg"):
            dst = HDF5[src].with_name(HDF5[src].stem + "_masked.hdf5")
            if not dst.exists():
                shutil.copyfile(HDF5[src], dst)
            with h5py.File(dst, "a") as f:
                if "mask" not in f:
                    f.create_group("mask")
                for arm, a in arms.items():
                    if a["src"] != src or "extra" in a:
                        continue
                    key = f"mask/{arm}"
                    if key in f:
                        del f[key]
                    f.create_dataset(key, data=np.array(a["demos"], dtype="S"))
            print(f"[arms] masked copy {dst} sha256 {sha256_file(dst)[:16]}... masks: {[k for k, v in arms.items() if v['src'] == src and 'extra' not in v]}")
    if args.only and (out / "arms_index.json").exists():
        idx = json.loads((out / "arms_index.json").read_text()); idx["arms"] = sorted(set(idx["arms"]) | set(arms)); (out / "arms_index.json").write_text(json.dumps(idx, indent=1)); print(f"[arms] arms_index.json += {list(arms)}"); return
    (out / "arms_index.json").write_text(json.dumps(dict(seed=args.seed, arms=list(arms), source_sha256=shas,
                                                          masked=({s: str(HDF5[s].with_name(HDF5[s].stem + '_masked.hdf5')) for s in ('ph', 'mh', 'mg')} if not args.no_masked_copies else None)), indent=1))
    print(f"[arms] wrote {out}/arms_index.json")


if __name__ == "__main__":
    main()
