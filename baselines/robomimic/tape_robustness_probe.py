"""Is a tape's survival of one perturbed replay a property of THE TAPE or of that particular noise draw?

Motivation (2026-09-08): the A4 control arms `MH200_re15` (95 tapes, 15,702 rows, RLPD 0.080) and `MH200_re20`
(61 tapes, 9,806 rows, ~0.33) share 60 tapes and have indistinguishable action statistics; they differ only in which
tapes survived their pair's treatment replay. If the survivors are simply the generically well-behaved tapes, that is a
robustness-SELECTION effect of the kind `paper/REPLAY_YIELD_2026-09-08.md` documents, and the two findings connect.

Test: replay every source tape (a) natively (unmodified actions) and (b) under K INDEPENDENT uniform-noise draws at the
same epsilon, with seeds DISJOINT from the build's (build used seed 0). A tape's "robustness score" = how many of the K
independent draws it survives. Then compare that score across the groups defined by the BUILD's draw:
  G_re20     tapes in MH200_re20 (survived the build's harsher eps 0.20 draw)
  G_re15only tapes in MH200_re15 but not re20
  G_rest     the remaining source tapes
If G_re20 scores higher on INDEPENDENT draws, survival is a tape property (generic robustness), not draw luck.

  $LAB/robo_venv/bin/python baselines/robomimic/tape_robustness_probe.py --src MH200 --eps 0.15 --draws 2 --seeds 101,202
"""
import argparse, json, pathlib as pl, sys, time
import numpy as np

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import DATA_ROOT, make_env  # noqa: E402
from build_reexec_arms import load_src, reexecute  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="MH200")
    ap.add_argument("--eps", type=float, default=0.15)
    ap.add_argument("--seeds", default="101,202")
    ap.add_argument("--arms-root", default=str(DATA_ROOT / "arms"))
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--out", default=str(DATA_ROOT / "tape_robustness.json"))
    args = ap.parse_args()
    root = pl.Path(args.arms_root)
    src_man, tapes = load_src(args.src, root)
    if args.limit:
        tapes = tapes[: args.limit]
    groups = {}
    for arm in ("MH200_re20", "MH200_re15"):
        p = root / arm / "manifest.json"
        groups[arm] = set(json.loads(p.read_text())["demos"]) if p.exists() else set()
    seeds = [int(s) for s in args.seeds.split(",")]
    env, _ = make_env(src_man["hdf5"])
    rec = {}
    t0 = time.time()
    for i, t in enumerate(tapes):
        native = reexecute(env, t, t["acts"])["success"]
        wins = []
        for sd in seeds:
            rng = np.random.default_rng(sd * 10000 + i)
            a = t["acts"].copy()
            a[:, :6] = np.clip(a[:, :6] + rng.uniform(-args.eps, args.eps, size=a[:, :6].shape), -1, 1)
            wins.append(bool(reexecute(env, t, a.astype(np.float32))["success"]))
        rec[t["demo"]] = dict(native=bool(native), indep=wins, score=int(sum(wins)))
        if i % 25 == 0 or i == len(tapes) - 1:
            print(f"[probe] {i+1}/{len(tapes)} ({time.time()-t0:.0f}s)", flush=True)
    def grp(name, sel):
        s = [rec[d] for d in sel if d in rec]
        if not s:
            return f"{name}: (empty)"
        return (f"{name}: n={len(s)} native {sum(x['native'] for x in s)}/{len(s)} "
                f"| independent eps={args.eps} draws survived {sum(x['score'] for x in s)}/{len(s)*len(seeds)} "
                f"= {sum(x['score'] for x in s)/(len(s)*len(seeds)):.3f} per draw")
    re20, re15 = groups["MH200_re20"], groups["MH200_re15"]
    all_d = set(rec)
    lines = [grp("G_re20    ", re20), grp("G_re15only", re15 - re20), grp("G_rest    ", all_d - re15 - re20)]
    print("\n".join(lines), flush=True)
    json.dump(dict(src=args.src, eps=args.eps, seeds=seeds, per_tape=rec, groups={k: sorted(v) for k, v in groups.items()},
                   summary=lines), open(args.out, "w"), indent=1)
    print(f"[probe] wrote {args.out}", flush=True)


if __name__ == "__main__":
    main()
