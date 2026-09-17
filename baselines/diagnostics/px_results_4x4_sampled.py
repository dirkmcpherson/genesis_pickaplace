#!/usr/bin/env python3
"""Per-seed SAMPLED-action statistics for the pixel 4 x 4 (user 2026-09-17: "sampled eval for DV3 and R2, whatever works
best for RLPD").  Reads cells straight from the analysis mirror; writes a per-seed CSV in the schema
px_results_4x4_plot.py reads.

Design seeds and checkpoints are the first-round rule of px_phase_analysis.py (8 per condition; world models = 8 lowest
seeds that trained to 2M, mean of the 1.5M and 2M cells).  Every learner uses the rnd30 SAMPLE cell:
  world models  mean of rnd30_sample `home` at online_1500000 and online_2000000 (both required)
  {RLPD}        rnd30_sample at the 250k-decision checkpoint. SAMPLE was chosen post hoc because it scores higher than
                MODE on the human/machine seeds (machine 38 v 30 `home` of 240; human 0 v 0).
  {Diffusion Policy}  rnd30_sample (its only cell).
A seed without its cell(s) is absent, not zero.
"""
import argparse, csv, json, os
from pathlib import Path

WM_SET = {"human": "dHfull_all_rns10h_img", "machine": "dDPfull_first_rns10h_img"}
WM_SEEDS = {("{DreamerV3 losses}", "human"): range(8, 16), ("{DreamerV3 losses}", "machine"): range(8, 16),
            ("{r2dreamer}", "human"): range(3, 11), ("{r2dreamer}", "machine"): range(3, 11)}
REP = {"{DreamerV3 losses}": "dreamer", "{r2dreamer}": "r2dreamer"}
ROOT = {"planner72": "P72", "r2teacher": "R2T"}
RLPD_TAG = {"human": "dH", "machine": "dDPfirst", "planner72": "dPlanner72", "r2teacher": "dR2fromH_px"}
DP_TAG = {"human": "dH", "machine": "dM"}


def cell(path):
    try:
        m = json.load(open(path))
    except Exception:
        return None
    h = m.get("headline_stages") or {}
    if h.get("home") is None:
        return None
    return h["home"], h.get("picked")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default="~/data/genesis_pickaplace/px_analysis_2026-09-14")
    ap.add_argument("--out", default="paper/figures/px_phase_2026-09-14/px_results_4x4_sampled_per_seed.csv")
    ap.add_argument("--set", default="rnd30", choices=["rnd30", "hold15"],
                    help="hold15 = the 15 demonstration starts (14 are training starts)")
    ap.add_argument("--wm-mode", default="sample", choices=["sample", "mode"],
                    help="world-model cell mode (the milestone sweep scores hold15 in MODE only)")
    a = ap.parse_args()
    M = Path(os.path.expanduser(a.data_root))
    rows = []
    for lab in ("{DreamerV3 losses}", "{r2dreamer}", "{RLPD}", "{Diffusion Policy}"):
        for ds in ("human", "machine", "planner72", "r2teacher"):
            seeds = WM_SEEDS.get((lab, ds), range(8))
            for s in seeds:
                vals, paths = [], []
                if lab in REP:
                    if ds in WM_SET:
                        base = M / "W/ln_milestone_cells" / f"full_r2d_state_{WM_SET[ds]}_{REP[lab]}_s{s}"
                    else:
                        base = M / ROOT[ds] / "evaluation" / f"full_r2d_state_native_rns10h_img_{REP[lab]}_s{s}"
                    for ms in (1_500_000, 2_000_000):
                        p = base / f"online_{ms}" / f"{a.set}_{a.wm_mode}" / "metrics.json"
                        c = cell(p)
                        if c is None:
                            vals = None
                            break
                        vals.append(c); paths.append(str(p))
                elif lab == "{RLPD}":
                    if ds in ("human", "machine"):
                        p = M / "LAB/gp_pxr/e2e_px" / f"e2e_rlpd_px_{RLPD_TAG[ds]}_s{s}" / f"fresh_eval_{a.set}_sample/metrics.json"
                    else:
                        p = M / ROOT[ds] / "runs/rlpd" / f"e2e_rlpd_px_{RLPD_TAG[ds]}_s{s}" / f"fresh_eval_{a.set}_sample/metrics.json"
                    c = cell(p)
                    vals = [c] if c else None; paths = [str(p)]
                else:
                    if ds in DP_TAG:
                        p = M / "LAB/gp_ah/dp_px" / f"ah_dp_px_{DP_TAG[ds]}_s{s}" / f"fresh_eval_{a.set}_sample/metrics.json"
                    else:
                        hits = sorted((M / ROOT[ds] / "runs/dp").glob(f"*_s{s}/fresh_eval_{a.set}_sample/metrics.json"))
                        p = hits[0] if hits else Path("/nonexistent")
                    c = cell(p)
                    vals = [c] if c else None; paths = [str(p)]
                ok = bool(vals)
                home = sum(v[0] for v in vals) / len(vals) if ok else ""
                picks = [v[1] for v in vals] if ok else []
                picked = sum(picks) / len(picks) if ok and all(x is not None for x in picks) else ""
                rows.append(dict(learner=lab, dataset=ds, seed=s, in_design=True, home=home, picked=picked,
                                 cells=";".join(paths) if ok else "", excluded_reason="" if ok else "no sampled cell yet"))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)
    print(f"[csv] {a.out} ({sum(1 for r in rows if r['home'] != '')} seeds with a sampled statistic)")


if __name__ == "__main__":
    main()
