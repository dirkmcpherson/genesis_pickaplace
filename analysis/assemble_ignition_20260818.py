#!/usr/bin/env python
"""Assemble per-seed ignition + performance numbers for the 2026-08-18 figures.

Reads the raw wandb pulls in the scratchpad (pull_rlpd/pull_dp/pull_dv3/
pull_r2d/pull_r2d_local .json) and writes ONE canonical numbers file:
paper/figs/ignition_numbers_20260818.json
"""
import json, os, sys, collections

SP = sys.argv[1]
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "paper", "figs", "ignition_numbers_20260818.json")

def L(n): return json.load(open(os.path.join(SP, n)))

rlpd = L("pull_rlpd.json")["rlpd"]
dp   = L("pull_dp.json")
dv3  = L("pull_dv3.json")
r2   = L("pull_r2d.json")
r2l  = L("pull_r2d_local.json")

D = {"rlpd": {}, "dp": {}, "dv3": {}, "r2d": {}}

# ---------------- RLPD ----------------
for key, v in rlpd.items():
    arm, fam, seed = key.split("|"); seed = int(seed)
    curve = {}
    for row in v["hist"]:
        s = row.get("eval/train_step")
        if s is None: continue
        curve[int(s)] = {"indist": row.get("eval_indist/picked"),
                         "random": row.get("eval_random/picked"),
                         "n": row.get("eval_indist/n")}
    D["rlpd"].setdefault(f"{arm}_{fam}", {})[seed] = {
        "created": v["created"], "run_id": v["run_id"], "state": v["state"],
        "demo_dir": v["config"].get("demo_dir"),
        "curve": {str(k): curve[k] for k in sorted(curve)},
        "final_step": max(curve) if curve else None,
        "final_indist": curve[max(curve)]["indist"] if curve else None,
        "final_random": curve[max(curve)]["random"] if curve else None,
        "eval_n": curve[max(curve)]["n"] if curve else None,
    }

# ---------------- DP ----------------
for key, evs in dp.items():
    src, seed = key.split("|"); seed = int(seed)
    evs = sorted(evs, key=lambda e: e["created"])
    record = [e for e in evs if e["created"] < "2026-08-10"]
    reeval = [e for e in evs if e["created"] >= "2026-08-10"]
    D["dp"].setdefault(src, {})[seed] = {
        "record": record[0] if record else None,
        "hardened_reeval": reeval[0] if reeval else None}

# ---------------- dv3 ----------------
for seed, v in dv3["train"].items():
    fresh = sorted(dv3["eval"].get(seed, []), key=lambda e: e["step"])
    D["dv3"][int(seed)] = {
        "created": v["created"], "final_step": v["final_step"],
        "inloop": [{"step": r["_step"], "picked": r.get("eval/picked")} for r in v["eval_rows"]],
        "fresh": [{"step": e["step"], "picked": e["summary"].get("policy_eval/picked"),
                   "n": e["summary"].get("policy_eval/n"), "created": e["created"]} for e in fresh]}

# ---------------- r2dreamer ----------------
alltr = dict(r2["train"]); alltr.update(r2l["train"])
allev = dict(r2["eval"]);  allev.update(r2l["eval"])
# family -> (source, wave label, era)
FAM = {
 "pick_delta25d4": ("dH", "local (delta recipe)", "pre-E1/E2; local box; 08-11/12"),
 "pick_d4clamp":   ("dH", "local (delta recipe)", "pre-E1/E2; local box; 08-11/12 clamp ablation"),
 "dH_R2Dshort":    ("dH", None, None),
 "dDP_R2Dshort":   ("dDP", None, None),
 "pick_v5d4c_delta": ("dH", "cluster wave 3", "post-E2; 3M steps; 08-17/18"),
 "dH_R2D":         ("dH", "cluster 3M wave (SUPERSEDED)", "08-11/12; superseded by the *short* waves"),
 "pick_delta":     ("dH", "local pilot (VOID)", "evals before 2026-08-11T03:00 are VOID per the delta-eval cutoff"),
 "dDP_R2D":        ("dDP", "cluster 3M wave (SUPERSEDED)", "08-11/12; dDP prefill/reward-dilution defect"),
}
for key in sorted(set(list(alltr) + list(allev))):
    fam, seed = key.split("|"); seed = int(seed)
    src, wave, era = FAM[fam]
    if wave is None:  # the *short* families split by seed block
        if seed <= 19: wave, era = "cluster wave 1", "pre-E1/E2 env; ~1M steps; 08-12/13"
        else:          wave, era = "cluster wave 2 (firming)", "post-E1/E2 env; ~1M steps; 08-14/15"
    tr = alltr.get(key, {})
    ev = sorted(allev.get(key, []), key=lambda e: e["step"])
    vals = [e["picked"] for e in ev if e["picked"] is not None]
    D["r2d"][key] = {
        "family": fam, "source": src, "wave": wave, "era": era, "seed": seed,
        "created": tr.get("created"), "env_step_final": tr.get("env_step_final"),
        "inloop": [{"step": r.get("env_step"), "picked": r.get("eval/picked"),
                    "mode": r.get("eval/mode_flag")} for r in tr.get("rows", [])
                   if r.get("env_step") is not None],
        "fresh": [{"step": e["step"], "picked": e["picked"], "created": e["created"]} for e in ev],
        "fresh_best": max(vals) if vals else None,
        "fresh_n_evals": len(vals),
        "inloop_best": max([r.get("eval/picked") for r in tr.get("rows", [])
                            if r.get("eval/picked") is not None] or [None]) if tr.get("rows") else None,
    }

os.makedirs(os.path.dirname(OUT), exist_ok=True)
json.dump(D, open(OUT, "w"), indent=1, default=str)
print("wrote", OUT)

# ---- quick console audit against the docs ----
def rate(vals, bar): return sum(1 for v in vals if v is not None and v >= bar), len(vals)
print("\n== RLPD final in-train eval_indist/picked (n=10 eps each) ==")
for famk in ["dH_n20", "dR2D_n20", "dDP_pair"]:
    d = D["rlpd"][famk]
    vals = [d[s]["final_indist"] for s in sorted(d)]
    ig, n = rate(vals, 0.20)
    print(f"{famk:10s} n={n:2d} ignited(>=0.20)={ig}  mean={sum(vals)/n:.3f}  vals={[round(v,2) for v in vals]}")
print("\n== DP eval_indist/picked ==")
for src in ["dH", "dDP"]:
    d = D["dp"][src]
    rec = [d[s]["record"]["indist"] for s in sorted(d)]
    har = [d[s]["hardened_reeval"]["indist"] for s in sorted(d)]
    print(f"{src:4s} record  n={len(rec)} mean={sum(rec)/len(rec):.3f} range={min(rec):.2f}-{max(rec):.2f} {[round(v,2) for v in rec]}")
    print(f"{src:4s} re-eval n={len(har)} mean={sum(har)/len(har):.3f} range={min(har):.2f}-{max(har):.2f} {[round(v,2) for v in har]}")
print("\n== r2dreamer best FRESH checkpoint eval per seed (n=15 eps/eval) ==")
byw = collections.defaultdict(list)
for k, v in sorted(D["r2d"].items()):
    byw[(v["source"], v["wave"])].append((v["seed"], v["fresh_best"], v["inloop_best"]))
for (src, wave), lst in sorted(byw.items()):
    vals = [b for _, b, _ in lst]
    nz = sum(1 for v in vals if v is not None and v > 0)
    ig = sum(1 for v in vals if v is not None and v >= 0.20)
    print(f"{src:4s} {wave:32s} n={len(lst):2d} nonzero={nz} >=0.20={ig}  {[(s, None if b is None else round(b,3)) for s,b,_ in lst]}")
print("\n== dv3 msrecipe ==")
for s in sorted(D["dv3"], key=int):
    v = D["dv3"][s]
    print(f"s{s} inloop={[r['picked'] for r in v['inloop']]} fresh={[(e['step'], e['picked'], 'n='+str(e['n'])) for e in v['fresh']]}")
