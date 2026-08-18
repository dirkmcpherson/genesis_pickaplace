#!/usr/bin/env python
"""Raw wandb pulls backing the 2026-08-18 ignition figures.

Usage:  <venv>/bin/python analysis/pull_ignition_wandb_20260818.py <outdir>
Writes pull_rlpd.json, pull_dp.json, pull_dv3.json, pull_r2d.json and
pull_r2d_local.json into <outdir>; feed that dir to
analysis/assemble_ignition_20260818.py.
"""
import os, sys, json, re
import wandb
OUTDIR = os.path.abspath(sys.argv[1]); os.makedirs(OUTDIR, exist_ok=True)
os.chdir(OUTDIR)
api = wandb.Api(timeout=240)

# ------------------------------------------------------------ pull.py
OUT = {}

def hist(r, keys):
    rows = []
    for row in r.scan_history(keys=keys, page_size=2000):
        rows.append(row)
    return rows

# ---------- RLPD ----------
rl = {}
for r in api.runs("jambotime/genesis_paper"):
    m = re.match(r"^(dH|dR2D)_RLPD-n20_s(\d+)$", r.name) or re.match(r"^(dDP)_RLPD-(pair)_s(\d+)$", r.name)
    if not m: continue
    if r.name.startswith("dDP"):
        arm, seed = "dDP", int(m.group(3)); fam = "pair"
    else:
        arm, seed = m.group(1), int(m.group(2)); fam = "n20"
    h = hist(r, ["eval/train_step","eval_indist/picked","eval_random/picked","eval_indist/n","eval_random/n"])
    rl[f"{arm}|{fam}|{seed}"] = {
        "run_id": r.id, "created": str(r.created_at)[:19], "state": r.state,
        "config": {k: r.config.get(k) for k in ["seed","steps","demo_dir","utd","action_mode","eval_freq","gamma"]},
        "summary": {k: r.summary.get(k) for k in ["eval_indist/picked","eval_random/picked","eval_indist/n","eval_random/n","eval/train_step"]},
        "hist": h}
    print("RLPD", r.name, len(h), file=sys.stderr)
OUT["rlpd"] = rl
json.dump(OUT, open("pull_rlpd.json","w"), indent=1, default=str)
print("done rlpd", len(rl), file=sys.stderr)

# ------------------------------------------------------------ pull2b.py
dv3 = {"train": {}, "eval": {}}
for r in api.runs("jambotime/dreamer_v3"):
    m = re.match(r"^genesis_pixels_dH_msr_ar4_s(\d+)-joint$", r.name)
    if m:
        rows=[x for x in r.scan_history(keys=["_step","eval/picked","eval/placed"], page_size=5000)]
        tr=[x for x in r.scan_history(keys=["_step","train_return","train_success_rate"], page_size=5000)]
        dv3["train"][int(m.group(1))]={"created":str(r.created_at)[:19],"id":r.id,
            "final_step":r.summary.get("_step"),
            "summary":{k:r.summary.get(k) for k in ["eval/picked","train_return","train_success_rate","eval_rollouts"]},
            "eval_rows":rows,"train_rows":tr[::20]}
        print("train",r.name,len(rows),len(tr),file=sys.stderr); continue
    m = re.match(r"^dH_msr_ar4_s(\d+)-eval-step(\d+)$", r.name)
    if m:
        dv3["eval"].setdefault(int(m.group(1)),[]).append({"step":int(m.group(2)),
            "created":str(r.created_at)[:19],
            "summary":{k:v for k,v in r.summary.items() if not k.startswith("_") and not isinstance(v,dict)}})
        print("eval",r.name,file=sys.stderr)
json.dump(dv3, open("pull_dv3.json","w"), indent=1, default=str)

# ------------------------------------------------------------ pull3.py
TRAIN = re.compile(r"^(dH_R2D|dDP_R2D|dH_R2Dshort|dDP_R2Dshort|pick_v5d4c_delta|pick_delta25d4)_s(\d+)$")
EVAL  = re.compile(r"^(dH_R2D|dDP_R2D|dH_R2Dshort|dDP_R2Dshort|pick_v5d4c_delta|pick_delta25d4)_s(\d+)-eval-step(\d+)$")
out = {"train": {}, "eval": {}}
for r in api.runs("jambotime/r2dreamer_genesis"):
    m = TRAIN.match(r.name)
    if m:
        rows=[x for x in r.scan_history(keys=["env_step","eval/picked","eval/mode_flag"], page_size=5000)]
        out["train"][f"{m.group(1)}|{int(m.group(2))}"]={"created":str(r.created_at)[:19],"state":r.state,
            "id":r.id,"env_step_final":r.summary.get("env_step"),
            "summary":{k:r.summary.get(k) for k in ["eval/picked","episode/train_picked","env_step"]},
            "rows":rows}
        print("T",r.name,len(rows),file=sys.stderr); continue
    m = EVAL.match(r.name)
    if m:
        out["eval"].setdefault(f"{m.group(1)}|{int(m.group(2))}",[]).append({
            "step":int(m.group(3)),"created":str(r.created_at)[:19],"id":r.id,
            "picked":r.summary.get("eval/picked"),"mode":r.summary.get("eval/mode_flag"),
            "n":r.summary.get("eval/n"),
            "keys":sorted([k for k in r.summary.keys() if not k.startswith("_")])})
json.dump(out, open("pull_r2d.json","w"), indent=1, default=str)
print("done",len(out["train"]),len(out["eval"]),file=sys.stderr)

# ------------------------------------------------------------ pull4.py
TR = re.compile(r"^(pick_d4clamp|pick_delta)_s(\d+)$")
EV = re.compile(r"^(pick_d4clamp|pick_delta)_s(\d+)-eval-step(\d+)$")
out={"train":{}, "eval":{}}
for r in api.runs("jambotime/r2dreamer_genesis"):
    m=TR.match(r.name)
    if m:
        rows=[x for x in r.scan_history(keys=["env_step","eval/picked","eval/mode_flag"],page_size=5000)]
        out["train"][f"{m.group(1)}|{int(m.group(2))}"]={"created":str(r.created_at)[:19],"id":r.id,
          "env_step_final":r.summary.get("env_step"),"rows":rows}
        print("T",r.name,len(rows),file=sys.stderr); continue
    m=EV.match(r.name)
    if m:
        out["eval"].setdefault(f"{m.group(1)}|{int(m.group(2))}",[]).append(
          {"step":int(m.group(3)),"created":str(r.created_at)[:19],"picked":r.summary.get("eval/picked"),
           "mode":r.summary.get("eval/mode_flag")})
json.dump(out,open("pull_r2d_local.json","w"),indent=1,default=str)
for k,v in out["eval"].items():
    vals=sorted([(e["step"],e["picked"],e["created"][:10]) for e in v])
    print(k,"n=",len(vals),"best=",max(x[1] for x in vals if x[1] is not None),vals[:20],file=sys.stderr)
