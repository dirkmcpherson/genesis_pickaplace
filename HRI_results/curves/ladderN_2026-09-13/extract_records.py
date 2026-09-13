#!/usr/bin/env python3
"""Extract per-episode stage records from every Ladder-N run into compact CSVs (run on the cluster login node).

{r2dreamer}: <run>/metrics.jsonl rows with episode/score carry the sticky episode/train_ep_* record (amendment (w));
  step is converted to ONLINE steps by subtracting step_contract.json:prefill_counter_origin.
{RLPD}: <run>/episode_rollouts.jsonl rows carry episode/train_ep_* for the OLDER stage set (no farside/slide_event/home);
  step is in decisions (the training budget unit).
Output: records/<learner>__<set>__s<seed>.csv with columns step,<stages...>; plus records/INDEX.tsv (learner,set,seed,
ladder,budget,n_episodes,last_step,source_path).
"""
import json, glob, os, sys, csv
LAB = "/cluster/tufts/shortlab/jstale02"
W = LAB + "/wm_fix_2026-09-03"
OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
R2_STAGES = ["picked", "placed_v2", "farside", "slide_event", "home", "nested_v2", "tipped"]
RL_STAGES = ["picked", "placed_v2", "nested_v2", "slide_success", "tipped"]
index = []

def write(name, rows, stages):
    with open(os.path.join(OUT, name + ".csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["step"] + stages); w.writerows(rows)

# ---- r2dreamer ----
for d in sorted(glob.glob(W + "/runs/full_r2d_state_*_r*h_s9[5-8][0-9]")):
    base = os.path.basename(d)[len("full_r2d_state_"):]
    setname, seed = base.rsplit("_s", 1)
    if setname.split("_")[-1] not in ("rnrh", "rnsh", "rns10h"):
        continue
    try:
        origin = int(json.load(open(d + "/step_contract.json"))["prefill_counter_origin"])
    except Exception as e:
        print("SKIP (no contract)", d, e); continue
    prov = json.load(open(d + "/ladder_provenance.json"))
    rows = []
    with open(d + "/metrics.jsonl") as f:
        for line in f:
            try: r = json.loads(line)
            except json.JSONDecodeError: continue
            if "episode/score" not in r: continue
            rec = r.get("episode/train_ep_record_valid")
            if rec is not None and rec != 1: continue
            vals = [r.get("episode/train_ep_" + k) for k in R2_STAGES]
            if any(v not in (0, 1) for v in vals): continue
            rows.append([int(r["step"]) - origin] + [int(v) for v in vals])
    write(f"r2dreamer__{setname}__s{seed}", rows, R2_STAGES)
    index.append(["r2dreamer", setname, seed, prov.get("ladder"), prov.get("steps", "4000000"), len(rows), rows[-1][0] if rows else 0, d])
    print("r2", setname, seed, len(rows), "episodes, last online step", rows[-1][0] if rows else None)

# ---- RLPD ----
roots = [LAB + "/gp_ladderN/baselines/rl/checkpoints/e2e", LAB + "/gp_ladderN/baselines/rl/checkpoints/e2e_rev3",
         LAB + "/gp_ac/baselines/rl/checkpoints/e2e_ac"]
for root in roots:
    for d in sorted(glob.glob(root + "/e2e_rlpd_*")):
        prov = json.load(open(d + "/ladder_provenance.json"))
        setname = os.path.basename(prov["arm"]); seed = prov["seed"]
        rows = []
        with open(d + "/episode_rollouts.jsonl") as f:
            for line in f:
                try: r = json.loads(line)
                except json.JSONDecodeError: continue
                vals = [r.get("episode/train_ep_" + k) for k in RL_STAGES]
                if any(v not in (0, 1) for v in vals): continue
                rows.append([int(r["step"])] + [int(v) for v in vals])
        write(f"rlpd__{setname}__s{seed}", rows, RL_STAGES)
        index.append(["rlpd", setname, seed, prov.get("ladder"), prov.get("steps"), len(rows), rows[-1][0] if rows else 0, d])
        print("rlpd", setname, seed, len(rows), "episodes, last decision", rows[-1][0] if rows else None)

with open(os.path.join(OUT, "INDEX.tsv"), "w") as f:
    f.write("learner\tset\tseed\tladder\tbudget\tn_episodes\tlast_step\tsource\n")
    for row in index: f.write("\t".join(str(x) for x in row) + "\n")
