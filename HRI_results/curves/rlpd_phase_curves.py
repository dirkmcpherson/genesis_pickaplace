#!/usr/bin/env python3
"""Per-phase acquisition curves for the {RLPD} e2e long runs.

usage: rlpd_phase_curves.py <checkpoints/e2e dir> <out.csv> [n_bins]

r2dreamer's curves come from metrics.jsonl via learning_curves.py. RLPD writes no metrics of
that shape, but PHASE_PLAN amendment (u)'s EpisodeRolloutLogCallback writes one JSONL row per
finished online rollout episode with sticky episode/train_ep_<stage> flags -- the same content
in a different container. This bins those on a fixed decision grid and emits the SAME CSV schema
learning_curves.py produces, so the identical plotter draws both learners.

These are ONLINE TRAINING ROLLOUTS from the exploring policy -- NOT the evaluation protocol.
An endpoint here is not a table number.
"""
import csv, glob, json, os, sys
import numpy as np

src, out_csv = sys.argv[1], sys.argv[2]
NB = int(sys.argv[3]) if len(sys.argv) > 3 else 20
STAGES = ("picked", "placed_v2", "contact", "contact_push", "slide_success", "nested")
ARMS = (("human", "e2e_rlpd_dH_s90*"), ("machine", "e2e_rlpd_dDPfirst_s92*"))

def seed_rows(d):
    p = os.path.join(d, "episode_rollouts.jsonl")
    rows = []
    if not os.path.exists(p):
        return rows
    for line in open(p):
        line = line.strip()
        if not line:
            continue
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue          # a run still writing may end mid-line; skip only the last partial
        if "step" in o:
            rows.append(o)
    return rows

data = {}
for arm, pat in ARMS:
    for d in sorted(glob.glob(os.path.join(src, pat))):
        r = seed_rows(d)
        if len(r) >= 50:      # a run must have some history before it can be a curve
            data[(arm, os.path.basename(d))] = r
if not data:
    sys.exit("no RLPD episode records found under " + src)

grid_max = max(max(o["step"] for o in r) for r in data.values())
edges = np.linspace(0, grid_max, NB + 1)
recs = []
for stage in STAGES:
    for arm, _ in ARMS:
        per_seed, ids = [], []
        for (a, name), rows in sorted(data.items()):
            if a != arm:
                continue
            key = "episode/train_ep_" + stage
            if not any(key in o for o in rows):
                continue      # stage never emitted by this producer; do not invent zeros
            b = []
            for i in range(NB):
                sel = [float(o.get(key, 0.0)) for o in rows
                       if edges[i] <= o["step"] < edges[i + 1] and key in o]
                b.append(float(np.mean(sel)) if sel else np.nan)
            per_seed.append(b); ids.append(name.rsplit("_s", 1)[1])
        if not per_seed:
            continue
        M = np.array(per_seed, dtype=float)
        for i in range(NB):
            col = M[:, i]; col = col[~np.isnan(col)]
            if col.size == 0:
                continue
            recs.append(dict(family="e2e", arm=arm, stage=stage, bin=i,
                             step=float((edges[i] + edges[i + 1]) / 2),
                             mean=float(col.mean()),
                             se=float(col.std(ddof=1) / np.sqrt(col.size)) if col.size > 1 else 0.0,
                             n_seeds=int(col.size),
                             per_seed=" ".join(f"{v:.4f}" for v in col),
                             seed_ids=" ".join(ids)))
with open(out_csv, "w", newline="") as fh:
    fh.write("# {RLPD} e2e per-phase acquisition from episode_rollouts.jsonl (amendment (u) records).\n")
    fh.write("# ONLINE TRAINING ROLLOUTS from the exploring policy -- NOT the evaluation protocol.\n")
    fh.write("# Endpoints are NOT table numbers (eval uses sampled/mode actions on rnd30/spots60, final ckpt).\n")
    w = csv.DictWriter(fh, fieldnames=list(recs[0].keys())); w.writeheader()
    for r in recs:
        w.writerow(r)
print(f"wrote {out_csv}: {len(recs)} rows, "
      f"{len({k[1] for k in data})} runs, stages={sorted({r['stage'] for r in recs})}")
