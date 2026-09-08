#!/usr/bin/env python3
"""Cluster-side: node provenance for every world-model cell that has a `_cp` re-score.

Answers the standing explanation for cells that do not reproduce -- "the re-score ran on a
different class of machine". Emits one row per (run, cell, mode) with:

  * the ORIGINAL record node: the training job's node. The fresh_eval of record runs at the end
    of the training job, so the tfevents filename identifies it.
  * the RE-SCORE node: the cpsc lane job's `host=` stamp, matched to the cell by its latest
    START line before the re-scored metrics.json mtime.

It also prints an arch x core-count census of the cluster, because the claim that instruction
set and core count are collinear here is itself testable.

usage: python3 harvest_nodes.py > node_provenance.csv
"""
import glob, json, os, re, subprocess, sys, csv
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"

# node -> (cores, features)
node = {}
out = subprocess.run(["sinfo","-h","-N","-o","%n %c %f"], capture_output=True, text=True).stdout
for ln in out.splitlines():
    p = ln.split()
    if len(p) >= 3:
        node[p[0]] = (int(p[1]), p[2])

# --- ISA vs core-count collinearity check (broadwell = AVX2, no AVX-512)
import collections
c = collections.Counter()
for n,(cores,feat) in node.items():
    arch = feat.split(',')[0]
    c[(arch, cores)] += 1
print("### arch x cores census (unique nodes)")
for k,v in sorted(c.items()):
    print(f"   {k[0]:16s} {k[1]:>3d} cores : {v} nodes")

# --- rescore host per (run, tag, mode): the latest START before the metrics mtime
starts = []
for f in sorted(glob.glob(os.path.join(W, "slurm", "cpsc_rescore_*.out"))):
    host = None
    for ln in open(f, errors="ignore"):
        m = re.match(r"# (\S+) host=(pax\d+)", ln)
        if m: host = m.group(2)
        m = re.match(r"# (\S+) START (\S+) (\S+) (\S+)", ln)
        if m and host:
            starts.append((m.group(1), m.group(2), m.group(3), m.group(4), host))
starts.sort()

import datetime
def ts(s):
    return datetime.datetime.fromisoformat(s).timestamp()

rows = []
for mpath in sorted(glob.glob(os.path.join(W, "runs", "*", "fresh_eval_*_cp", "metrics.json"))):
    run = mpath.split(os.sep)[-3]
    ev = mpath.split(os.sep)[-2][len("fresh_eval_"):-3]   # strip trailing _cp
    m = re.match(r"(.+)_(sample|mode)$", ev)
    if not m: continue
    tag, mode = m.group(1), m.group(2)
    mt = os.path.getmtime(mpath)
    cands = [s for s in starts if s[1]==run and s[2]==tag and s[3]==mode and ts(s[0]) <= mt]
    rescore_host = cands[-1][4] if cands else ""
    # original cell's node = the TRAINING node (the fresh_eval of record runs in the train job)
    tev = glob.glob(os.path.join(W, "runs", run, "events.out.tfevents.*"))
    oh = ""
    if tev:
        mm = re.search(r"tfevents\.\d+\.(pax\d+)\.", os.path.basename(sorted(tev)[-1]))
        oh = mm.group(1) if mm else ""
    rows.append(dict(run=run, tag=tag, mode=mode,
                     rescore_host=rescore_host,
                     rescore_cores=node.get(rescore_host,("",""))[0],
                     rescore_arch=node.get(rescore_host,("",""))[1].split(',')[0] if rescore_host else "",
                     orig_host=oh, orig_cores=node.get(oh,("",""))[0],
                     orig_arch=node.get(oh,("",""))[1].split(',')[0] if oh else ""))
w = csv.DictWriter(sys.stdout, fieldnames=list(rows[0].keys()))
print("### cells"); w.writeheader()
for r in rows: w.writerow(r)
