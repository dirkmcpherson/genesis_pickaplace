import glob, json, os, re, subprocess
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"
REAL = ("picked", "contact", "nested")
# one pass over slurm logs: which job wrote which eval dir
path2job = {}
for f in glob.glob(W + "/slurm/*.out"):
    jid = "".join(c for c in os.path.basename(f).rsplit("_", 1)[-1] if c.isdigit())
    if not jid: continue
    try: txt = open(f, errors="ignore").read()
    except Exception: continue
    for m in re.finditer(r"(/cluster/\S*/runs/\S*/fresh_eval_[A-Za-z0-9_]+)", txt):
        path2job.setdefault(m.group(1), jid)
job2node, node2cores = {}, {}
def node_of(jid):
    if jid in job2node: return job2node[jid]
    o = subprocess.run(["sacct", "-j", jid, "--format=NodeList,State", "-n", "-P"], capture_output=True, text=True, timeout=60).stdout.strip().split("\n")
    n, st = ("?", "?")
    if o and o[0]:
        parts = o[0].split("|"); n = parts[0]; st = parts[1] if len(parts) > 1 else "?"
    job2node[jid] = (n, st); return (n, st)
def cores(n):
    if n in node2cores: return node2cores[n]
    o = subprocess.run(["scontrol", "show", "node", n], capture_output=True, text=True, timeout=30).stdout
    c = next((t.split("=")[1] for t in o.split() if t.startswith("CPUTot=")), "?")
    node2cores[n] = c; return c
tab = {}; states = {}
for cp in sorted(glob.glob(W + "/runs/*/fresh_eval_*_cp/metrics.json")):
    rec = cp.replace("_cp/metrics.json", "/metrics.json"); rdir = os.path.dirname(rec)
    if not os.path.exists(rec): continue
    a, b = json.load(open(rec)), json.load(open(cp))
    pa, pb = a["per_episode"], b["per_episode"]
    if len(pa) != len(pb): continue
    d_ep = sum(1 for x, y in zip(pa, pb) if x["outcome"] != y["outcome"] or x["steps"] != y["steps"])
    moved = d_ep > 0 or any(sum(1 for x in pa if (x.get("stages") or {}).get(k)) != sum(1 for y in pb if (y.get("stages") or {}).get(k)) for k in REAL)
    jid = path2job.get(rdir)
    n, st = node_of(jid) if jid else ("?", "?")
    c = cores(n) if n != "?" else "?"
    cls = "36-core" if c == "36" else ("other(%s)" % c)
    arm = "HUMAN" if ("_dH" in cp or "dHfull" in cp) else "MACHINE"
    tab[(cls, moved)] = tab.get((cls, moved), 0) + 1
    tab[(cls + "|" + arm, moved)] = tab.get((cls + "|" + arm, moved), 0) + 1
    if moved: states[st] = states.get(st, 0) + 1
print("CONTINGENCY: record-node class  x  did a pre-existing statistic/episode move?")
for k in sorted(set(k[0] for k in tab if "|" not in k[0])):
    mv = tab.get((k, True), 0); nm = tab.get((k, False), 0)
    print("   %-12s moved %3d   unmoved %3d" % (k, mv, nm))
print("\nsame, split by arm:")
for k in sorted(set(k[0] for k in tab if "|" in k[0])):
    mv = tab.get((k, True), 0); nm = tab.get((k, False), 0)
    print("   %-22s moved %3d   unmoved %3d" % (k, mv, nm))
print("\nslurm State of the record jobs behind moved cells:", states)
