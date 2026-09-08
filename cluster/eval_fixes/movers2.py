import glob, json, os
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"
REAL = ("picked", "contact", "nested")          # statistics that existed in BOTH record and re-score
NEW  = ("placed_v2",)                            # structurally 0 in the full scope before amendment (j) fix 4
tot = only_new = real_move = 0
real_rows, arm_new = [], {"HUMAN": 0, "MACHINE": 0}
for cp in sorted(glob.glob(W + "/runs/*/fresh_eval_*_cp/metrics.json")):
    rec = cp.replace("_cp/metrics.json", "/metrics.json")
    if not os.path.exists(rec): continue
    a, b = json.load(open(rec)), json.load(open(cp))
    pa, pb = a["per_episode"], b["per_episode"]
    if len(pa) != len(pb): continue
    tot += 1
    cell = cp.replace(W + "/runs/", "").replace("/metrics.json", "")
    arm = "HUMAN" if ("_dH" in cell or "dHfull" in cell) else "MACHINE"
    d_ep = sum(1 for x, y in zip(pa, pb) if x["outcome"] != y["outcome"] or x["steps"] != y["steps"])
    dreal = {}
    for k in REAL:
        ra = sum(1 for x in pa if (x.get("stages") or {}).get(k)); rb = sum(1 for y in pb if (y.get("stages") or {}).get(k))
        if ra != rb: dreal[k] = (ra, rb)
    dnew = {}
    for k in NEW:
        ra = sum(1 for x in pa if (x.get("stages") or {}).get(k)); rb = sum(1 for y in pb if (y.get("stages") or {}).get(k))
        if ra != rb: dnew[k] = (ra, rb)
    if dnew and not dreal and d_ep == 0:
        only_new += 1; arm_new[arm] += 1
    if dreal or d_ep:
        real_move += 1
        real_rows.append((cell, arm, len(pa), d_ep, dreal, json.load(open(rec)).get("scope")))
print("compared %d record/_cp cell pairs" % tot)
print("  cells where ONLY the NEW placed_v2 column moved, with 0 differing episodes: %d  (human %d / machine %d)"
      % (only_new, arm_new["HUMAN"], arm_new["MACHINE"]))
print("  cells where a PRE-EXISTING statistic (picked/contact/nested) or any episode moved: %d" % real_move)
for cell, arm, n, d_ep, dreal, scope in real_rows:
    print("   %-72s %-7s scope=%-12s eps=%d differing_eps=%d %s" % (cell[:72], arm, scope, n, d_ep, dreal))
