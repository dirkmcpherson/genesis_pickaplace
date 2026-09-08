import glob, json, os, collections
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"
cells = sorted(glob.glob(W + "/runs/*/fresh_eval_*_v2/metrics.json"))
have_side = have_field = 0
by_class = collections.Counter(); e2e_class = collections.Counter()
missing = []
for m in cells:
    d = os.path.dirname(m)
    hw = os.path.join(d, "hardware.json")
    j = json.load(open(m))
    if j.get("node") or j.get("cpu_model"): have_field += 1
    if os.path.exists(hw):
        have_side += 1
        h = json.load(open(hw))
        cm = (h.get("cpu_model") or "?").strip()
        by_class[cm] += 1
        if j.get("scope") == "full": e2e_class[cm] += 1
    else:
        missing.append(d.replace(W + "/runs/", ""))
print("_v2 cells: %d | with hardware.json sidecar: %d | with node/cpu INSIDE metrics.json: %d" % (len(cells), have_side, have_field))
print("\nCPU model across all _v2 cells:")
for k, v in by_class.most_common(): print("   %-42s %d" % (k, v))
print("\nCPU model across the END-TO-END (_v2, scope=full) cells -- these are the pinned ones:")
for k, v in e2e_class.most_common(): print("   %-42s %d" % (k, v))
if missing: print("\ncells with NO sidecar (%d):" % len(missing)); [print("   ", x) for x in missing[:5]]
