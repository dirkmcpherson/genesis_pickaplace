"""Retro-fill hardware provenance INTO each _v2 cell's metrics.json from its hardware.json sidecar.
Metadata only: adds node / cpu_model / cpu_cores / slurm_job / hw_source. Never touches a measurement.
Atomic (tmp + rename) and idempotent."""
import glob, json, os, sys
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"
dry = "--write" not in sys.argv
n_done = n_skip = n_nohw = 0
changed_numbers = []
for m in sorted(glob.glob(W + "/runs/*/fresh_eval_*_v2/metrics.json")):
    hw_p = os.path.join(os.path.dirname(m), "hardware.json")
    if not os.path.exists(hw_p):
        n_nohw += 1; continue
    j = json.load(open(m))
    if j.get("node"):
        n_skip += 1; continue
    h = json.load(open(hw_p))
    before = {k: v for k, v in j.items() if k != "per_episode"}
    j["node"] = h.get("node")
    j["cpu_model"] = (h.get("cpu_model") or "").strip()
    j["cpu_cores"] = h.get("ncpus_machine", h.get("ncpus_cgroup"))
    j["slurm_job"] = h.get("slurm_job")
    j["hw_source"] = "hardware.json sidecar, back-filled 2026-09-08 (metadata only)"
    after = {k: v for k, v in j.items() if k != "per_episode" and k in before}
    if any(before[k] != after[k] for k in before):
        changed_numbers.append(m); continue
    if not dry:
        tmp = m + ".tmp"
        with open(tmp, "w") as fh: json.dump(j, fh, indent=1)
        os.replace(tmp, m)
    n_done += 1
print(("DRY RUN: would back-fill" if dry else "back-filled") + " %d cells | already had it %d | no sidecar %d" % (n_done, n_skip, n_nohw))
if changed_numbers:
    print("REFUSED (a pre-existing value would change):"); [print("   ", c) for c in changed_numbers]
