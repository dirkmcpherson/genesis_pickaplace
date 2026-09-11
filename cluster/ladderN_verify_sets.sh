#!/usr/bin/env bash
# Verify the six Ladder-N demonstration sets INDEPENDENTLY of the builder that wrote them
# (PHASE_PLAN amendment (aa), step 3). Read-only; safe to re-run.
#
#   bash cluster/ladderN_verify_sets.sh
#
# What it checks, per set:
#   1. tape count, Sigma reward, the rungs tapes reach, and specifically how many pay `home`
#      (P-aa-7: 13 human / 14 machine +- 1 against this box's 32-core measurement)
#   2. the manifest carries the ladder stamp, the tip guard and -- on the machine sets --
#      `one_per_ic_first` inherited from the source (not re-derived from a CLI flag: that was
#      the 2026-09-09 false-claim defect)
#   3. the action stream sha256 is identical to the SOURCE set, tape for tape
#   4. the node that built it
#   5. BOTH launcher gates: the {RLPD} one via DRYRUN=1, the {r2dreamer} one by running its own
#      gate block against the set
set -uo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
GP=${GP:-$LAB/gp_ladderN}
PY=${PY:-$LAB/r2d_venv/bin/python}
DEMOS=$W/demos_state_full
SETS=${SETS:-"dHfull_all_rnrh dDPfull_first_rnrh dHfull_all_rnsh dDPfull_first_rnsh dHfull_all_rzh dDPfull_first_rzh"}

echo "===================== 1-4: manifests, grants, action fidelity ====================="
"$PY" - "$DEMOS" $SETS <<'PY'
import glob, hashlib, json, os, sys
import numpy as np
root, sets = sys.argv[1], sys.argv[2:]
def sha(a): return hashlib.sha256(np.ascontiguousarray(np.asarray(a, np.float32)).tobytes()).hexdigest()
def actions(z):
    return np.asarray(z['action'], np.float32)[1:] if 'action' in z.files else np.asarray(z['actions_delta'], np.float32)
for s in sets:
    d = os.path.join(root, s)
    man = json.load(open(os.path.join(d, 'manifest.json')))
    rep = json.load(open(os.path.join(d, 'repeat.json')))
    src = rep['relabel']['source_set']
    fs = sorted(glob.glob(os.path.join(d, '*.npz')))
    ident = home_uids = 0
    home_list = []
    for f in fs:
        g = os.path.join(src, os.path.basename(f))
        zo, zs = np.load(f, allow_pickle=True), np.load(g, allow_pickle=True)
        ident += int(sha(actions(zo)) == sha(actions(zs)))
        gr = json.loads(str(zo['rz_grants']))
        if gr.get('home'):
            home_uids += 1
            home_list.append(os.path.basename(f).split('-')[-2])
    tg = man['tapes_granting']
    print(f"\n--- {s} ---")
    print(f"  tapes            : {len(fs)}  (repeat.json n_written={rep['n_written']})")
    print(f"  Sigma reward     : {man['reward_total_new']:.2f}   (source recorded {man['reward_total_old']:.1f})")
    print(f"  action sha256    : {ident}/{len(fs)} tapes identical to {os.path.basename(src)}")
    print(f"  tapes granting   : { {k: v for k, v in sorted(tg.items()) if v} }")
    print(f"  HOME tapes       : {home_uids}   uids {sorted(home_list)}")
    print(f"  end reasons      : {man['end_reasons']}")
    print(f"  tip guard        : {man['tip_guard']} (sustain {man['tip_guard_sustain_frames']} frames)")
    print(f"  one_per_ic_first : {rep.get('one_per_ic_first')}  (inherited {rep['relabel']['selection_inherited']})")
    print(f"  node             : {rep.get('relabel_node')}")
    print(f"  can_dev p50/max  : {man['can_dev_p50_m']*1000:.1f} / {man['can_dev_max_m']*1000:.1f} mm; "
          f"{man['n_tapes_can_dev_over_1cm']} tapes over 1 cm")
    print(f"  stamp            : {man['ladder_stamp']}")
PY

echo
echo "===================== 5: the two launcher gates ====================="
for s in $SETS; do
  case "$s" in dH*) ARM=dH ;; dDP*) ARM=dDPfirst ;; esac
  case "$s" in *_rnrh) L=nested_ramp ;; *_rnsh) L=nested_sparse ;; *_rzh) L=staged ;; esac
  echo "--- $s (ARM=$ARM LADDER=$L) ---"
  ( cd "$GP" && DRYRUN=1 GENESIS_PICKAPLACE_ROOT=$GP LADDER=$L TIP_GUARD=not_in_hand ARM=$ARM SEED=0 \
      DEMO=$DEMOS/$s bash cluster/sbatch_rlpd_e2e.sh 2>&1 | grep -E '^DEMO-SHA|^FATAL|Error|^\[dry\] ARM' ) || echo "  RLPD GATE FAILED"
  REPEAT=4 "$PY" - "$DEMOS/$s" gc_kp4_riser3_shelf6 <<'PYG' || echo "  WM GATE FAILED"
import json, sys, glob, numpy as np
d, want = sys.argv[1], sys.argv[2]
m = json.load(open(f"{d}/repeat.json"))
assert m["sim_variant"] == want, (m["sim_variant"], want)
import os as _o
assert int(m["action_repeat"]) == int(_o.environ.get("REPEAT", "4")) and m.get("with_state") is True and abs(float(m["terminal_reward"]) - 1.0) < 1e-9 and m.get("reward_from_tape") is True and m.get("scope") == "full", m
fs = sorted(glob.glob(f"{d}/*.npz")); assert len(fs) == int(m["n_written"]), (len(fs), m.get("n_written"))
z = np.load(fs[0]); assert z["state"].shape == (len(z["reward"]), 17), z["state"].shape
print(f"  [demo-gate] {d.split('/')[-1]}: {len(fs)} tapes, variant {m['sim_variant']}, stride {m['action_repeat']}, with_state, terminal 1.0, total_reward {m['total_reward']}")
PYG
done
