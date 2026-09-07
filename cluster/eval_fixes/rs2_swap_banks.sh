#!/usr/bin/env bash
# amendment (j): point the canonical policy-generated bank names at the rebuilt physical-grip banks (*_physgrip.json --
# the (j) re-score itself already uses those by explicit path, with the sha stamped into every cell, so this step only
# makes the canonical names the banks of record for future consumers, e.g. amendment (h)'s eval_place.py).
# GATE: every contact_push (amendment (g)/(g')) polE cell -- lanes 3350666-74, whose in-flight cell list still names the
# canonical files and whose registered protocol is "banks AS THEY ARE" -- must have written its _cp metrics.json
# (64 = 32 runs x 2 modes), so no running eval can open a canonical file after it changes.
# Raw originals stay as *_rawgrip.json; the rebuilt files stay as *_physgrip.json. FORCE=1 overrides the gate.
set -euo pipefail
W=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03; B=$W/phase_banks; SUF=${1:-_physgrip}
n=0; for r in s2_r2d_carrycontact_state_dH_bnormclamp1ent5_s{0..7} s2_r2d_carrycontact_state_dDP_bnormclamp1ent5_n21_s{0..7} \
             s2_r2d_contact_state_dH_bnormclamp1ent5_subfloor_s{0..7} s2_r2d_contact_state_dDP_bnormclamp1ent5_n11_s{0..7}; do
  for m in mode sample; do [ -f $W/runs/$r/fresh_eval_polE_${m}_cp/metrics.json ] && n=$((n+1)); done; done
echo "gate: contact_push polE _cp cells complete: $n/64"
[ "${FORCE:-0}" = 1 ] || [ $n -ge 64 ] || { echo "GATE CLOSED (need 64); not swapping"; exit 2; }
for b in polE_place polE_place_dDP polE_contact; do
  cmp -s $B/$b.json $B/${b}_rawgrip.json || { echo "FATAL: $b.json is not the raw original (already swapped?)"; exit 3; }
  [ -s $B/$b$SUF.json ] || { echo "FATAL: rebuilt bank $B/$b$SUF.json missing"; exit 4; }
  python3 - $B/$b$SUF.json <<'PY'
import json, sys
e = json.load(open(sys.argv[1])); v = list(e.values())
assert all(x.get("bank_version") == "physgrip_2026-09-07" and 0.0 <= x["grip_cmd"] <= 1.0 and "grip_cmd_raw" in x for x in v), sys.argv[1]
print(f"  ok {sys.argv[1].split('/')[-1]}: {len(v)} entries, bank_version physgrip_2026-09-07, grip in [0,1]")
PY
  cp -p $B/$b$SUF.json $B/$b.json
  echo "  canonical $b.json <- $b$SUF.json  sha256 $(sha256sum $B/$b.json | cut -c1-16)  (raw kept: ${b}_rawgrip.json $(sha256sum $B/${b}_rawgrip.json | cut -c1-16))"
done
echo "$(date -Is) amendment (j): canonical polE_place/polE_place_dDP/polE_contact now = the physical-grip rebuild (raw kept as *_rawgrip.json, rebuilt as *_physgrip.json); gate $n/64 contact_push polE cells complete" >> $W/COMMANDS.log
echo SWAP-DONE
