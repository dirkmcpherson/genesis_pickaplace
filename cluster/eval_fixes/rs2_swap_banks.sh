#!/usr/bin/env bash
# amendment (j): swap the canonical policy-generated banks to the rebuilt physical-grip versions. GATE: every contact_push
# polE cell (amendments (g)/(g'), lanes 3350666-74, which read the canonical names) must have its _cp metrics.json
# (64 = 32 runs x 2 modes), so no running process can still open the canonical files. Raw originals stay as *_rawgrip.json.
set -euo pipefail
W=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03; B=$W/phase_banks; SUF=${1:-_physgrip_staging}
n=0; for r in s2_r2d_carrycontact_state_dH_bnormclamp1ent5_s{0..7} s2_r2d_carrycontact_state_dDP_bnormclamp1ent5_n21_s{0..7} \
             s2_r2d_contact_state_dH_bnormclamp1ent5_subfloor_s{0..7} s2_r2d_contact_state_dDP_bnormclamp1ent5_n11_s{0..7}; do
  for m in mode sample; do [ -f $W/runs/$r/fresh_eval_polE_${m}_cp/metrics.json ] && n=$((n+1)); done; done
echo "gate: contact_push polE _cp cells complete: $n/64"
[ "${FORCE:-0}" = 1 ] || [ $n -ge 64 ] || { echo "GATE CLOSED (need 64); not swapping"; exit 2; }
for b in polE_place polE_place_dDP polE_contact; do
  cmp -s $B/$b.json $B/${b}_rawgrip.json || { echo "FATAL: $b.json is no longer the raw original (already swapped?)"; exit 3; }
  [ -s $B/$b$SUF.json ] || { echo "FATAL: staging bank $B/$b$SUF.json missing"; exit 4; }
  python3 - "$B/$b$SUF.json" <<'PY'
import json, sys; e = json.load(open(sys.argv[1])); vals = list(e.values())
assert all(v.get("bank_version") == "physgrip_2026-09-07" and 0.0 <= v["grip_cmd"] <= 1.0 and "grip_cmd_raw" in v for v in vals), sys.argv[1]
print(f"  ok {sys.argv[1]}: {len(vals)} entries, bank_version physgrip_2026-09-07, grip in [0,1]")
PY
  mv $B/$b$SUF.json $B/$b.json
  echo "  swapped $b.json <- $b$SUF.json  sha256 $(sha256sum $B/$b.json | cut -c1-16)  (raw kept: ${b}_rawgrip.json $(sha256sum $B/${b}_rawgrip.json | cut -c1-16))"
done
[ -f $B/rebuild_report$SUF.json ] && mv $B/rebuild_report$SUF.json $B/rebuild_report_physgrip_2026-09-07.json
echo "$(date -Is) amendment (j): canonical polE_place/polE_place_dDP/polE_contact swapped to the physical-grip rebuild (raw as *_rawgrip.json); gate $n/64 contact_push polE cells complete" >> $W/COMMANDS.log
echo SWAP-DONE
