#!/usr/bin/env bash
# amendment (j): submit the 4 re-score lanes (16 concurrent evals). Requires the swapped canonical banks + a passed smoke.
set -euo pipefail
W=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03; B=$W/phase_banks; cd $W
for b in polE_place polE_place_dDP polE_contact; do python3 -c "
import json,sys; e=json.load(open('$B/$b.json')); assert all(v.get('bank_version')=='physgrip_2026-09-07' for v in e.values()), '$b not rebuilt'; print('bank ok $b', len(e))"; done
bash rs2_make_cells.sh
for L in 0 1 2 3; do J=$(sbatch --parsable -J rs2_rescore_L$L rs2_rescore.sbatch $W/rs2_cells.txt $L 4); echo "lane $L -> $J"; echo "$(date -Is) rs2_rescore_L$L $J (amendment j re-score, 4 lanes x 4 evals, cells rs2_cells.txt)" >> $W/COMMANDS.log; done
