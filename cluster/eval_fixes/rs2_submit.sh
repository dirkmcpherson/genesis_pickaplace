#!/usr/bin/env bash
# amendment (j): submit the 4 re-score lanes (16 concurrent evals). Requires the swapped canonical banks + a passed smoke.
set -euo pipefail
W=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03; B=$W/phase_banks; cd $W
for b in polE_place_physgrip polE_place_dDP_physgrip polE_contact_physgrip; do python3 -c "
import json,sys; e=json.load(open('$B/$b.json')); assert all(v.get('bank_version')=='physgrip_2026-09-07' for v in e.values()), '$b not rebuilt'; print('bank ok $b', len(e))"; done
bash rs2_make_cells.sh
# Priority order (coordinator 2026-09-08): P1 = the 64 three-learner-table cells (dH + dDP n39 x {polE rebuilt, holdE} x
# {mode, sample}); P2 = the remaining place cells (uncapped machine-63 arm + polEdDP symmetry) so PHASE_RESULTS 2.y sits
# on the rebuilt bank; P3 = carrycontact + contact-after-release, which carry the same raw-grip/unstamped defect.
P=$W/rs2_cells_phase.txt
grep -E "s2_r2d_place_state_(dH_bnormclamp1ent5|dDP_bnormclamp1ent5_n39)_s[0-7] " $P | grep -E " (polE|holdE) " > $W/rs2_cells_p1.txt
grep "s2_r2d_place_state" $P | grep -vxF -f $W/rs2_cells_p1.txt > $W/rs2_cells_p2.txt
grep -v "s2_r2d_place_state" $P > $W/rs2_cells_p3.txt
cat $W/rs2_cells_p1.txt $W/rs2_cells_p2.txt $W/rs2_cells_p3.txt > $W/rs2_cells_ordered.txt
wc -l $W/rs2_cells_p1.txt $W/rs2_cells_p2.txt $W/rs2_cells_p3.txt
# Phase cells: hardware-insensitive (3488 episodes bit-exact across CPU classes) -> any node. 6 lanes x 4 = 24 concurrent.
NL=${NL:-6}
for L in $(seq 0 $((NL-1))); do J=$(sbatch --parsable -J rs2_ph_L$L --qos=preempt -p preempt,batch rs2_rescore.sbatch $W/rs2_cells_ordered.txt $L $NL); echo "phase lane $L -> $J"; echo "$(date -Is) rs2_ph_L$L $J (phase cells, ordered P1/P2/P3, any node)" >> $W/COMMANDS.log; done
# End-to-end cells: pinned to ONE CPU class so the 64 cells form an internally consistent set and the arm-vs-hardware
# confound of EVAL_FIXES 7.6 cannot recur. Slurm feature labels are unreliable (pax001 advertises broadwell but is
# Cascade Lake), so pin by explicit nodelist of model-verified nodes.
E2E_NODES=${E2E_NODES:-pax030,pax033,pax035,pax056,pax064,pax111,pax112}
J=$(sbatch --parsable -J rs2_e2e --nodelist=$E2E_NODES -N 1 rs2_rescore.sbatch $W/rs2_cells_e2e.txt 0 1); echo "e2e lane -> $J (nodes $E2E_NODES)"
echo "$(date -Is) rs2_e2e $J (64 end-to-end cells pinned to $E2E_NODES; hardware.json per cell)" >> $W/COMMANDS.log
