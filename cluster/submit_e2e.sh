#!/bin/bash
# Submit the 32 END-TO-END (full task) runs of PHASE_PLAN amendment (n): RLPD + DP x {dH, dDP} x seeds 0-7,
# --nice=9000 (deliberately behind the robomimic recovery, dv3 G3 and the contact-phase runs), with the
# registered 150 GB free-space hold. The datasets must already exist (cluster/e2e_build_sets.sh).
# usage (from the code clone): bash cluster/submit_e2e.sh   [SEEDS="0 1 2 3 4 5 6 7"] [DRY=1]
set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT
SEEDS=${SEEDS:-"0 1 2 3 4 5 6 7"}
FREE=$(df -BG /cluster/tufts/shortlab | tail -1 | awk '{gsub(/G/,"",$4); print $4}')
echo "DISK-GUARD: ${FREE}G free (registered hold: 150G)"
[ "$FREE" -lt 150 ] && { echo "HOLD: under 150G free -- end-to-end runs NOT submitted"; exit 1; }
[ -n "${DRY:-}" ] && { for A in dH dDP; do ARM=$A SEED=0 DRYRUN=1 bash cluster/sbatch_rlpd_e2e.sh | tail -2; ARM=$A SEED=0 DRYRUN=1 bash cluster/sbatch_dp_e2e.sh | tail -2; done; exit 0; }
R=""; for A in dH dDP; do for S in $SEEDS; do R="$R $(ARM=$A SEED=$S sbatch --parsable -J e2e_rlpd_${A}_s$S cluster/sbatch_rlpd_e2e.sh)"; done; done
D=""; for A in dH dDP; do for S in $SEEDS; do D="$D $(ARM=$A SEED=$S sbatch --parsable -J e2e_dp_${A}_s$S cluster/sbatch_dp_e2e.sh)"; done; done
echo "E2E_RLPD:$R"; echo "E2E_DP:$D"
echo "$(date -Is) SUBMIT e2e (amendment n) e2e_rlpd:$R e2e_dp:$D free=${FREE}G git=$(git rev-parse --short HEAD)" >> SUBMISSIONS.log
sleep 10; squeue -u "$USER" -o "%j %t %R" | grep -E "^e2e_(rlpd|dp)_" | awk '{split($1,a,"_s"); print a[1], $2}' | sort | uniq -c
