#!/bin/bash
# Submit the 32 CONTACT/SLIDE-phase runs of PHASE_PLAN amendment (m): RLPD + DP x {dH, dDP} x seeds 0-7,
# --nice=8000 (behind the robomimic recovery and dv3 G3), with the registered 150 GB free-space hold.
# usage (from the code clone): bash cluster/submit_contact.sh   [SEEDS="0 1 2 3 4 5 6 7"] [DRY=1]
set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT
SEEDS=${SEEDS:-"0 1 2 3 4 5 6 7"}
FREE=$(df -BG /cluster/tufts/shortlab | tail -1 | awk '{gsub(/G/,"",$4); print $4}')
echo "DISK-GUARD: ${FREE}G free (registered hold: 150G)"
[ "$FREE" -lt 150 ] && { echo "HOLD: under 150G free -- contact runs NOT submitted"; exit 1; }
[ -n "${DRY:-}" ] && { for A in dH dDP; do ARM=$A SEED=0 DRYRUN=1 bash cluster/sbatch_rlpd_contact.sh | tail -2; ARM=$A SEED=0 DRYRUN=1 bash cluster/sbatch_dp_contact.sh | tail -2; done; exit 0; }
R=""; for A in dH dDP; do for S in $SEEDS; do R="$R $(ARM=$A SEED=$S sbatch --parsable -J sl_rlpd_${A}_s$S cluster/sbatch_rlpd_contact.sh)"; done; done
D=""; for A in dH dDP; do for S in $SEEDS; do D="$D $(ARM=$A SEED=$S sbatch --parsable -J sl_dp_${A}_s$S cluster/sbatch_dp_contact.sh)"; done; done
echo "SL_RLPD:$R"; echo "SL_DP:$D"
echo "$(date -Is) SUBMIT contact (amendment m) sl_rlpd:$R sl_dp:$D free=${FREE}G" >> SUBMISSIONS.log
sleep 10; squeue -u "$USER" -o "%j %t %R" | grep -E "^sl_(rlpd|dp)_" | awk '{split($1,a,"_s"); print a[1], $2}' | sort | uniq -c
