#!/bin/bash
# Scheduled readout for the 08-29/31 blackout: one file per firing, so the post-blackout read is ONE file.
# Submit: sbatch -p batch -c 1 --mem 2g -t 00:30:00 --begin=now+20hours cluster/harvest_readout.sh
#SBATCH -J harvest
#SBATCH --output=/cluster/tufts/shortlab/jstale02/genesis_pickaplace/harvest_%j.out
GPR=/cluster/tufts/shortlab/jstale02/genesis_pickaplace; DV3=/cluster/tufts/shortlab/jstale02/dreamerv3-torch
PY=/cluster/tufts/shortlab/jstale02/condaenv/genesis/bin/python
OUT=$GPR/paper/harvest_$(date +%Y-%m-%d_%H%M).md; cd $GPR
{
echo "# Harvest $(date) job=$SLURM_JOB_ID"; echo; echo "## queue"; squeue -u $USER -h -o "%t %j %r" | sort | uniq -c
echo; echo "## sacct last 48h (non-pending)"; sacct -u $USER -S $(date -d "-48 hours" +%Y-%m-%dT%H:%M) -X -n -o JobID,JobName%14,State%12,Elapsed | grep -v PENDING
echo; echo "## pack rc"; grep -aH "rc=" r2d_pack_*.out $DV3/dv3_pack_*.out 2>/dev/null | grep -v "rc=0" | tail -20
echo; echo "## RLPD headlines (all .out, newest 60)"; for f in $(ls -t rlpd_*.out | head -60); do n=$(grep -m1 "^== RLPD" $f | awk '{print $3}'); ts=$(grep total_timesteps $f | tail -1 | awk '{print $4}'); mc=$(grep critic_loss $f | awk '{print $4}' | sort -g | tail -1); wd=$(grep -c WATCHDOG $f); hl=$(grep -a "SWEEP-HEADLINE" $f | sed 's/.*selected=/selected=/;s/ node=.*//'); sel=$(grep -a "tag=selected" $f | grep -o "hold=[^ ]* rnd=[^ ]*"); echo "$n steps=$ts maxCL=$mc wd=$wd ${hl:-$sel}"; done
echo; echo "## r2dreamer (train .out newest 24)"; for f in $(ls -t r2d_train_*.out | head -24); do echo "$f $(grep -m1 -ao 'arm=[^ ]*' $f) $(grep -a 'train/val' $f | tail -1 | grep -o '^\[[0-9]*\]\|train/val [0-9.-]*' | tr '\n' ' ') $(grep -a 'R2D-RESULT\|rc=1\|FATAL' $f | tail -1 | cut -c1-140)"; done
echo; echo "## r2d re-scores"; for f in $(ls -t baselines/outputs/n12_rescore/*.log 2>/dev/null | head -400); do grep -a "^RESCORE-RESULT" $f; done | sort -u | tail -40
echo; echo "## dv3"; for d in $DV3/logs_cluster/genesis_final/* $DV3/logs_cluster/genesis_touchgoal/*; do [ -d $d ] || continue; m=$(ls -t $d/*/metrics.jsonl 2>/dev/null | head -1); e=$(grep -a "Traceback\|FATAL" $d/run.log 2>/dev/null | head -1 | cut -c1-70); [ -n "$m" ] && $PY - "$m" "$(basename $d)" "$e" <<'PYEOF'
import json,sys
rows=[json.loads(l) for l in open(sys.argv[1]) if l.strip()]
s=[(r["step"],round(r["train_success_rate"],2)) for r in rows if "train_success_rate" in r]; v=[(r["step"],round(r["value_mean"],1)) for r in rows if "value_mean" in r]
ev=[(r["step"],round(r["eval_success_rate"],2)) for r in rows if "eval_success_rate" in r]
print(sys.argv[2],"step",rows[-1].get("step"),"succ last4",s[-4:],"max",max([x for _,x in s] or [0]),"val",v[-1:],"eval",ev[-3:],sys.argv[3])
PYEOF
done
echo; echo "## dv3 launcher headlines"; grep -aH "DV3-HEADLINE\|== selected checkpoint\|FATAL" $DV3/slurm-*.out $DV3/dv3_train_*.out 2>/dev/null | tail -20
} > $OUT 2>&1
echo "wrote $OUT"
