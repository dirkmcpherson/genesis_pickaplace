# POST-MAINTENANCE COMMANDS (literal; written 2026-08-25 for a low-context operator)
# Context docs if anything is unclear: paper/SESSION_RUNBOOK_2026-08-24.md (final section),
# paper/PREREG_final_round_robin_2026-08-23.md, paper/SESSION_LOG_2026-08-23_cluster.md,
# paper/PAPER_NOTES.md (N5/N6). RULES: run in order; log every command (use ~/bin/ccmd from the
# laptop, or copy the command line into the session log by hand); if a gate prints FATAL or a
# pass bar fails, STOP that item, write what happened in the session log, and move to the next
# item -- do not improvise fixes. Never rebuild an existing matched_* dir in place.

# 0. SETUP (login node)
ssh tufts
cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace
git fetch && git pull --ff-only origin 4dof-cartesian && git log --oneline -1
source ~/.bashrc; conda activate /cluster/tufts/shortlab/jstale02/condaenv/genesis
python baselines/make_eval_ics.py --check                     # must print: MATCHES
python baselines/tests/test_record_demos_contract.py          # must print: ALL OK
python baselines/tests/test_terminal_guard.py                 # must print: ALL OK

# 1. R2D DISENTANGLE (was the horizon the killer? ~12h/job)
cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace
for S in 70 71; do CONFIG=genesis_pick_v5d4c_delta_shaped SEED=$S TIME_LIMIT=400 \
  R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 DEMO_DIR=baselines/demos_v1/r2d_dH_w2 \
  GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/sbatch_r2dreamer.sh; done
# READ: grep R2D-RESULT r2d_train_<jobid>.out   PASS = best-ckpt picked >= 0.5 on >= 1 seed.
# PASS -> the printed BEST_selected.pt is the NEW-WORLD dR2D TEACHER; continue to 2.
# FAIL -> STOP items 2-3's dR2D parts; run item 3 with dH/dDP arms only; note it.

# 2. dR2D_w3 SOURCE + FULL SETS (only if item 1 passed; CKPT = the BEST_selected.pt path from 1)
TEACHER=r2d CKPT=<BEST_selected.pt from item 1> OUTDIR=baselines/demos_v1/dR2D_w3 SHARD_N=4 \
  PARTITION=batch CPUS=4 MEM=12g TIME=2:00:00 \
  EXTRA="--ic-mode demo --ic-from-tape --attempts 3 --mode sample --verify --sim-variant gc_kp4_riser3_shelf6" \
  GENESIS_PICKAPLACE_ROOT=$PWD bash cluster/sbatch_record.sh
# when the rec_dR2D_w3_* jobs finish:
/cluster/tufts/shortlab/jstale02/r2d_venv/bin/python baselines/record_demos.py --teacher r2d --merge --outdir baselines/demos_v1/dR2D_w3
sbatch -p batch -c 4 --mem 16g --time 0:45:00 -J w4build --wrap "source ~/.bashrc; conda activate /cluster/tufts/shortlab/jstale02/condaenv/genesis; cd $PWD; export CUDA_VISIBLE_DEVICES='' GENESIS_PICKAPLACE_ROOT=$PWD; python baselines/make_matched_sets.py --dH baselines/demos_v1/dH_w2 --dDP baselines/demos_v1/dDP_w2 --dR2D baselines/demos_v1/dR2D_w3 --fails-dDP baselines/demos_v1/dDP_w2_fails --out-root baselines/matched_w4 --seed 0 --cap-n 66 --census && for A in dH dDP dR2D; do python baselines/convert_to_lerobot.py baselines/matched_w4/\$A baselines/matched_w4/\$A/lerobot 8 4 none image; done"
# gate: baselines/matched_w4/MATCHED_SETS.json shows equal N and one sim_variant everywhere.

# 3. FILL THE PICK MATRIX on the corrected world (use matched_w4 if item 2 ran, else matched_w3)
# DP missing cells (dR2D x 5 seeds; skip if no dR2D):
for S in 20 21 22 23 24; do ARM=dR2D SEED=$S WAVE=w2final DEMO_ROOT=baselines/matched_w4 \
  SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/sbatch_dp.sh; done
# RLPD: dR2D x {sparse,dense} x seeds 20-23, plus PREREG-count top-ups for dH/dDP (seeds 24-29):
for A in dR2D; do for R in sparse dense; do for S in 20 21 22 23; do ARM=$A SEED=$S REWARD=$R \
  WAVE=w2final DEMO_ROOT=baselines/matched_w4 SIM_VARIANT=gc_kp4_riser3_shelf6 \
  GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/sbatch_rlpd.sh; done; done; done
for A in dH dDP; do for R in sparse dense; do for S in 24 25 26 27 28 29; do ARM=$A SEED=$S REWARD=$R \
  WAVE=w2final DEMO_ROOT=baselines/matched_w3 SIM_VARIANT=gc_kp4_riser3_shelf6 \
  GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/sbatch_rlpd.sh; done; done; done
# READ: grep -h SWEEP-HEADLINE rlpd_*.out ; cat baselines/outputs/dp_w2final/*/sweep/HEADLINE.txt

# 4. WM BLOCK -- THE N5 DECISION CELLS FIRST (dv3; ~17h/job at 1M steps)
# build dreamer dirs (fails arms REQUIRE matched_v2-style fails sets; use matched_w3/w4 + the
# fails arms dir that exists: check `ls baselines/matched_w3` first; if no dDPfails there, build
# them: python baselines/make_matched_sets.py ... --fails-dDP baselines/demos_v1/dDP_w2_fails):
D=/cluster/tufts/shortlab/jstale02/dreamerv3-torch/demonstrations
sbatch -p batch -c 4 --mem 16g --time 1:00:00 -J dv3conv --wrap "source ~/.bashrc; conda activate /cluster/tufts/shortlab/jstale02/condaenv/genesis; cd $PWD; export CUDA_VISIBLE_DEVICES='' GENESIS_PICKAPLACE_ROOT=$PWD; for A in dH dDP dR2D dDPfails dR2DDPfails; do python baselines/rl/to_dreamer_native.py --src baselines/matched_w4/\$A --dst $D/genesis_final_\${A}_r4 --repeat 4 --scope pick --terminal-reward 100 --force; done"
cd /cluster/tufts/shortlab/jstale02/dreamerv3-torch
for A in dR2D dR2DDPfails dDP dDPfails dH; do for R in sparse dense; do for S in 20 21 22 23; do \
  GENESIS_PICKAPLACE_ROOT=/cluster/tufts/shortlab/jstale02/genesis_pickaplace ARM=$A SEED=$S REWARD=$R \
  sbatch sbatch_genesis_final_rr.sh; done; done; done
# READ: grep DV3-RESULT dv3*.out    N5 DECISION: compare dR2D vs dR2DDPfails (and dDP vs dDPfails)
# -- if the fails-included WM cells are flat-or-better where RLPD dropped, that is the paper's
# headline; hand the readout to a full-capability session for the writeup.
# r2dreamer arms: ONLY if item 1 passed -- use the patched sbatch_r2dreamer with DEMOSET arms per
# its header (r2d demo dirs for w4 must be built by the same converter path the r2d agent wired;
# see cluster/patches/r2dreamer_final_rr.patch header notes).

# 5. SMALL REGISTERED FOLLOW-UPS (run after 3-4 are queued; cheap)
# 5a. RLPD share-matched fails arm: STOP-AND-ASK a full-capability session to build it (it needs a
#     new subsampling flag in make_matched_sets; pre-registered in PAPER_NOTES N5 -- do not improvise).
# 5b. dHunpruned_either DP control x3:
cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace
sbatch -p batch -c 4 --mem 16g --time 0:30:00 -J unprep --wrap "source ~/.bashrc; conda activate /cluster/tufts/shortlab/jstale02/condaenv/genesis; cd $PWD; export CUDA_VISIBLE_DEVICES='' GENESIS_PICKAPLACE_ROOT=$PWD; python baselines/record_demos.py --teacher human --merge --outdir baselines/demos_v1/dHunpruned_either && rm -rf baselines/matched_w3/dHunpruned && mkdir -p baselines/matched_w3/dHunpruned && for f in baselines/demos_v1/dHunpruned_either/*.npz; do ln -s $PWD/\$f baselines/matched_w3/dHunpruned/; done && cp baselines/demos_v1/dHunpruned_either/manifest.json baselines/matched_w3/dHunpruned/ && python baselines/convert_to_lerobot.py baselines/matched_w3/dHunpruned baselines/matched_w3/dHunpruned/lerobot 8 4 none image"
# NOTE: dHunpruned_either was recorded in the OLD world (base) BEFORE the world switch -- its
# manifest says sim_variant=base, so it must run with SIM_VARIANT=base (it is an old-world
# preprocessing control) OR be re-recorded in the new world first (preferred):
TEACHER=human SRC=baselines/episodes_pick_phase_all OUTDIR=baselines/demos_v1/dHunpruned_w2 SHARD_N=8 \
  PARTITION=batch CPUS=4 MEM=12g TIME=2:00:00 \
  EXTRA="--ic-from-tape --arrival either --sim-variant gc_kp4_riser3_shelf6" \
  GENESIS_PICKAPLACE_ROOT=$PWD bash cluster/sbatch_record.sh
# 5c. RLPD retune probe (single-knob; run only these four, one seed each, sparse, dH, matched_w3):
for K in "GAMMA=0.99" "UTD=5" "TRAIN_HORIZON=900" "STEPS=200000"; do echo "STOP-AND-ASK: $K needs a
  full-capability session to wire the knob into sbatch_rlpd cleanly -- do not hand-edit"; done

# 6. WHEN RESULTS ARE IN: pull artifacts to the laptop (from the laptop):
#   bash -c 'R=tufts:/cluster/tufts/shortlab/jstale02/genesis_pickaplace; rsync -a --bwlimit=4000 \
#     $R/baselines/matched_w4 ~/workspace/final_rr_artifacts_2026-08-24/; ...' (mirror pull.log phases)
# and hand ALL readouts to a full-capability session for: the results audit (PREREG §11 gate 2),
# PAPER_NOTES N7, and the paper draft. The physical to-do that gates shelf6 vs shelf10 is the
# user's real-rig shelf-height measurement -- record it in PAPER_PLAN when known.
