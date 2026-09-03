# ============================================================================
# TONIGHT'S LAUNCH PLAN (final, 2026-08-25). Supersedes everything below.
# Sources: paper/CRITIQUE_launch_plan_2026-08-25.md (accepted), PAPER_NOTES N5/N7/N8/N9/N10/N11.
# 56 GPU jobs, ~275 GPU-h, ~21 h wall at ~13 concurrent.
# SUBMIT LONG JOBS FIRST (they set the critical path), shorts backfill.
# ============================================================================
#
# STEP 0 -- ZERO GPU, DO ALL OF IT BEFORE SUBMITTING ANYTHING
LAB=/cluster/tufts/shortlab/jstale02; cd $LAB/genesis_pickaplace
git pull --ff-only origin 4dof-cartesian && git log --oneline -1
source ~/.bashrc; conda activate $LAB/condaenv/genesis
python baselines/make_eval_ics.py --check && python baselines/tests/test_record_demos_contract.py && python baselines/tests/test_terminal_guard.py
# 0a. purge the placeholder dR2D provenance row (the DIR was deleted at build time, the ROW survived
#     carrying dDP's sha -- BL-7). Already fixed on the laptop copy; redo here:
python - <<'EOF'
import json
p='baselines/matched_w3/MATCHED_SETS.json'; j=json.load(open(p))
j['sets'].pop('dR2D', None)
j['NOTE_2026-08-25']='placeholder dR2D row purged (was a duplicate of dDP); matched_w3 = dH + dDP only'
json.dump(j, open(p,'w'), indent=1); print('purged'); 
EOF
# 0b. r2d-format demo dirs for the N5 fails arm (dR2D exists already; dR2DDPfails does not):
python baselines/rl/to_dreamer_native.py --src baselines/matched_v2/dR2DDPfails \
  --dst baselines/matched_v2/r2d/dR2DDPfails --repeat 4 --scope pick --terminal-reward 1 --force
ls baselines/matched_v2/r2d/   # expect: dH dDP dR2D dR2DDPfails
# 0c. N7 pruned BC sets + their lerobot datasets (sbatch_dp has no inline build for native arms):
for E in 1e-3 1e-2; do T=$(echo $E | tr -d '.-'); \
  python baselines/make_pruned_bc_set.py --src baselines/matched_w3/dH  --dst baselines/matched_w3/dHallpruned_$T  --eps $E; \
  python baselines/make_pruned_bc_set.py --src baselines/matched_w3/dDP --dst baselines/matched_w3/dDPallpruned_$T --eps $E; done
for A in dHallpruned_1e3 dHallpruned_1e2 dDPallpruned_1e3 dDPallpruned_1e2; do \
  python baselines/convert_to_lerobot.py baselines/matched_w3/$A baselines/matched_w3/$A/lerobot 8 4 none image; done
# 0d. DRYRUN every new arm before any real submission:
for A in dHallpruned_1e3 dDPallpruned_1e3; do DRYRUN=1 ARM=$A SEED=20 WAVE=density DEMO_ROOT=baselines/matched_w3 \
  SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_PICKAPLACE_ROOT=$PWD bash cluster/sbatch_dp.sh | grep -E "FATAL|PROVENANCE|DEMO-SHA"; done
DRYRUN=1 ARM=dR2DDPfails SEED=80 DEMOSET=v2 CONFIG=genesis_pick_v5d4c_delta_shaped TIME_LIMIT=400 \
  GENESIS_PICKAPLACE_ROOT=$PWD bash cluster/sbatch_r2dreamer.sh | grep -E "FATAL|dry\] demo"
#
# STEP 1 -- LONG JOBS (submit first; they define the critical path)
# 1a. N5 WM decision cells: does a world model care about the 8 DP fail tapes that took RLPD
#     0.55 -> 0.15? OLD-world matched_v2 (the same tapes the RLPD numbers were measured on), dense,
#     TIME_LIMIT=400 (the ONLY horizon at which r2d has ever ignited here), R2D_SIM_VARIANT UNSET.
for A in dR2D dR2DDPfails; do for S in 80 81 82 83; do \
  ARM=$A SEED=$S DEMOSET=v2 CONFIG=genesis_pick_v5d4c_delta_shaped TIME_LIMIT=400 \
  GENESIS_PICKAPLACE_ROOT=$PWD sbatch --time=24:00:00 cluster/sbatch_r2dreamer.sh; done; done      # 8 jobs
#     READ-GATE: if dR2D ignites in <2/4 seeds, the fails contrast is a floor effect -- do not read it.
#     REPORT WITH IT: demos are ~3% of the r2d replay ring vs ~36% of every RLPD batch (17x less
#     exposure to the mediator), and adding fails cuts reward density 3.4x. A null without those two
#     numbers is uninterpretable.
# 1b. r2d disentangle: was the 08-24 corrected-world failure (0/2) the WORLD, or the horizon?
for S in 70 71; do CONFIG=genesis_pick_v5d4c_delta_shaped SEED=$S TIME_LIMIT=400 \
  R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 DEMO_DIR=baselines/demos_v1/r2d_dH_w2 \
  GENESIS_PICKAPLACE_ROOT=$PWD sbatch --time=24:00:00 cluster/sbatch_r2dreamer.sh; done            # 2 jobs
#     If it ignites -> the recipe survives the world change AND its BEST ckpt is the corrected-world
#     dR2D teacher (unblocks the whole dR2D column). If not -> the world change broke r2dreamer.
#
# STEP 2 -- SHORT JOBS (backfill; ~3 h each)
# 2a. RLPD seed top-ups: N6's source-parity NULL and N10's vanished spread are BOTH n=4 with
#     per-seed values spanning 0.07-0.73. This takes them to n=10 -- the cheapest claim-strength
#     gain available, and the direct test of N10 explanation (1) (floor compression).
for A in dH dDP; do for R in sparse dense; do for S in 24 25 26 27 28 29; do \
  ARM=$A SEED=$S REWARD=$R WAVE=w2final DEMO_ROOT=baselines/matched_w3 \
  SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/sbatch_rlpd.sh; done; done; done   # 24 jobs
# 2b. DP seed top-ups (N11): the BC source claim now rests on `rnd`, where the gap is 0.05 at n=5.
for A in dH dDP; do for S in 25 26 27 28 29; do \
  ARM=$A SEED=$S WAVE=w2final DEMO_ROOT=baselines/matched_w3 SIM_VARIANT=gc_kp4_riser3_shelf6 \
  GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/sbatch_dp.sh; done; done                                            # 10 jobs
# 2c. N7 action-density control -- PRIMARY READOUT IS `rnd`, NOT `hold` (hold is ceilinged at 14/15,
#     N11). Seeds 20-22 pair one-to-one with the baseline for PREREG §7's paired analysis.
for A in dHallpruned_1e3 dHallpruned_1e2 dDPallpruned_1e3 dDPallpruned_1e2; do for S in 20 21 22; do \
  ARM=$A SEED=$S WAVE=density DEMO_ROOT=baselines/matched_w3 SIM_VARIANT=gc_kp4_riser3_shelf6 \
  GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/sbatch_dp.sh; done; done                                            # 12 jobs
#
# STEP 3 -- ~1 GPU-HOUR, decides whether N8 is worth ~100 more
#     dv3_interrogate.py has never run against a real checkpoint (5 API-GUESS sites) and its
#     value_reach returns None on a flat/untrained value head -- the modal N8 outcome. Calibrate on
#     an existing checkpoint FIRST:
python analysis/dv3_interrogate.py --checkpoint <an existing dv3 ckpt, e.g. rr_dH_s2 latest.pt> \
  --logdir <its logdir> --configs genesis_pixels genesis_pick_msrecipe \
  --demodir $LAB/dreamerv3-torch/demonstrations/genesis_pick_msr_delta25_r4 --n-episodes 8 \
  --device cuda --dreamer-root $LAB/dreamerv3-torch --out /tmp/interrogate_calib.json
#     Then grep API-GUESS in the tool and check each against what it printed.
#
# NOT TONIGHT, and why:
#   N8 (repeat-1 vs repeat-4): needs a CPU re-record of dH at action_repeat=1 WITH the camera rig
#     (contract-v1 tapes carry images per DECISION only -- verified), and sbatch_genesis_final_rr.sh
#     hard-codes ACTREP with no EXTRA_ARGS, so the arms would silently be 3 copies of A sharing one
#     RUNDIR, and B/C would collide in the registry (discount is not in REG_KNOBS).
#   Gripper-model changes: would invalidate every recorded set.
#   Corrected-world dR2D cells: no teacher until STEP 1b reports.
#   Full-task / place-scope: user scoped the program to pick phase for now.
#
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

# 4b. WM CLOCK/HORIZON INVESTIGATION (N8; run alongside item 4 -- dv3, dH only, 2 seeds each)
#     A = current recipe; B = native 30 Hz with the discount horizon matched in physical time;
#     C = native 30 Hz with gamma unchanged (separates clock from horizon).
cd /cluster/tufts/shortlab/jstale02/dreamerv3-torch
for S in 40 41; do GENESIS_PICKAPLACE_ROOT=/cluster/tufts/shortlab/jstale02/genesis_pickaplace \
  ARM=dH SEED=$S REWARD=sparse STEPS=1000000 WAVE=n8A sbatch sbatch_genesis_final_rr.sh; done          # A
for S in 40 41; do GENESIS_PICKAPLACE_ROOT=/cluster/tufts/shortlab/jstale02/genesis_pickaplace \
  ARM=dH SEED=$S REWARD=sparse STEPS=250000 ACTREP=1 EXTRA_ARGS="--discount 0.99925" WAVE=n8B sbatch sbatch_genesis_final_rr.sh; done  # B
for S in 40 41; do GENESIS_PICKAPLACE_ROOT=/cluster/tufts/shortlab/jstale02/genesis_pickaplace \
  ARM=dH SEED=$S REWARD=sparse STEPS=250000 ACTREP=1 WAVE=n8C sbatch sbatch_genesis_final_rr.sh; done   # C
# CHECK FIRST: does sbatch_genesis_final_rr.sh expose ACTREP and EXTRA_ARGS? grep its header. If not,
# STOP-AND-ASK -- do not hand-edit. Arms B/C also need a STRIDE-1 demo dir: the recorder tapes carry a
# per-sim-step sub-tape (sim_states/sim_actions), so it is derivable without re-recording -- that
# builder does not exist yet: STOP-AND-ASK.
# PRIMARY READOUT is NOT ignition (dv3 ignition is marginal, N4) but the diagnostic:
for D in <each n8 logdir>; do python /cluster/tufts/shortlab/jstale02/genesis_pickaplace/analysis/dv3_interrogate.py \
  --checkpoint $D/latest.pt --logdir $D --configs genesis_pixels genesis_pick_msrecipe genesis_final_rr \
  --demodir <the demo dir that run used> --n-episodes 8 --device cuda \
  --dreamer-root /cluster/tufts/shortlab/jstale02/dreamerv3-torch --out $D/interrogate.json; done
# COMPARE value_head.value_reach_sim_steps across A/B/C -> answers N8. On FIRST use, verify the five
# `API-GUESS` comments in dv3_interrogate.py (grep API-GUESS); the tool warns rather than guessing wrong.

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

# 5d. ACTION-DENSITY CONTROL (N7, pre-registered; BC ONLY -- never point an RLPD/WM arm at these)
cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace
for E in 1e-3 1e-2; do T=$(echo $E | tr -d '.-'); \
  python baselines/make_pruned_bc_set.py --src baselines/matched_w3/dH  --dst baselines/matched_w3/dHallpruned_$T  --eps $E; \
  python baselines/make_pruned_bc_set.py --src baselines/matched_w3/dDP --dst baselines/matched_w3/dDPallpruned_$T --eps $E; done
for A in dHallpruned_1e3 dHallpruned_1e2 dDPallpruned_1e3 dDPallpruned_1e2; do \
  python baselines/convert_to_lerobot.py baselines/matched_w3/$A baselines/matched_w3/$A/lerobot 8 4 none image; done
for A in dHallpruned_1e3 dHallpruned_1e2 dDPallpruned_1e3 dDPallpruned_1e2; do for S in 30 31 32; do \
  ARM=$A SEED=$S WAVE=density DEMO_ROOT=baselines/matched_w3 SIM_VARIANT=gc_kp4_riser3_shelf6 \
  GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/sbatch_dp.sh; done; done
# NOTE: sbatch_dp's ARM table may not know these names -- if it prints FATAL on the ARM case,
# STOP-AND-ASK (a full-capability session adds the arms; do not hand-edit the gate).
# READ: cat baselines/outputs/dp_density/*/sweep/HEADLINE.txt ; compare hold vs dH_DP 0.90 (N7).

# 6. WHEN RESULTS ARE IN: pull artifacts to the laptop (from the laptop):
#   bash -c 'R=tufts:/cluster/tufts/shortlab/jstale02/genesis_pickaplace; rsync -a --bwlimit=4000 \
#     $R/baselines/matched_w4 ~/workspace/final_rr_artifacts_2026-08-24/; ...' (mirror pull.log phases)
# and hand ALL readouts to a full-capability session for: the results audit (PREREG §11 gate 2),
# PAPER_NOTES N7, and the paper draft. The physical to-do that gates shelf6 vs shelf10 is the
# user's real-rig shelf-height measurement -- record it in PAPER_PLAN when known.
