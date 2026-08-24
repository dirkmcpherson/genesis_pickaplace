# SESSION RUNBOOK — cluster session 2026-08-24 (36 h window before shutdown)

Scope agreed 08-23 (user): **prerequisites + full DP matrix + RLPD subset** (3 sources ×
sparse+dense × 4 seeds = 24 jobs); pilots fall back per PREREG §8 and continue; all WM
(r2dreamer/dv3) work deferred to the cluster's return; dDP teacher = the new DP-r4 dH
pilot (median-seed rule). Spec: paper/PREREG_final_round_robin_2026-08-23.md. Code:
commit d53a5b1 (branch 4dof-cartesian).

Who does what: if the assistant gets SSH to the cluster (VPN), it runs Phases 0–3 and 5
(CPU work, set building, census, submission commands prepared) and the user only launches
GPU jobs and makes the decision calls marked ◆. Without SSH the user runs everything below
in order. Every path marked **[VERIFY]** must be checked live before use (documented
paths may have drifted).

Conventions: `$LAB=/cluster/tufts/shortlab/jstale02` [VERIFY]; repo `$LAB/genesis_pickaplace`
[VERIFY]; genesis+lerobot conda env `$LAB/condaenv/genesis` (`conda activate`, never
conda-install into it) [VERIFY]; r2dreamer venv `$LAB/r2d_venv` [VERIFY]; r2dreamer
champion `CHAMPION_1576820.pt` — location on the cluster UNKNOWN (runs/ were excluded
from the rsync in R2DREAMER_CLUSTER.md; if absent, the dR2D recording runs on the devbox
where `~/workspace/r2dreamer/runs/pick_delta25d4_s0/CHAMPION_1576820.pt` lives, and the
tapes rsync to the cluster) [VERIFY]; gen-0 DP teacher (only needed for the fallback)
`ouroboros/<TAG>/gen0/dp/checkpoints/last/pretrained_model` [VERIFY TAG]; human stride-1
source `baselines/episodes_pick_phase_dppruned` (66 pruned success demos) and
`baselines/episodes_pick_phase_all` (unpruned) [VERIFY counts]. Outputs go under
`baselines/demos_v1/` (recordings) and `baselines/matched_v1/` (matched sets) — datasets
travel by rsync only, never git.

Timeline (wall, from session start T0; GPU cap assumed ~13 concurrent):
T0–T0+1h Phase 0–1 · T0+1h–T0+2h Phase 2 (dH re-record, CPU) · T0+1h–T0+4.5h Phase 3a
(DP-r4 pilots, GPU, overlaps) · T0+4.5h–T0+6h Phase 3b/3c (teacher harvests, CPU) ·
T0+6h Phase 4 (sets + census) · T0+6.5h Phase 5 launches (DP 15, RLPD 24 → ~3 h each,
≤13 at a time → done by ~T0+18h) · T0+18h–T0+30h readout, re-queue anything that
crashed, pull results off the cluster · T0+36h shutdown. Slack ≈ 6 h.

---

## Phase 0 — verify environment & paths (≈15 min)

```bash
cd $LAB/genesis_pickaplace && git fetch && git checkout 4dof-cartesian && git pull && git log --oneline -1   # expect d53a5b1 or later
conda activate $LAB/condaenv/genesis
python -c "import genesis, lerobot, torch; print(genesis.__version__, torch.__version__)"
python baselines/make_eval_ics.py --check            # eval_ics.json MATCHES
python baselines/tests/test_record_demos_contract.py && python baselines/tests/test_terminal_guard.py
ls baselines/episodes_pick_phase_dppruned | wc -l     # expect 66
ls baselines/episodes_pick_phase_all | wc -l          # expect ~91-93
ls ouroboros/*/gen0/dp/checkpoints/last/pretrained_model 2>/dev/null   # gen-0 DP teacher (fallback only)
find $LAB -name 'CHAMPION_1576820.pt' 2>/dev/null | head   # r2d champion on cluster? [decides where dR2D records]
for S in rlpd dp; do DRYRUN=1 ARM=dH SEED=0 bash cluster/sbatch_$S.sh | tail -3; done
```
◆ If the champion is not on the cluster: decide devbox-side dR2D recording (Phase 3c-alt).

## Phase 1 — recorder smoke + negative control (CPU node, ≈20 min)

Purpose: prove the contract-v1 recorder builds the env, renders the rig, writes valid
tapes, and that a random teacher keeps ≈0 (keep-predicate sanity).
```bash
srun -p batch -c 8 --mem 16g --time 1:00:00 --pty bash     # [VERIFY partition]
conda activate $LAB/condaenv/genesis
python baselines/record_demos.py --teacher random --ic-mode demo --n 10 --seed 0 \
   --outdir baselines/demos_v1/_smoke_negctl
python baselines/record_demos.py --teacher human --src baselines/episodes_pick_phase_dppruned \
   --uids 232 233 242 --outdir baselines/demos_v1/_smoke_dH
```
Pass: negctl kept 0 (≤1); the 3 human demos re-earn the pick (≥2/3), manifest shows
dilation ≈0.7–1.5; `validate_tape` errors = none. Fail → STOP and read the traceback
(API guesses listed in record_demos.py header comments: `rig_obs`, `tool_pos`,
`_calib_tool_offset`, `info['picked']/['tipped']`).

## Phase 2 — dH re-record at repeat 4 (CPU, 16-way, ≈20–40 min)

Purpose: the human set recorded-as-executed in the learners' MDP (PREREG §3.1/§4.2).
```bash
for K in $(seq 0 15); do
  sbatch -p batch -c 2 --mem 8g --time 2:00:00 --wrap "conda activate $LAB/condaenv/genesis && \
    python baselines/record_demos.py --teacher human --src baselines/episodes_pick_phase_dppruned \
    --ic-mode demo --shard-idx $K --shard-n 16 --outdir baselines/demos_v1/dH --seed 0"
done
# when all 16 finish:
python baselines/record_demos.py --merge --outdir baselines/demos_v1/dH
python analysis/characterize_demo_sets.py dH=baselines/demos_v1/dH --out baselines/demos_v1/census_dH.md
```
Pass bar (PREREG §8): **≥50/66 survive**. 45–49: lower `--tol` / raise `--dilation-cap`
and rerun the failures only (`--uids ...`). <45 ◆ fallback: DP at stride 1 on the same
episodes (sim_* sub-tape) — log an amendment in PREREG; RLPD still uses the survivors.
Also re-record the unpruned set for the dHunpruned_DP control (same command, `--src
baselines/episodes_pick_phase_all --outdir baselines/demos_v1/dHunpruned`) — low priority,
run it whenever CPU is idle.

## Phase 3a — DP-r4 pilot on dH (GPU, 2 seeds, ≈3 h) — starts as soon as Phase 2 finishes

Purpose: positive control for DP at the new clock AND the dDP teacher (median-seed rule).
Needs the dH lerobot dataset first (fps 7.5):
```bash
mkdir -p baselines/matched_v1_pilot/dH && ln -s $PWD/baselines/demos_v1/dH/*.npz baselines/matched_v1_pilot/dH/ 2>/dev/null
cp baselines/demos_v1/dH/manifest.json baselines/matched_v1_pilot/dH/
python baselines/convert_to_lerobot.py baselines/matched_v1_pilot/dH baselines/matched_v1_pilot/dH/lerobot 8 4 none image   # fps auto 7.5 (contract v1); if lerobot rejects float fps use LEROBOT_FPS=8 and note it
for S in 0 1; do ARM=dH SEED=$S WAVE=pilot DEMO_ROOT=baselines/matched_v1_pilot sbatch cluster/sbatch_dp.sh; done
```
Read: `DP-HEADLINE arm=dH seed=S ... sel=a/15 hold=b/15 rnd=c/30`. Pass bar: **selected
checkpoint ≥0.5 on sel** for at least one seed (prior stride-1 dHpruned 0.62). Teacher =
the seed whose sel score is the median (with 2 seeds: the lower one — pre-registered
choice to avoid best-seed bias; write it in the manifest). Fail ◆: DP stays stride 1
(fallback); dDP teacher = gen-0 stride-1 DP at hold-4 (Phase 3b with its ckpt); amendment.

## Phase 3b — dDP recording (CPU, ≈1 h at 16-way; needs Phase 3a's teacher)

```bash
CK=baselines/outputs/dp_pilot/dH_DP-pilot_s<S>/checkpoints/<selected step>/pretrained_model   # [from DP-HEADLINE]
# yield pilot first (30 rollouts): PREREG §8 bar = ≥50% of the teacher's stride-1 yield (gen-0 was 19.7%; new teacher unknown -> bar = kept ≥ 0.5*sel_rate*30)
python baselines/record_demos.py --teacher dp --checkpoint $CK --ic-mode demo --n 30 --mode sample --attempts 1 --verify --seed 0 --outdir baselines/demos_v1/_pilot_dDP
# full harvest, target ≥66 successes (shard like Phase 2):
... --teacher dp --checkpoint $CK --ic-mode demo --target-kept 70 --mode sample --attempts 3 --verify --outdir baselines/demos_v1/dDP --fails-outdir baselines/demos_v1/dDP_fails
python baselines/record_demos.py --merge --outdir baselines/demos_v1/dDP
```
Record in the manifest: teacher ckpt, sel rate, first-attempt success rate on harvest ICs.

## Phase 3c — dR2D recording (CPU, needs the r2dreamer venv; ≈30 min)

```bash
source $LAB/r2d_venv/bin/activate   # [VERIFY]; record_demos imports genesis from the same interpreter -> the r2d venv must have genesis (it did for harvest_champion_demos)
python baselines/record_demos.py --teacher r2d --checkpoint <CHAMPION_1576820.pt> --ic-mode demo --n 5 --mode mode --verify --outdir baselines/demos_v1/_smoke_dR2D   # expect ~5/5
python baselines/record_demos.py --teacher r2d --checkpoint <CHAMPION> --ic-mode demo --target-kept 70 --mode sample --attempts 3 --verify --outdir baselines/demos_v1/dR2D --fails-outdir baselines/demos_v1/dR2D_fails
```
3c-alt (champion only on the devbox): run the same two commands on the devbox with
`/home/j/workspace/r2dreamer/.venv/bin/python`, then `rsync -a baselines/demos_v1/dR2D* <cluster>:$LAB/genesis_pickaplace/baselines/demos_v1/`.
Smoke yield < 4/5 → the champion acting through FullTaskEnv's integrator differs from
GenesisPick's ◆ (B1 API note); investigate before harvesting.

## Phase 4 — matched sets + census (CPU, ≈15 min)

```bash
python baselines/make_matched_sets.py --dH baselines/demos_v1/dH --dDP baselines/demos_v1/dDP --dR2D baselines/demos_v1/dR2D \
   --fails-dDP baselines/demos_v1/dDP_fails --out-root baselines/matched_v1 --seed 0 --cap-n 66 --census
# lerobot datasets for DP (one per source; fps 7.5):
for A in dH dDP dR2D; do python baselines/convert_to_lerobot.py baselines/matched_v1/$A baselines/matched_v1/$A/lerobot 8 4 none image; done   # layout is FLAT: $DEMO_ROOT/<ARM>/*.npz + manifest.json (+ lerobot/), exactly what sbatch_rlpd/sbatch_dp gate on
cat baselines/matched_v1/MATCHED_SETS.json      # N, shas, ic histogram
```
Read the census table (`baselines/matched_v1/census.md`): N equal across sources,
contract rows populated, stride table blank-by-construction, dilation for dH, fail arms
share ≤0.30. ◆ If N < 40 decide: accept N or re-harvest to raise the minimum.
Commit ONLY `MATCHED_SETS.json` + census.md into paper/ (no npz in git).

## Phase 5 — launches (GPU)

Order: DP full matrix first (short), then RLPD.
```bash
# DP: 3 sources x 5 seeds (+ dHunpruned x3 when its set exists)
for A in dH dDP dR2D; do for S in 10 11 12 13 14; do ARM=$A SEED=$S WAVE=final sbatch cluster/sbatch_dp.sh; done; done
# RLPD subset: 3 sources x {sparse,dense} x 4 seeds  (seeds 10-13, new values — disjoint from history)
for A in dH dDP dR2D; do for R in sparse dense; do for S in 10 11 12 13; do
  ARM=$A SEED=$S REWARD=$R WAVE=final sbatch cluster/sbatch_rlpd.sh; done; done; done
```
Each job prints REGISTRY-OK/REGISTERED at start, `DEMO-SHA`, K=5 `SWEEP-RESULT ... sel=`,
then `SWEEP-HEADLINE` / `DP-HEADLINE`; wandb project genesis_paper, runs
`{ARM}_RLPD-final_s{S}[-dense]`, `{ARM}_DP-final_s{S}`. Before the flood: run ONE
RLPD job with `STEPS=3000` and ONE DP job with `STEPS=2000 WAVE=smoke` to see the
full pipeline end-to-end (ckpt dirs, sweep, headline) — ≈30 min, worth it.
If a pilot fell back (Phase 3a/2), set `ACTION_REPEAT=1 DEMO_FORMAT=...` per the
amendment and say so in WAVE (e.g. WAVE=final-stride1).

## Phase 6 — before shutdown (T0+30h → T0+36h)

- Results: `grep -h "SWEEP-HEADLINE\|DP-HEADLINE" *.out > paper/final_rr_headlines_$(date +%F).txt`; wandb has the same.
- Pull off the cluster (persistent storage): `baselines/matched_v1/` (sets + shas — the
  paper's demo data), `baselines/demos_v1/*/manifest.json`, the selected DP checkpoints
  (small), the RLPD `ckpt_*` dirs for igniting seeds, all `.out`, `cluster/RUN_REGISTRY.jsonl`.
- Re-queue any crashed job once (registry will refuse exact repeats: add `DUPLICATE_OK=crash`).
- Leave a note in paper/ROUND_ROBIN_RUNNING-style: what launched, seeds, shas, what
  fell back.

## Decision points summary (◆)
1. Champion location → where dR2D records (Phase 0).
2. dH survivors <50 → tol/dilation retry; <45 → DP stride-1 fallback (Phase 2).
3. DP-r4 pilot <0.5 → DP stride 1 + gen-0 teacher (Phase 3a).
4. dR2D smoke <4/5 → integrator investigation (Phase 3c).
5. N < 40 → accept or re-harvest (Phase 4).

---
# POST-MAINTENANCE WM LAUNCHES (r2dreamer; prepared 2026-08-24, fork session)

Prereqs already in place on the cluster: r2dreamer patched (cluster/patches/r2dreamer_final_rr.patch:
shaping gamma = 1-1/horizon = 0.997 + phi(terminal)=0; demo_prefill native-stride stamp support;
eval_genesis --ic-file/--ic-set/--ic-index); native stride-4 demo dirs `baselines/matched_v2/r2d/{dH,dDP,dR2D}`
(56 each, terminal reward 1.0, repeat.json-stamped, gated by sbatch DEMOSET=v2). NOTE: 8 dR2D tapes pay +2
(picked+placed grant in the terminal window -- recorded from the env, consistent with online; footnote in captions).

CORRECTED-WORLD NOTE (user decision 08-24): the program's world is now sim variant
`gc_kp4_riser3_shelf6`; every launch below MUST set R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 and use
demo dirs recorded in that world (the manifest/gate enforces the match; the old-world matched_v2
r2d dirs will FAIL the gate once R2D_SIM_VARIANT is set -- that is intended). Rebuild the r2d dirs
from the new-world sets with to_dreamer_native --terminal-reward 1 when the main session announces
them. NOTE for the set builder: to_dreamer_native does not yet propagate sim_variant into
repeat.json -- the gate falls back to the SOURCE manifest, which carries it.

Pilot gate first (if the 08-24 pilot did not run or failed):
    for S in 60 61; do CONFIG=genesis_pick_v5d4c_delta_shaped ARM=dH SEED=$S DEMOSET=v2 R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 STEPS=2e6 sbatch cluster/sbatch_r2dreamer.sh; done
    # bar (PREREG §8 + amendment): >=1 seed ignites with best-ckpt >=0.5; 2e6 not 3M (time-boxed; ignition window was 0.7-1M)

Main r2d block (after the pilot passes; seeds 60-65, sparse+dense, ~14-18 h/job at 3M or ~10-12 h at 2e6):
    for A in dH dDP dR2D; do for R in "" _shaped; do for S in 60 61 62 63 64 65; do
      CONFIG=genesis_pick_v5d4c_delta$R ARM=$A SEED=$S DEMOSET=v2 R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 STEPS=3e6 sbatch cluster/sbatch_r2dreamer.sh
    done; done; done          # 36 jobs; REWARD auto from CONFIG (*_shaped -> dense)

dv3 (queued LAST per PREREG): apply cluster/patches/dreamerv3-torch-genesis_final_rr.patch to
$LAB/dreamerv3-torch (patch -p1 --dry-run first), build its stride-4 sets with to_dreamer_native
--terminal-reward 100 (dv3 convention; dirs baselines/matched_v2/dv3/<ARM>), one 2e4 smoke, then
the sbatch_genesis_final_rr.sh block (3 sources x sparse+dense x 4 seeds, 1M steps).

dR2D TEACHER NOTE (corrected world): the old champion/s51-BEST are OLD-WORLD policies; the
new-world dR2D teacher = the BEST checkpoint of the 08-24 corrected-world pilot (jobs 2830979/80)
if it ignites, else a post-maintenance retrain. Rebuild r2d demo dirs from the new-world matched
sets with to_dreamer_native --terminal-reward 1 (r2d) / --terminal-reward 100 (dv3).

DECISION GATE before any WM launch: tier-2 world (arm + shelf fix, paper/real2sim_follower_lab_2026-08-23.md
§7/§9). If adopted, teachers retrain first and ALL demo sets re-record in the new world -- do not run the WM
block twice. The user's real-rig shelf-height measurement decides shelf6 vs shelf10.
