# WM BEST-of-5 re-selection (AUDIT_approach_2026-09-02 finding 10) — frozen corrected-world r2dreamer block, n=8v8

**STATUS: JOBS SUBMITTED, RESULTS PENDING** (192 CPU jobs on `-p batch`; this box lost its Tufts DNS path at ~17:40
on 09-02 — `login-prod.pax.tufts.edu` is an internal-only name — so the readout below stops at the first cell; §5 will
be completed when the cluster is reachable again. Everything in §1–§4 is settled.)

Scope: `pick_v5d4c_delta_shaped_{dH,dDP}_s80–87` (16 runs, sim variant `gc_kp4_riser3_shelf6`, dense reward, 3M sim
steps, packed). Finding 10: the WM "BEST" of record is the max of in-job `sel` over EVERY `latest.pt` write
(K = 29–58 snapshots), while DP/RLPD select over exactly 5 archived checkpoints. Repair: re-select among ONLY the five
fraction checkpoints the launcher kept (nearest recorded step to 20/40/60/80/100 % of STEPS, `sbatch_r2dreamer.sh`
~:489-503), fresh process, hold-15 + rnd-30 from the frozen `baselines/eval_ics.json` (the block's IC file, audit f25).

## 0. Two things found while building the repair (both bigger than finding 10 for this block)

**(a) Every r2d RESCORE-RESULT of record was scored in the BASE world, not the corrected world.** The r2dreamer
adapter picks the Genesis world from the env var `R2D_SIM_VARIANT` (`$LAB/r2dreamer/envs/genesis.py:135`, default
`base`; `baselines/sim_variant_hook.py`). `sbatch_r2dreamer.sh:215` exports it for training and for the in-job evals;
the runs' `run.log` (6 lines per pass = the 6 train envs) and `eval.log` carry the adapter's
`[sim-variant] gc_kp4_riser3_shelf6: kp=[800,800,600,400,240,240] gc=1.0 riser=0.03` line. `cluster/r2d_rescore.sh`
never sets it and the SESSION_LOG submit lines for the block (`n12_rescore/*_W3_*`, 08-29 and 09-01 12:16) pass only
`GENESIS_PICKAPLACE_ROOT RUNDIR SET OUT TAGSUF PAR` → **0 of the 2289 per-episode logs under
`baselines/outputs/n12_rescore/` contain a `[sim-variant]` line** (`grep -la "sim-variant" .../n12_rescore/*.log | wc -l`
= 0), including all 32 `*_s8?_W3_{hold,rnd}` logs behind RESULTS §3.1's n=8v8 table (BEST rnd dH 0.554 v dDP 0.308;
ignition 6/8 v 3/8). The `_W3` suffix was a label only. Policies trained at kp×4 / gravity-comp / riser were therefore
evaluated at base kp / no riser. The new scorer exports the variant and refuses (never averages) any episode whose log
lacks the line; the smoke episode's log carries it (kp=[800,...], job 3177410).

**(b) Restarts made the "F100" checkpoint of five runs a ~112k-step snapshot.** Runs s84–87 were OOM-killed in their
4-seed packs (3025535 dH after 11:58 h, 3025536 dDP after 13:22 h) and resubmitted as 2-seed packs (3085547/48 dH,
3115514/15 dDP). train.py resume is a warm restart with the step counter re-running the full budget
(`sbatch_r2dreamer.sh:115-123`). Some seeds had already FINISHED pass 1 (dH s84/s87, dDP s85/s86/s87: K=58, a second
full 3M pass = 6M sim steps total); the others restarted at ~0.9M (dH s85/s86, dDP s84: K=37, ≈3.9M total). The
archiver names snapshots by the LAST `metrics.jsonl` step, so the first snapshot after a restart was named
`ckpt_2999xxx.pt` (stale row) while its internal `step` is 112–118k; the fraction rule then keeps it as "100 %" and the
real end-of-pass-1 snapshot was pruned (survives only where it was best-2/newest). Budget is not matched across seeds
of this block: 3M (s80–83), ≈3.9M (dH s85/86, dDP s84), 6M (dH s84/87, dDP s85/86/87).

## 1. Inventory (step 1 of the task): all 16 runs have BEST_selected.pt + the 5 fraction checkpoints

Columns: K = rows of `ckpt_scores.tsv`; "restarts" = step drops in the tsv; BEST-of-K = the file `BEST_selected.pt`
is byte-identical to (md5); sel = in-job seed-0 15-episode sampled score (corrected world, concurrent with training);
p1/p2 = which pass the snapshot came from; int = the checkpoint's own `step` field.

| arm | seed | K (rows) | restarts (row-step drops) | BEST-of-K = BEST_selected.pt (sel, pass, internal step) | 5 fraction checkpoints kept by the launcher rule (sel score, pass, internal step) |
|---|---|---|---|---|---|
| dDP | 80 | 29 | 0 (none) | 2922490 (sel 0.00, p1, int 2917828) | F20=625696 (sel 0.00, p1, int 617828); F40=1223405 (sel 0.00, p1, int 1217824); F60=1823892 (sel 0.00, p1, int 1817828); F80=2425316 (sel 0.00, p1, int 2417836); F100=2922490 (sel 0.00, p1, int 2917828) |
| dDP | 81 | 29 | 0 (none) | 2926014 (sel 0.87, p1, int 2917816) | F20=620182 (sel 0.00, p1, int 617824); F40=1224196 (sel 0.00, p1, int 1217832); F60=1820153 (sel 0.40, p1, int 1817820); F80=2421867 (sel 0.40, p1, int 2417836); F100=2926014 (sel 0.87, p1, int 2917816) |
| dDP | 82 | 29 | 0 (none) | 820454 (sel 0.13, p1, int 817816) | F20=625237 (sel 0.00, p1, int 617824); F40=1224764 (sel 0.00, p1, int 1217828); F60=1824896 (sel 0.00, p1, int 1817836); F80=2425445 (sel 0.00, p1, int 2417820); F100=2925833 (sel 0.00, p1, int 2917820) |
| dDP | 83 | 29 | 0 (none) | 1721921 (sel 0.93, p1, int 1717820) | F20=618050 (sel 0.00, p1, int 617836); F40=1218678 (sel 0.00, p1, int 1217816); F60=1824214 (sel 0.00, p1, int 1817832); F80=2418904 (sel 0.00, p1, int 2417828); F100=2924722 (sel 0.33, p1, int 2917836) |
| dDP | 84 | 37 | 1 (pass-1 ended at row step 917820) | 2521820 (sel 0.73, p2, int 2517820) | F20=618732 (sel 0.00, p2, int 617828); F40=1221892 (sel 0.00, p2, int 1217832); F60=1823154 (sel 0.00, p2, int 1817828); F80=2423332 (sel 0.27, p2, int 2417824); F100=2919357 (sel 0.07, p2, int 2917828) |
| dDP | 85 | 58 | 1 (pass-1 ended at row step 2999632) | 2720784 (sel 0.40, p1, int 2717832) | F20=618679 (sel 0.00, p1, int 617820); F40=1221693 (sel 0.00, p2, int 1217816); F60=1823793 (sel 0.20, p1, int 1817836); F80=2418751 (sel 0.00, p1, int 2417820); F100=2999632 (sel 0.00, p1, int 117816) |
| dDP | 86 | 58 | 1 (pass-1 ended at row step 2999659) | 2622120 (sel 0.80, p2, int 2617824) | F20=619343 (sel 0.00, p1, int 617820); F40=1219630 (sel 0.00, p2, int 1217824); F60=1823189 (sel 0.00, p1, int 1817820); F80=2422649 (sel 0.13, p2, int 2417832); F100=2999659 (sel 0.00, p1, int 117816) |
| dDP | 87 | 58 | 1 (pass-1 ended at row step 2999737) | 420606 (sel 0.20, p1, int 417832) | F20=619424 (sel 0.00, p1, int 617816); F40=1221999 (sel 0.00, p2, int 1217816); F60=1822705 (sel 0.20, p2, int 1817820); F80=2420929 (sel 0.00, p2, int 2417816); F100=2999737 (sel 0.00, p1, int 117816) |
| dH | 80 | 29 | 0 (none) | 1618453 (sel 0.40, p1, int 1612056) | F20=617512 (sel 0.00, p1, int 612068); F40=1216389 (sel 0.07, p1, int 1212056); F60=1817064 (sel 0.00, p1, int 1812056); F80=2413479 (sel 0.00, p1, int 2412076); F100=2913721 (sel 0.00, p1, int 2912060) |
| dH | 81 | 29 | 0 (none) | 2913425 (sel 0.93, p1, int 2912060) | F20=613193 (sel 0.00, p1, int 612068); F40=1214289 (sel 0.73, p1, int 1212060); F60=1816844 (sel 0.00, p1, int 1812064); F80=2415137 (sel 0.67, p1, int 2412060); F100=2913425 (sel 0.93, p1, int 2912060) |
| dH | 82 | 29 | 0 (none) | 2414281 (sel 0.93, p1, int 2412076) | F20=619613 (sel 0.00, p1, int 612064); F40=1219696 (sel 0.00, p1, int 1212076); F60=1815639 (sel 0.93, p1, int 1812056); F80=2414281 (sel 0.93, p1, int 2412076); F100=2916755 (sel 0.27, p1, int 2912056) |
| dH | 83 | 29 | 0 (none) | 2915262 (sel 0.93, p1, int 2912072) | F20=614273 (sel 0.00, p1, int 612060); F40=1214620 (sel 0.00, p1, int 1212076); F60=1817788 (sel 0.33, p1, int 1812060); F80=2414276 (sel 0.93, p1, int 2412068); F100=2915262 (sel 0.93, p1, int 2912072) |
| dH | 84 | 58 | 1 (pass-1 ended at row step 2999588) | 2013156 (sel 0.07, p2, int 2012060) | F20=615184 (sel 0.00, p1, int 612068); F40=1213757 (sel 0.00, p1, int 1212068); F60=1815145 (sel 0.00, p2, int 1812056); F80=2412872 (sel 0.00, p1, int 2412064); F100=2999588 (sel 0.00, p1, int 112056) |
| dH | 85 | 37 | 1 (pass-1 ended at row step 912072) | 1513043 (sel 0.87, p2, int 1512072) | F20=617056 (sel 0.33, p1, int 612060); F40=1214650 (sel 0.20, p2, int 1212068); F60=1814095 (sel 0.00, p2, int 1812076); F80=2420239 (sel 0.00, p2, int 2412068); F100=2918762 (sel 0.07, p2, int 2912068) |
| dH | 86 | 37 | 1 (pass-1 ended at row step 913796) | 714789 (sel 0.93, p1, int 712072) | F20=616033 (sel 0.00, p1, int 612076); F40=1215056 (sel 0.00, p2, int 1212056); F60=1819733 (sel 0.00, p2, int 1812076); F80=2417056 (sel 0.07, p2, int 2412076); F100=2915603 (sel 0.87, p2, int 2912064) |
| dH | 87 | 58 | 1 (pass-1 ended at row step 2999939) | 2916245 (sel 1.00, p1, int 2912072) | F20=614395 (sel 0.00, p1, int 612060); F40=1213055 (sel 0.67, p1, int 1212076); F60=1813981 (sel 0.00, p2, int 1812056); F80=2417507 (sel 0.00, p1, int 2412068); F100=2999939 (sel 0.00, p1, int 112056) |

BEST_selected.pt provenance: s80–83 written by the launcher's `sort -k2 -rn ckpt_scores.tsv | head -1` on 08-29
(`BEST_selected.txt`: "sel-best among archived ckpts (ties->latest step) ... corrected world w3 gate"); s84–87 pinned
by hand on 09-01 (dH s84's txt: "highest sel among ON-DISK archived ckpts; tsv best 2222525/0.07 not archived"; the
other seven have no txt — identity established here by md5 against the surviving `ckpt_*.pt`).
BEST-of-K coincides with a fraction checkpoint in 5 runs (dH s81/s83 = F100, dH s82 = F80, dDP s80/s81 = F100).

## 2. Numbers of record being repaired (base world, `n12_rescore/*_W3_{hold,rnd}_*.log`, BEST_selected.pt, sampled, seed 0)

| arm | seed | BEST-of-K hold | BEST-of-K rnd |
|---|---|---|---|
| dH | 80 | 10/15 | 20/30 |
| dH | 81 | 15/15 | 25/30 |
| dH | 82 | 11/15 | 22/30 |
| dH | 83 | 10/15 | 16/30 |
| dH | 84 | 1/15 | 1/30 |
| dH | 85 | 11/15 | 21/30 |
| dH | 86 | 3/15 | 11/30 |
| dH | 87 | 13/15 | 17/30 |
| dDP | 80 | 1/15 | 1/30 |
| dDP | 81 | 2/15 | 2/30 |
| dDP | 82 | 2/15 | 3/30 |
| dDP | 83 | 15/15 | 28/30 |
| dDP | 84 | 13/15 | 22/30 |
| dDP | 85 | 0/15 | 1/30 |
| dDP | 86 | 9/15 | 12/30 |
| dDP | 87 | 6/15 | 5/30 |

Means: hold dH 0.617 v dDP 0.400 (+0.217); rnd 0.554 v 0.308 (+0.246). Ignition (BEST hold ≥ 8/15): dH 6/8 v dDP 3/8,
Fisher two-sided p = 0.315 (pure python, reproduced by `cluster/r2d_best5_collect.py`). All 32 cells present==expected.

## 3. Protocol of the re-score (step 2)

- Scorer: `cluster/r2d_rescore5.sh` (new; the `r2d_rescore.sh` loop re-implemented so it can pass `--mode` and
  export/verify the world). Per job: ONE checkpoint × ONE action mode, hold (15) then rnd (30), one fresh
  `eval_genesis.py` process per episode, `--seed 0 --device cpu --max-steps 1200` (sim steps), config = the run's own
  `.hydra/config.yaml` (delta_joint, action_repeat 4), `--ic-file baselines/eval_ics.json`. Cost-only differences from
  `r2d_rescore.sh`: `--torch-threads 2` (was default 4 → 16 threads on 8 cores) and `--scale 1` (64-px mp4).
- Action mode: `r2d_rescore.sh` passes no `--mode`; `eval_genesis.py` defaults to `sample` (actor rsample), which is
  therefore the mode of record. Adding the deterministic pass is one flag in the new scorer, so BOTH were submitted:
  `sample` (of record) and `mode` (actor mode; `--nice 500`, queued behind the sampled pass).
- World: `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6` exported; every episode log must contain
  `[sim-variant] gc_kp4_riser3_shelf6:` or the episode is counted as `wrongworld` and the cell is INVALID.
- Denominators (audit f23): `RESCORE5-RESULT ... picked=n/present expected=N missing=[...] wrongworld=[...]`; a cell
  is used only when present == expected and both lists are empty; otherwise it is rerun, never averaged.
- Checkpoints per run: F20/F40/F60/F80/F100 (the launcher's rule, recomputed verbatim over the surviving files) PLUS
  `BEST_selected.pt` (BEST-of-K) re-scored under the same world/process, so the finding-10 effect is separable from
  the world effect in §0(a). 16 × 6 × 2 modes = 192 jobs; `-p batch -c 8 --mem 24g --time 4:00:00 --exclude=pax007`,
  TMPDIR on the shared fs (`$LAB/tmp/r2dbest5_<job>`).
- Selection rule for BEST-of-5 (the DP/RLPD analogue): among the five, max in-job `sel` (tsv column 2), ties → later
  step; report that checkpoint's fresh hold/rnd. The max-over-5 of the fresh hold is reported separately as an
  optimistic bound, not as a selection.
- Outputs: `baselines/outputs/best5_rescore/{manifests/<run>.json, logs/<run>_<F>_<mode>_<job>.out,
  <run>_<F>_<mode>_<set>_<k>/metrics.json}`.

Exact commands (cluster, repo sha 6729e87→c.f. `git log cluster/r2d_rescore5.sh`; all through `~/bin/ccmd tufts`):
```
cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace && git pull -q --ff-only origin 4dof-cartesian
DRYRUN=1 bash cluster/r2d_best5_submit.sh                          # plan + manifests, no submit
SMOKE=1 RUNS="dH:84" bash cluster/r2d_best5_submit.sh              # job 3177410 (dH s84 F20 sample): world line verified
MODES="sample" bash cluster/r2d_best5_submit.sh                    # 96 jobs (3177501-…), sampled = mode of record
MODES="mode"   bash cluster/r2d_best5_submit.sh                    # 96 jobs, --nice 500
python3 cluster/r2d_best5_collect.py [--mode sample|mode]          # tables below
```
Queue note: the user's QOS caps CPUs at 250 (`sacctmgr show qos normal`: cpu=250); 248 were in use by the running
RLPD/r2d-pack trainings, so 4–5 re-score jobs run at a time (~45–120 min each on loaded batch nodes) — the sampled
pass alone is ≈1 day of wall time. Nothing was cancelled or modified; the pending GPU jobs already showed
`QOSMaxCpuPerUserLimit` before these were submitted.

## 4. Per-checkpoint results (step 3/4) — PENDING

First cell (job 3177410): `RESCORE5-RESULT tag=..._dH_s84_F20 mode=sample variant=gc_kp4_riser3_shelf6
ck=ckpt_615184.pt ck_step=612068 set=hold picked=0/15 expected=15 missing=[] wrongworld=[] node=pax002`.

(The per-seed table, arm means, ignition counts and Fisher p for BEST-of-K-in-corrected-world and BEST-of-5 are
produced by `cluster/r2d_best5_collect.py` and will be pasted here verbatim.)

## 5. Verdict — PENDING

Can only be written once all 96 sampled-mode cells are present == expected. What is already certain: the n=8v8 WM
numbers in RESULTS §3.1 are base-world evaluations of corrected-world policies (§0a), and five seeds' "100 %"
checkpoints are post-restart 112k-step snapshots (§0b); both must be disclosed with whatever finding 10 turns out to
change.
