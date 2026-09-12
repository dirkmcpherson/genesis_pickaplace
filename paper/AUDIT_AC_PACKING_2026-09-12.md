# AUDIT — PHASE_PLAN amendment (ac) `nested_sparse10` and its REVISION 1 (two r2dreamer seeds per GPU)

Audit lane (Fable), pop-os, 2026-09-12. Repo `/home/j/workspace/genesis_pickaplace`, branch `ladder-unify-2026-09-11`
@ `cbe41aa`, read-only. **No cluster access from this box (VPN down):** every finding is from repository artefacts,
`git show`, the two pytest suites (run), and the r2dreamer tree of record `0cf3d9e` checked out from
`cluster/bundles/r2dreamer_full_main_2026-09-12.bundle` into the scratchpad (read only). Nothing that opens a
Genesis world was run. An earlier lane's draft at this path was read and overwritten; every claim below was re-derived.

**Counts: 1 BLOCKING · 9 SHOULD FIX · 12 NOTE.** Tests, from tool output:
`~/workspace/genesis_sim2real/venv/bin/python -m pytest baselines/tests/test_ladder_unified.py baselines/tests/test_stage_predicates.py -q`
→ **63 passed, 1 warning in 3.53s**. `test_10h_…` is at `baselines/tests/test_ladder_unified.py:940`; the script runner
now sits below `test_11` (:981), as the log claims (:1151-1153).

## A. Registration discipline — CLEAN (positive finding, evidence in N1)

`git log --format='%h %ci'`: (ac) registered **8ec3f24 09:29:30** → code+sets+local RLPD smoke **5d92a09 09:46:46** →
launcher fix **8e54346 10:50:19** → cluster smokes 3618550/3618552 (log :1206, :1216; commit d19c56a 10:57:46) → batch
script **98527cb 11:46:55**, jobs 3620808-15 (log :1250-1259, a760daf 11:50:00) → **rev 1 registered dcbb483 11:56:25**
(re-worded ae379c6 11:57:02, 37 s later, before any pack) → smoke fixes 11:57:45 / 11:58:34 / **d1038ee 12:04:42** →
packed smoke 3621470 → packs 3622513/14 (log :1287-1292, a1bb988 12:47:36). The (ac) text at 8ec3f24 is byte-identical
to HEAD `PHASE_PLAN_2026-09-04.md:1596-1645`, **including P-ac-3's point numbers** (`home` 12/12, `n_pick` 65/64,
Σ 120.0/120.0, :1629-1632) — nothing was filled in after the fact. One thing the repo cannot show: that the sets were
built after 09:29:30 (only build-box mtimes can; the registration precedes the commit that reports the build by 17 min).

## B. The ladder — CORRECT

`full_env.py:162-163` `'nested_sparse10': dict(stage_reward=dict(home=10.0), terminal=('home',))` beside
`'nested_sparse'` (:154-155, untouched); `NESTED_LADDERS` (:178) includes it; `max_return` derives 10.0 (:240-247) and
`ladder_provenance()` sets `return_clamp_required` from it (:330-331); the stamp is generic (:342-360). RLPD demo gate:
`full_demos.py:93-99` asserts the set's recorded ladder equals the run's ladder and `:137-147` derives reachable values
from `full_env.LADDERS` — no edit needed, as the log says (:1148-1149). Suffix `_rns10` at `relabel_reward.py:121`.
Test `test_10h` pins 10 on a home tape, 0 on a drop, clamp 10.0, `nested_sparse` still 1.0 (test file :940-961).

## C. Launcher changes — what packing changes, and the blast radius of the two defects

`cluster/wmfix_full.sbatch` (dcbb483 → d1038ee): preflight (gates :79-81, demo gate :86-95, stamp+clamp :102-119) runs
once per job; `run_seed()` (:128-175) holds the logdir guard (:133), requeue clear (:134), the train command (:140-145),
budget check (:146-147), `[sim-variant]`/`Step accounting` checks (:157-159) and the eval loop (:161-173); pack mode
(:177-192) backgrounds one `run_seed` per seed with its own log and `wait`s each PID. Per seed nothing else changes:
`seed=$SEED` is set explicitly (:140), `R2_LONG_RUN`/`R2_MILESTONES` reach both processes through the job environment,
each process runs `train.py`'s own D6 check and writes its own `ladder_provenance.json` (r2dreamer `train.py:77-98`,
:114). Three real differences vs an unpacked run: (i) `TORCHINDUCTOR_CACHE_DIR` moved into `run_seed()` and gained a
`_s${SEED}` suffix (:132) — so the "single-seed path is byte-identical" claim (:123, commit dcbb483, PHASE_PLAN :1737) is
not exact (S8); (ii) a requeue restarts BOTH seeds from zero (:25 `--requeue` + :134) (S7); (iii) `-n 16 --mem=96g`
(`submit_ac_r2_pack.sh:28`) is exactly 2× the script's `-n 8 --mem=48g` (:29-30) — per-seed CPU/RAM matched.

**Blast radius of the disclosed defects: zero prior jobs.** `run_seed()` did not exist before dcbb483 (11:56:25);
`git log -- cluster/wmfix_full.sbatch` shows the previous touch is 5b69585 (2026-09-11 15:05), so at script level `"$@"`
was the hydra extras for (aa) 3581558-77, rev 2 and rev 3 3596021-32 — all submitted from `$LAB/gp_ladderN` @ a40c8aa1,
which predates the change, and Slurm spools the script at submission anyway. The bug also cannot mis-seed: inside the
function `"$@"` is the seed number, which Hydra rejects at start (`LexerNoViableAltException`, log :1277), and no submit
script in the tree passes hydra extras. The mid-line `#` lived 49 s (aafd993 11:57:45 → e9be11d 11:58:34) in a file
created that minute; under `set -u` (`submit_ac_r2_pack.sh:5`) an unset `TAGV` aborts at the `rm -rf` expansion (:17)
before any `sbatch`. The unpacked (ac) r2dreamer jobs 3620808-11 were submitted from gp_ac @ 98527cb9 (pre-bug) and
cancelled before starting (log :1269-1271, cluster-side, unverifiable here).

---

## BLOCKING

### B1. The milestone sweep cannot see the (ac) runs — P-ac-1 is unevaluable at the milestones it names
`cluster/ln_r2_milestone_sweep.sh:154-155`: `for suf in ("rnrh", "rnsh", "rzh"): runs += glob(… f"full_r2d_state_*_{suf}_s9*")`.
The (ac) run dirs are `full_r2d_state_dHfull_all_rns10h_s957` / `…dDPfull_first_rns10h_s977`
(`submit_ac_r2_pack.sh:26-27`); `_rns10h_s9` matches none of the three globs, and the sweep enumerates nothing new, with
no FATAL. {r2dreamer} milestone cells exist ONLY through this sweep — the launcher evaluates only `latest.pt` at the end
(`wmfix_full.sbatch:161-173`; log :966-970). P-ac-1 is registered "at matched milestones (0.5M/1M/2M/4M)"
(`PHASE_PLAN:1621-1624`). `HANDOFF_2026-09-11.md:168-169` says the deployed `$W/ln14_milestone_sweep.sh` "is identical"
to the repo copy; the repo's last commit to the file is 326910b (04:54, the dedupe fix), so unless the deployed copy was
edited without a commit, the (ac) 4M runs (~15 h each) will produce no milestone cells. **Fix (one line, no rerun):**
`("rnrh", "rnsh", "rns10h", "rzh")` in both copies, sha256 of the deployed copy into the log, before the first (ac) 0.5M
milestone (~2 h into a run). Same class: `HANDOFF:172`'s census command lists `e2e/` and `e2e_rev3/` only — the (ac)
{RLPD} runs live under `e2e_ac/` (`submit_ac_batch.sh:19`), so the P-ac-2 readout must add that root.

## SHOULD FIX

### S1. The r2dreamer launcher has no set-ladder == run-ladder gate; (ac) is protected only by the submit script
`wmfix_full.sbatch:86-95` checks `sim_variant`, stride, `with_state`, `terminal_reward == 1.0` (an INHERITED source stamp,
`relabel_reward.py:861-863`), `reward_from_tape`, `scope`, `n_written`, state shape — never `m['relabel']['ladder']`.
The r2dreamer tree checks only the stride (`demo_prefill.py:179-184`); `train.py:55` takes the ladder from the config.
So `LADDER=nested_sparse10` with a `_rnsh` set (Σ 12, a +1 prefill under a +10 clamp) — or the reverse — starts and
stamps cleanly. The RLPD side has the gate (`full_demos.py:93-99`). For this batch `submit_ac_batch.sh:24-27` asserts
ladder and Σ 120.0 at submission and `submit_ac_r2_pack.sh` relies on hand-pairing (:31, :34). **Fix:** add
`assert (m.get('relabel') or {}).get('ladder', 'staged') == os.environ['LADDER']` to the PYG block, and print the set's
ladder in the `[demo-gate]` line instead of the misleading `terminal 1.0`.

### S2. Packing IS on the P-ac-1 axis, and the disclosure argues the wrong contrast
Rev 1 (:1741-1747) argues packing "cannot by itself introduce a between-arm difference" — true, both arms are packed
identically (`submit_ac_r2_pack.sh:28-34`). But P-ac-1 compares `nested_sparse10` (packed) with rev-3's `nested_sparse`
(unpacked, `submit_ln_rev3.sh:83-84`) per seed: packing sits exactly on that comparison. The evidence that it is inert
exists and should be quoted: updates are gated by DATA steps (`trainer.py:32-33`, `Every(batch_steps/train_ratio*action_repeat)`,
`train_ratio: 512` in `configs/env/genesis_full_state.yaml`), the only wall-clock code is a logging timer (`tools.py:239-243`),
Genesis runs on CPU (`envs/genesis.py:214`) with per-seed cores matched, and the launcher already greps `Step accounting`
per seed (:159). **Fix:** one paragraph in rev 1 naming P-ac-1 as the exposed contrast and the train-ratio gate as the
reason it is not a confound; when 3622513 starts, put its two `Step accounting` lines beside rev-3's `s957`
(log :764-768 shows the form) — identical `prefill`/`counter target` closes it for 0 GPU-h.

### S3. No stall detection, and the one pack-specific lesson on record was not carried over
`CLAUDE.md` (cbe41aa) describes `ladder_health.py` as "deaths by sacct signature …, requeue-guard exit-2 pattern, disk"
— liveness only. The pop-os hang (`HANDOFF:140-144`: main blocked on the `ParallelEnv` socket, five workers in `poll`,
onset with six extra Genesis worlds, no OOM) would read RUNNING on sacct until the 2-day walltime (:31), losing both
seeds (S7). A pack is 2 × (`env_num: 6` workers + main) = 12 CPU Genesis worlds on 16 CPUs by construction — the same
count as the incident, though the two smokes (38 min, and the pre-unification 2-4/GPU wrappers) ran without a stall.
`sbatch_r2dreamer_pack.sh:11-12` records that unpacked r2dreamer jobs oversubscribed (load 138 on 64 cores) and set
`OMP/MKL_NUM_THREADS` per packed run; `PACK_SEEDS` sets no thread cap (`wmfix_full.sbatch:128-145`). **Fix:** add to the
15-min check `metrics.jsonl` mtime > 45 min while RUNNING → alert (both per-seed logdirs are known from the job name);
consider `OMP_NUM_THREADS=8` per seed in pack mode (a change to the launcher of record, so register it).

### S4. `$LAB/gp_ac` was fast-forwarded under running {RLPD} jobs — inert, but the proof is not in the log
{RLPD} 3620812-15 were submitted from gp_ac @ 98527cb9 (log :1246); the packs from gp_ac @ 294c1fbe (:1287) while three
RLPD jobs were RUNNING and import `$GP` at run time (and run `e2e_eval_cells.sh` from it at the end). This is the class
the standing rule forbids. Verified here: `git diff --stat 98527cb 294c1fb` touches `relabel_reward.py`, `wmfix_full.sbatch`,
`submit_ac_r2_pack.sh`, CLAUDE.md, paper/ only; `full_env.py`, `genesis_can_env.py`, `stage_predicates.py`, `full_demos.py`,
`train_rlpd.py`, `eval_e2e.py`, `sbatch_rlpd_e2e.sh` are blob-identical across 8e54346..294c1fb..HEAD; `relabel_reward.py`
IS a runtime import of `eval_e2e.py` (:352, under `--records-out`) but the three imported symbols (`_cpu_stamp`,
`FrameRecorder`, `grip_phys_from_action`) are md5-identical across the fast-forward and the module-level imports are
unchanged. **Fix:** state this in the log; pin gp_ac for the life of (ac); take a separate clone for the pixel lanes.

### S5. `ladderN_verify_sets.sh` cannot verify the (ac) sets and was not run on them
`:23` default `SETS` lists the six (aa) sets; `:69` `case … *_rnrh|*_rnsh|*_rzh` has no `*_rns10h` → `L` unset under
`set -u` (:16). This is the INDEPENDENT check (tape count, Σ, `home`, stamp, inherited `one_per_ic_first`, per-tape action
sha, both launcher gates) written after the 2026-09-09 false-claim defect; log :1155-1167 reports the builder's own numbers
only. **Fix:** add the branch, run `SETS="dHfull_all_rns10h dDPfull_first_rns10h" bash cluster/ladderN_verify_sets.sh`, paste.

### S6. The literal-list inventory is wrong and two sites were missed — including the only silent one
`CLAUDE.md`: "Six literal ladder-name lists (4 Python `choices=`, 2 bash launchers, 2 set-builder suffix maps)" = 8 by its
own arithmetic; commit 8e54346 says "the SIXTH". Actual: `eval_e2e.py:65`, `train_rlpd.py:137`, `annotate_demos.py:603`,
`pilot_rescore.py:46`, `relabel_reward.py:119-121`, `sbatch_rlpd_e2e.sh:72,73,79`, `wmfix_full.sbatch:17,42,43,48`,
`ladderN_sets.sbatch:97-98`, `relabel_e2e_sets.sbatch:72-73` (all fixed) + `ladderN_verify_sets.sh:23,69` and
`ln_r2_milestone_sweep.sh:154` (NOT fixed). See verdict 3.

### S7. Packing makes the failure/requeue unit two seeds — undisclosed asymmetry vs rev 3
`#SBATCH --requeue` (:25) + the clean-restart handler (:134) → a preemption/node failure at hour 14 discards both seeds.
Low risk on `--qos=normal`, but rev-3's comparator loses one seed per event. **Fix:** one sentence in rev 1.

### S8. "Single-seed submissions … unchanged/byte-identical" is false in one detail
The inductor-cache path changed (:132 vs the removed line in dcbb483's diff). Harmless; the wording (commit dcbb483,
`wmfix_full.sbatch:123`, `PHASE_PLAN:1736-1737`) should say "unchanged except the inductor-cache path".

### S9. Three cluster-side scripts are driving the batch with no versioned identity
`$W/ln14_milestone_sweep.sh` (asserted identical, no sha), `ladder_health.py`, `ln14_hourly.py` — the last two exist
nowhere in the repository (`find . -name 'ln14*' -o -name 'ladder_health*'` → nothing). **Fix:** sha256 into the log; commit them.

## NOTE
- **N1.** Registration order and text integrity: see §A. Rev 1 precedes the packs by ~50 min; its 37-s rewording
  (one allocation → two packs) is disclosed in ae379c6's message.
- **N2.** The log's "stamps, verbatim" block edits the demo-gate line: :1218 drops `(expected 4)` and `terminal 1.0`
  that `wmfix_full.sbatch:94` prints, without an ellipsis. Trivial; corrosive to the word "verbatim".
- **N3.** The packing basis "1.8 GB of 80 GB at ~21 %" (`PHASE_PLAN:1734-1736`, CLAUDE.md) has no `nvidia-smi` output
  anywhere in the log (grep). Standing rule: every number from tool output.
- **N4.** No set-level sha256 for `dHfull_all_rns10h` / `dDPfull_first_rns10h` (earlier sets carry one, e.g. `70027c89`);
  the per-tape action-sha equality (74/74, 72/72, :1162-1163) is the stronger claim but not what a later reader diffs.
- **N5.** CLAUDE.md over-attributes: "all PASSED with … `return_clamp=10.0 (env and model agree)`" — that line is
  r2dreamer-only; the RLPD smoke prints `[ladder] max_return 10.0` (:1210). The phrase "no visible slowdown" appears in no
  document; the log states 38 min (:1281) and no unpacked comparator — the exact one (3618552, same set/ladder/15k/`EVAL_SETS=hold`)
  has no Elapsed recorded; the nearest is `ln_smoke_r2_ramp` 36:02 (:446). One `sacct -j 3618552 -o Elapsed` turns an
  impression into a measurement. Every other CLAUDE.md claim traces to the log (job ids :1206/:1216/:1250-1259/:1281/:1291-1292,
  seeds, trees, the two defects :1276-1279, the `is_terminal` gap :1228-1238).
- **N6.** The (ac) sets use the class of record (`$W/stage_records_2026-09-11/`, pax146, :1155-1158); `home` 12/12 matches
  `HANDOFF:69`. The 13-vs-12 discrepancy is the LOCAL records' and is confined to (ac-local)/(ad), disclosed twice
  (`PHASE_PLAN:1658-1664`, :1706-1707). `submit_ac_batch.sh:26`'s `total_reward == 120.0` assertion is therefore correct
  for the cluster sets and would correctly REFUSE a local Σ 130 copy — fail-closed, good.
- **N7.** Pack job names carry no seed (`ln_r2_sparse10_pack_dH`, `submit_ac_r2_pack.sh:31,34`) and `sparse` is a prefix of
  `sparse10`, so `ln_r2_sparse*` matches rev 3 and (ac) — the rev-3 `e2e_rev3/` class (log :852-877). Run dirs are
  unambiguous (set name inside); key every readout on run dir + `ladder_provenance.json`, never on job name.
- **N8.** The r2dreamer launcher never touches `run_registry.py` (only `sbatch_rlpd_e2e.sh:158-159` does); the registry
  of record is the cluster's dirty `RUN_REGISTRY.jsonl` (:1246). Pre-existing.
- **N9.** In-job `fresh_eval_*` cells of a packed seed run while the other seed may still be training on the node; they are
  `preview` anyway — cells of record come from the pinned sweep (B1), one more reason to fix B1 first.
- **N10.** `PHASE_PLAN:1610` cites `ReturnEMA` at `networks.py:420`; at the tree of record 0cf3d9e it is `:390-406`. The
  substance is right: `scale = clip(ema_p95 − ema_p05, min=1.0)`, quantiles (0.05, 0.95).
- **N11.** The failed packed-smoke attempt(s) (QOS refusal, `#` abort, Lexer error) have no job ids in the log; only the
  passing 3621470 is named (:1281).
- **N12.** `demo_prefill.py:224`'s `rewarded_terminals` counts tapes whose LAST row is rewarded AND terminal — a diagnostic,
  correctly read as such in the log (:1228-1231).

---

## Verdicts on the three questions

**1. Packing — disclosure adequate for the between-ARM contrast; INCOMPLETE for P-ac-1 (S2); no unpacked control seed
needed.** No shared process state: two OS processes, two CUDA contexts, two Genesis worker pools (CPU backend), two
seeds, logdirs, inductor caches, per-seed Slurm logs; per-seed CPU and RAM are 2× matched. Computation is gated by data
steps (`trainer.py:33`), so the update:env-step ratio cannot move with wall clock; only elapsed time and GPU time-sharing
differ — the CONFOUNDS-79 nondeterminism class, not a new one. What a control seed would answer, the `Step accounting`
comparison answers for free (S2). The genuine exposure is operational, not statistical: a 12-world pack with no stall
detector (S3) and a two-seed requeue unit (S7). The local stall is not evidence that the cluster pack will stall (different
trigger: a probe opening worlds beside a trainer; the 38-min packed smoke and the old 2-4/GPU wrappers ran), but the
15-min check cannot see one if it happens. Add the mtime check; leave the packs running.

**2. `is_terminal` — real, confirmed in code, plausible only as a SECOND-ORDER contributor; NOT a mechanism for "not
picking at 1M"; blocks nothing now.** Mechanism: the relabel keeps every row and zeroes reward after the terminal
(`relabel_reward.py:33-36`, :474-475), `is_terminal` is "unchanged from the source tape" (:902-904), the converter sets it
only on the last row (`to_dreamer_native.py:135`), and the trainer builds the continuation target directly from it
(`dreamer.py:471` `cont = 1 − is_terminal`). So the 12 `home` rows enter the prefill as +1/+10 with `cont = 1` and
zero-reward frames after them, while online `home` ends the episode. Effects: (a) the cont head is taught "continue" at
the one state the env terminates on, and online data never reaches `home` to correct it; (b) the critic's target at
`home` becomes `r + γ·V(post-home)` instead of `r`, an OVER-estimate that WIDENS the λ-return spread — the opposite
direction to the ReturnEMA-floor deficit (ac) is built to test, and 10× larger in return units under +10; (c) the
`paid-once` structure is history-dependent, so an imagined rollout parked at the goal can re-collect the terminal unless
the RSSM tracks history — the return clamp (10.0) bounds the damage. All of this is downstream of picking; the cluster
`nested_sparse` seed that failed had `picked` 0/45 at 1M (`HANDOFF:94-95`), which a value error localised at `home` cannot
explain — 12 rewarded rows in ~29k decisions and the +1 scale are the first-order candidates, which is what (ac) tests.
It affects `nested_sparse` and `nested_sparse10` identically, so the (ac) contrast is internally valid. **Decisive
experiment:** register `--cut-at-terminal` (`HANDOFF:157-162`), build `dHfull_all_rns10hT` (tape truncated at the paid
terminal, `is_terminal` set there — this changes tape LENGTH, so provenance becomes prefix-identical action sha and P-ac-3's
"identical" clause must be re-registered), run ONE r2dreamer seed to 1M against s957's 1M milestone; `picked` and `home`
decide. ~15 GPU-h. Run it only if P-ac-1's 1M milestone picks (else the scale account is already dead and this is moot).

**3. Appending was right for the day and wrong as the answer; the inventory was incomplete, and the missed site is the
silent one.** Under a live batch, appending avoids rewiring launchers that spooled jobs read — correct. But the failure
modes differ by site class and the fix should too: REFUSAL lists (`case … *) FATAL`, argparse `choices=`) fail closed and
loud — acceptable to leave, cheap to derive; SUFFIX maps (`ladderN_sets.sbatch:97`, `relabel_e2e_sets.sbatch:72`) fail
loud and are additionally asserted by `relabel_reward.LADDER_SUFFIX` — derive from it with one `$PY -c`; FLAG maps
(`FAR_CFG`/`FAR_FLAG`, `wmfix_full.sbatch:48`, `sbatch_rlpd_e2e.sh:79`) would REFUSE the next nested ladder's
`FAR_RELEASE=1` with a wrong message ("has no farside rung") — loud but misleading; `full_env.NESTED_LADDERS` (:176-178)
exists for exactly this and the bash side ignores it; ENUMERATION lists (`ln_r2_milestone_sweep.sh:154`,
`ladderN_verify_sets.sh:23`, `pilot_rescore.py:46`) fail SILENTLY — a ladder simply never appears in a table. Two of the
twelve sites are silent; one was fixed, one was missed (B1). **Recommendation:** derive validation lists from
`full_env.LADDERS`/`NESTED_LADDERS` and suffixes from `relabel_reward.LADDER_SUFFIX` (both launchers already shell out to
`$PY` for the stamp, :102-118 — zero new dependency); make enumeration sites discover work from artefacts
(`ladder_provenance.json` in run dirs, `repeat.json['relabel']['ladder']` in set dirs), never from suffix globs; add a
unit test that greps `cluster/` and `baselines/` for a literal `nested_sparse|nested_ramp` list outside `full_env.py`.
Do the derivation when no batch is spooled; fix the two enumeration sites now.

## Could not verify from here (needs the cluster)
1. Job states/exit codes: 3618417/18 (refused), 3618550, 3618552, 3621470, 3620808-15 (incl. that 3620808-11 were cancelled
   before starting and their logdirs cleared), 3622513/14; the failed packed-smoke attempt ids (N11).
2. Elapsed of the unpacked (ac) r2dreamer smoke 3618552 (the packing-throughput comparator, N5).
3. The `_rns10h` sets on `$W/demos_state_full/`: Σ, `home`, `n_pick`, `end_reasons`, per-tape shas, build node and
   timestamp (P-ac-3's exactness; that the build postdates 09:29:30).
4. `$LAB/gp_ac`'s HEAD/dirty state now; `$W/r2dreamer_ladderN` still @ 0cf3d9e; the `[ladder]` stamps of 3622513/14.
5. The deployed `$W/ln14_milestone_sweep.sh`, `ladder_health.py`, `ln14_hourly.py` — whether the sweep already carries
   `rns10h` and whether the health check has any stall rule (B1, S3, S9).
6. Whether `ladderN_verify_sets.sh` was run on the (ac) sets in a patched cluster-side form (S5).
7. `nvidia-smi` for the 1.8 GB / 21 % basis (N3); the actual GPU model and SMT state of the nodes the packs land on.
8. The bash version on the cluster (`"${EXTRA_OVERRIDES[@]}"` on an empty array under `set -u` needs bash ≥ 4.4; the
   passing smoke 3621470 implies it is fine).
