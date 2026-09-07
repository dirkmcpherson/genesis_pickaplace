# Adversarial review — data pipeline and demonstration sets (2026-09-07)

*Brief: `~/wm_fix_2026-09-03/agent_brief_adversarial_common.md`. Scope: `baselines/rl/to_dreamer_native.py`, `$W/make_phase_banks.py`, `$W/matched_n.py`, `baselines/record_demos.py`, `baselines/convert_to_lerobot.py`, `baselines/discrete_replay_check.py`, and the built sets under `$W/demos_state{,_full,_r1}` + `$W/phase_banks`. Read-only; every CONFIRMED line below is a command I ran against the sets of record on the cluster. Scratch scripts: `$W/adv_review_data/chk{1..9}.py`.*

---

## S1 — changes a number or a claim of record

### 1. `hold15` is NOT a held-out set, and the results doc calls it one — in direct violation of a registered instruction
**CONFIRMED.** `paper/RESULTS_WM_HUMAN_VS_MACHINE_2026-09-04.md:73` reads:

> `hold15` — 15 **held-out** demo starts (`eval_ics.json`).

`paper/PREREG_final_round_robin_2026-08-23.md:278-279` had already forbidden exactly this:

> …not a held-out set; only `rnd` (30 support ICs) is novel. Report "sel / hold / rnd" by name; **never call hold "held-out"**.

And it is factually false for the world-model arms. Counting `ic_uid` (not the filename field, which is a rollout id):

| set | tapes | unique ICs | of the 15 `hold` uids present |
|---|---|---|---|
| `matched_w3/dHv2raw` | 66 | 66 | **14** |
| `matched_w3/dHv2all` | 106 | 88 | **14** |
| `matched_w3/dH` | 58 | 58 | **12** |
| `matched_w3/dDP` | 58 | 58 | **12** |
| `demos_v2/dHfull_w3_all` | 74 | 74 | **15** |
| `demos_v2/dDPfull_w3_all` | 195 | 72 | **14** |

Scenario: a reviewer reads §2.1's secondary cell "hold-15 116/120 vs 119/120" as generalisation to unseen starts; it is a memorisation cell, like `holdv2` (which the same doc labels correctly). Fix: one-line edit to `RESULTS_WM…:73` — "15 demo starts (`eval_ics.json` `hold`); **these ICs are in every training set — in-distribution, not held out**" — and a grep of the other docs for the same word.

### 2. The end-to-end arms are matched in tape count and unmatched in demonstrated success by 5.3×
**CONFIRMED.** `--one-per-ic-best` (`to_dreamer_native.py:193-201`) keeps, per IC, the attempt with the largest recorded reward sum. On `src_dDPfull`: 195 tapes over 72 ICs, **59 ICs with 3 attempts, 5 with 2, 8 with 1** — 89 % of the kept machine tapes are a best-of-2-or-3. Reward per kept tape 2.861 vs 1.718 over all attempts (+66 % selection uplift; first-tape-per-IC would have been 2.444).

Resulting demo content (from the built dirs and the runs' own prefill logs):

| | tapes | Σ reward | reward-sum histogram | rewarded terminals seen by the learner |
|---|---|---|---|---|
| `demos_state_full/dHfull_all` | 74 | **118** | none 10 / picked 43 / +contact 18 / **nested 3** | **3** |
| `demos_state_full/dDPfull` | 72 | **206** | none 2 / picked 34 / +contact 20 / **nested 16** | **16** |

(`full_r2d_state_dH…_s0/console.log`: `rewarded_terminals: 3`; `full_r2d_state_dDPfull_s0`: `16`.)

`REVIEW_GUIDE §2.7` states the arms as "74 human tapes incl. 10 no-picks vs machine best-per-start 72" and §3.1 summarises "with demonstration counts matched". The count is matched; the demonstrated task completions are not — the machine arm shows the learner five times as many full-task solutions, an advantage engineered by the selection rule and absent from the threat list (§4.2 mentions demo *count*, never best-of-N). Scenario: had §2.7 come out machine-favouring it would have been uninterpretable; as a null it is a null *under a bias toward machine*, which is a different (and in places stronger, in places weaker) statement than the one written. Fix: state the 3-vs-16 completions and the best-of-3 rule in §2.7 and in threat #1; the cheap control is a `--one-per-ic` (first attempt) machine arm, whose reward mass would be 176, still 1.5× human.

---

## S2 — weakens a claim (asymmetry, confound, protocol drift)

### 3. `holdE_place` is 85 % inside the human training set and 46 % inside the machine one
**CONFIRMED.** `holdE_place.json` holds 13 entries (uids 252 254 256 265 273 276 284 302 311 325 327 331 335). Training segments:
- `demos_state/dH_place` (39 uids) contains **11 of the 13**: 252 254 256 265 273 276 284 302 325 327 335.
- `demos_state/dDP_place` (39 machine uids) contains **6 of the 13**: 252 256 265 276 302 327.

`PHASE_RESULTS §1` calls holdE a *shared* bank and labels it "(in-distribution)" — true, but in-distribution to a different degree per arm, and 11 of the 13 human evaluation entries are literally the first state of a segment in that arm's own training set. The reported cell (104/104 vs 102/104) is saturated so no number moves, but "the learnability floor is met by every seed of both arms" rests on an asymmetric cell. Fix: add the 11/13-vs-6/13 line to §1, or rebuild holdE from uids in neither training set.

Related but **not** a leak: `holdE_contact` overlaps `dH_contact` on 2 of 11 and `dDP_contact` on 1 of 11, so the project's significant contact result (88/88 vs 83/88, p 0.026) is *not* explained by tape overlap. It is still an entry-**source** asymmetry — every holdE_contact entry is a human `placed_v2` state, which is the generative process the human arm trains from and not the machine's. `PHASE_PLAN §4` discloses "entry-distribution asymmetry between arms during training" generically; `REVIEW_GUIDE §2.5` presents the cell without that caveat.

### 4. `make_phase_banks.py` mixes two index conventions: the pick-grant entry is one decision EARLY, the place reward one decision LATE
**CONFIRMED from code.** In `record_demos.py` the tape row `t` is built as `states[t] = obs BEFORE decision t` (line 280) while the stage flags are appended *after* the window: `T[st].append(bool(info.get(st)) or (st in env._granted))` (line 315). So `picked[k]` describes state `k+1`, not state `k`.

`make_phase_banks.py:50` takes `k_pick = int(np.argmax(picked))` — a post-decision flag index — and `:57` banks `entry(s, a, k_pick, uid)`, i.e. the state **before** the granting decision. Measured over the 74 human tapes: can-centre z is 0.1709 at `s[k_pick]` and 0.1818 at `s[k_pick+1]` (machine: 0.1684 / 0.1780) — the banked "pick grant" state is 1.1 cm short of the grant. `k_contact`/`k_nested` share the flag convention; `k_placed_v2` (`:51`, `placed_v2_frame`) is computed directly on `s[j]`, so it *is* a state index.

Consequences, both arms identically:
- `place` cut `[k_pick, k_placed_v2]` (`to_dreamer_native.py:224`) starts one decision before the grant and, because the dreamer layout attaches `reward[t]` to arrival at state `t` (`:126,134`), pays its +1 on arrival at state `k_placed_v2+1` — one decision **after** the predicate first held. The place segment is 2 decisions longer than its own phase and its demo reward timing disagrees by one decision with the reward the same env pays online.
- `contact` and `carrycontact` end on a flag index, so their +1 lands exactly on the first granting state. **Place is the only phase with the off-by-one.**
- The banked entry being pre-grant is a live candidate mechanism for `REVIEW_GUIDE` threat #5 ("a held-can state cannot be reconstructed reliably for ≈3 % of entries; mechanism unresolved") — the can is 1 cm lower and the grasp less settled than the docs assume. **PLAUSIBLE**, not tested (would need a sim run).

Fix (cheap, no rerun): document the convention in `PHASE_PLAN §1` — "entry = the state one decision before the pick grant; the place phase's +1 is paid one decision after `placed_v2` first holds" — and, if anything is rebuilt, use `k_pick+1` and `k_placed_v2-1`.

### 5. The stride-1 re-encoding is arm-asymmetric: 1.26 % of human rows carry a wrong action, 0.00 % of machine rows
**CONFIRMED.** `to_dreamer_native.py:65-68` reconstructs the per-sim-step action as `clip((SA[j]-SA[j-1])/stride1_cap)`. `SA` is `_dj_target`, which `full_env.py:583` writes as `_dj_qmeas + clip(sp - _dj_qmeas, ±5·delta_cap)` — a leash against the *measured* joint — and `:582` additionally clips to `ARM_LO/ARM_HI`. Wherever either binds, the reconstructed delta ≠ the executed normalized action. Comparing the reconstruction against `actions_delta` repeated `rep` times:

| set | rows | mean abs err | p99 | max | frac > 0.05 | forced-zero first row, mean err |
|---|---|---|---|---|---|---|
| `dHv2raw` | 57 128 | 0.00886 | 0.378 | **1.141** | **0.0124** | 0.044 |
| `dDP` | 28 983 | 0.00001 | 0.000 | 0.092 | **0.0000** | **0.355** |

The human arm's follower saturates its own `clip(...,-1,1)` (`record_demos.py:472`) against the leash; the DP teacher never does. Also `:65` forces `d[0]=0`, replacing the machine's first executed action (mean magnitude 0.355) with a no-op. §2.3's clock control is a 4 v 4 null with MDE 0.128, so this cannot have produced the result — but it is an undisclosed arm-asymmetric label error in a cell of record. Fix: disclose, or (better) re-encode from `actions_delta` repeated rather than differencing `sim_actions`.

Positive note in the same place: `stride1_cap = 0.025` is **correct**, not the `0.00625` the flag's help text suggests — `full_env.py:581` advances the target by `a·delta_cap` on *every* sim step. Saturation is 0.41 % (machine) / 0.61 % (human) of rows, i.e. genuine at-cap commands, not an encoding artefact.

### 6. "Matched" matched tapes; rows, idle structure and the online entry bank stayed unmatched
**CONFIRMED.** Rows (excluding the leading zero-action row) and idle fraction at |Δ|<0.05 of cap:

| pair | human rows (idle) | machine rows (idle) | ratio |
|---|---|---|---|
| pick headline `dHv2raw` v `dDP` (66 v 58 tapes) | 14 323 (0.665) | 7 285 (0.416) | **1.97×** |
| place `dH_place` v `dDP_place_n39` (39 v 39) | 4 265 (0.503) | 4 727 (0.395) | 0.90× |
| contact `dH_contact` v `dDP_contact_n11` (11 v 11) | 978 (0.437) | 569 (0.109) | **1.72×** |
| carrycontact `dH_carrycontact` v `dDP_carrycontact_n21` (21 v 21) | 2 684 (0.427) | 3 066 (0.309) | 0.88× |
| end-to-end `dHfull_all` v `dDPfull` (74 v 72) | 29 221 (0.596) | 32 851 (0.434) | 0.89× |

Who sees which asymmetry: r2dreamer prefills all demo rows into a FIFO buffer (`$LAB/r2dreamer/demo_prefill.py:196-290`, `demo_duplicate 1`, `demo_reinject_every 0`) and samples rows uniformly, so demo *influence* is row-proportional — the human pick arm gets 2× the machine's. The runs' own logs also contradict "prefill never evicted": `eviction_note: FIFO: first demo frame evicted after 495 620 online env steps, all demo frames gone by 500 000` on a 1M-step run, i.e. **demos vanish at the halfway point** in every place/pick run and at ~467k of 2M in the full runs. RLPD's 50/50 demo batches make the *arm-level* exposure equal but the per-transition exposure row-weighted. DP is purely row-weighted, which is the known raw-human collapse.

Separately, the matched-count rerun matched demos but **not** the online training entry bank: `s2_r2d_place_state_dH_…_s0` trains from `human_place.json` (64 entries) and `s2_r2d_place_state_dDP_…_n39_s0` from `machine_place.json` (**179** entries over 72 ICs, up to 3 per IC). Same for carrycontact. So §2.4's "39 vs 39 demos" pair still differs 2.8× in distinct training starts. Fix: state it in §2.y, or subsample `machine_place.json` to one entry per IC for a matched-start rerun.

### 7. `dHv2all` composition is not what §2.2/§4.9 describes
**CONFIRMED.** `matched_w3/dHv2all`: 106 tapes, **88 unique ICs** (18 repeated), labels 73 `success` / 33 `fail`, stages `picked` 73 / `none` 33. `REVIEW_GUIDE §4.9` says it "mixes 40 failure/no-pick tapes into 66 successes". The added 40 tapes are 7 successes + 33 no-picks, and the set double-counts 18 ICs. Fix: "66 successes + 7 further picks + 33 no-picks, over 88 distinct starts".

---

## S3 — fragility that could bite what is queued

### 8. `--max-tapes` and `matched_n.py` leave the census fields of `repeat.json` stale
**CONFIRMED.** `matched_n.py` updates `n_written, total_reward, decisions_min, decisions_median, subsample_kept` but not `n_pick, n_nopick, n_cap_truncated, n_tipped_terminal, decisions_max`. Observed: `demos_state/dDP_place_n39/repeat.json` has `n_written 39` beside **`n_pick 63`**; `dDP_contact_n11` → `n_pick 25`; `dDP_carrycontact_n21` → `n_pick 36`; all three keep the parent's `decisions_max`. The same hole exists in `to_dreamer_native.py:260-266` (recomputes `lens`/`total_reward`, not `census`). Any future table script that reads `n_pick` from an `_n*` dir gets the pre-subsample number. `total_reward` **is** correct everywhere (= count × `terminal_reward` for every phase dir — checked).

### 9. `--force` does not clear the destination directory
**CONFIRMED from code** (`to_dreamer_native.py:187-188`, `:274-278`). A rebuild with fewer or differently-indexed tapes leaves the previous episodes behind, and every loader globs `*.npz`. No set of record is affected — file count equals `n_written` in all 15 dirs I checked — but the next rebuild-in-place is one keystroke from a silently mixed demo set. Fix: `shutil.rmtree(dst)` under `--force`, or refuse `--force` on a non-empty dir without `--clean`.

### 10. `--one-per-ic-best` tie-break is a speed prior with no human counterpart
**CONFIRMED from code** (`:197-198`): `key = (reward_sum, -n)` → among equal-reward attempts, the **shortest** wins. Combined with 89 % multi-attempt ICs this systematically shortens the machine set relative to a one-attempt human set, on top of the reward selection. (Same block loads every npz a second time without `allow_pickle=True` and never closes the handles — an object-array tape would crash it.)

### 11. A quarter of the demonstrated grants lie beyond the learner's own horizon
**CONFIRMED.** Full-scope tapes were recorded to ~2 400 sim steps (601 decisions) but the full runs use `time_limit: 1200` sim steps = 300 decisions. Fraction of demos whose last positive grant is past decision 300: human **0.234**, machine **0.286**. Of the nested demos, only 2 of 3 (human) and 7 of 16 (machine) nest inside the horizon; the machine's nested grants run out to decision 534. Place segments: 12.8 % (human) / 15.4 % (machine) exceed the 150-decision place horizon; carrycontact 38 % / 43 %. Roughly symmetric, so not a source confound — but it caps what the demos can teach and belongs next to §2.7's "weakest null".

### 12. `discrete_replay_check.py`: the commanded path runs a 20 % clock stretch
**CONFIRMED from code.** `CartesianCanEnv.DT = 0.025` integrates the setpoint per tape frame (40 Hz, correct), but `GenesisCanEnv.step` advances `for _ in range(3): scene.step()` at `dt=0.01` (`genesis_can_env.py:251`, `replay_harness.py:126`) = **0.030 s of physics per 0.025 s command frame**. `DISCRETE_ACTION_REPLAY_2026-09-06.md §1` states both numbers and never reconciles them. The spatial setpoint path is unaffected (position targets), so this is unlikely to be the cause of 17/74 — the doc's own offline drift analysis (median 4.2 cm at first close vs the bag's `tool_pose`) is the convincing explanation — but the ternary grip ramp is specified in units **per frame** and therefore runs 20 % slower in physical time than the real button.

### 13. `convert_to_lerobot.py` silently drops very short tapes
**CONFIRMED from code** (`:61-63`): contract-v1 default `MIN_FRAMES = 4`, applied by a filter that produces no warning. `dHfull_all`, `dDPfull` and `dHv2all` all contain tapes with `n = 1` (`decisions_min: 2` in their `repeat.json`, i.e. T = n+1 = 2). Any DP/lerobot leg built from those dirs loses them without a line in the log; `genesis_source.json` records the survivors only, so the drop is auditable but only by diffing against the source dir. `:62` also requires integer filenames (`int(p.stem)`), which the current `genesis-{uid}-{k}-{T}` naming would break outright.

---

## S4 — hygiene

### 14. The local `r2dreamer` demo loader parses the uid as the episode length
`~/workspace/r2dreamer/train.py:41-49`: `ep_length_from_path = int(p.stem.split("-")[1])`. Under the current naming `genesis-{uid:06d}-{k:03d}-{T}.npz` (commit `9b33e33`) field [1] is the **uid**, not `T` (field [3]). Consequences there: the "N successful + M failed" line is meaningless (every uid exceeds `time_limit`), and `paths[:maxnumdemos]` would truncate the set **by uid**. Not the path of record — the cluster tree uses `$LAB/r2dreamer/demo_prefill.py`, which loads every file and never parses the name — but `to_dreamer_native.py:276-277`'s comment ("the trailing `-{T}` field is what the dreamer loaders parse as the episode length") is false for the local copy. Fix: delete or sync the local copy.

### 15. `src_sha` does not identify the written set
`to_dreamer_native.py:295` hashes **all** files in `--src`, so two `--max-tapes` subsets of the same source share a `src_sha`. `subsample_kept` is the only distinguisher. Same for `matched_n.py` (it copies the parent's `src_sha` verbatim).

### 16. `matched_n.py:3` asserts `len(fs) > N`
A no-op match (N == available) crashes instead of copying. Trivial, but it is the exact call a "matched to the smaller arm" rerun would make.

---

## Claims that survived

Everything below is something I tried to break and could not; the check is named so it can be repeated.

1. **Contract-v1 invariants hold.** `sim_actions[rep·t+rep-1] == actions[t]` and `sim_states[rep·t] == states[t]` (atol 1e-5) on a stride sample of 9 tapes from each of `dHv2raw` (66), `dHv2all` (106), `dH` (58), `dDP` (58), `dHfull_w3_all` (74), `dDPfull_w3_all` (195): **0 violations, 0 tapes missing a sim tape.**
2. **The og4 recorder filter is provably absent from every phase/full set.** Three independent locks: `record_demos.py:820` asserts `teacher == 'human'` before arming it; the gain is read only from `VARIANTS[sim_variant]['grip_open_gain']` (`sim_variants.py:159-162` — present only on `…_ognow` / `…_og4`, absent from `gc_kp4_riser3_shelf6`); and all 74 human + 195 machine full-scope tapes carry `sim_variant == 'gc_kp4_riser3_shelf6'` in the npz itself (counted, not assumed). CONFOUNDS row 50 is correctly scoped.
3. **`matched_n.py` subsampling is uniform and unbiased.** `rng.choice(len(fs), N, replace=False)` over a sorted list, no key; rows per tape 121 (n39) vs 120 (parent) confirms no length bias.
4. **`total_reward` equals the count.** Every phase dir: `total_reward == n_written × terminal_reward` (39/39, 11/11, 21/21, 63/63, 25/25, 36/36, 104/104, 27/27).
5. **The contact holdE result is not a train/eval leak.** Overlap 2/11 (human) vs 1/11 (machine) — nearly symmetric, so the p 0.026 stands on its own (see S2-3 for the entry-source caveat that does apply).
6. **`stride1_cap = 0.025` is right and there is no saturation problem.** `full_env.py:581` integrates `a·delta_cap` per *sim* step; at-cap rows are 0.4–0.6 %, exact-zero rows 7 % (human) / 0.2 % (machine) — real idling, not clipping.
7. **`discrete_replay_check.py`'s command normalisation is correct.** `CartesianCanEnv(control='vel')` consumes **physical** m/s clipped to `±VCAP` (`cartesian_env.py:302-305`), which is what the script passes; the grip goes 0..100 → /100 → `genesis_can_env.py:224` `clip(a[6],0,1)*100` → `gripper_targets` — no silent grip-column scale error. `workspace_limit` defaults **False** (`genesis_can_env.py:74`) and `CartesianCanEnv` never sets it, so the reactive workspace override is *not* silently killing the raw path. World is `gc_kp4_riser3_shelf6` via `apply_pre`/`apply_post`, asserted in-script. The 17/74 stands.
8. **Unique naming works.** `genesis-{uid}-{k}-{T}` produced no collisions; file count equals `n_written` in all 15 built dirs.
9. **No version skew between the human and machine phase cuts.** `dH_place`/`dH_contact` were written 17:27:12–13 and `dDP_place`/`dDP_contact` at 17:28:56–57, straddling commit `a1229f9`; that commit adds only `--one-per-ic` and its manifest keys (`git show a1229f9`) — the cut logic in `convert_one` is byte-identical across the pair.
10. **The polE banks are disjoint.** `polE_place` keys start at 900000, `polE_place_dDP` at 910000, `polE_contact` at 920000 — §2.x's "the two banks share no entries" holds.
11. **Human set counts and uid sets check out.** `dHv2raw` 66/66 ICs all `picked`; `dHfull_all` 74 with exactly 10 `none` (the 10 no-picks); `dH_place` 39, `dH_contact` 11, `dH_carrycontact` 21, uid lists as printed in `chk2.py`.
12. **Machine full harvest cannot manufacture success.** `--verify` is *refused* at full scope (`record_demos.py:772-774`) and all 195 tapes carry `verify: 'n/a'`, but the label is `nested[-1]` from the env's own per-decision predicate on a closed-loop rollout — there is no open-loop step that could be non-reproducible. The IC leak is the intended `--ic-mode demo`, i.e. the disclosed distillation confound, not a pipeline defect.

---

## Summary (≤ 15 lines)

1. **S1** `RESULTS_WM…:73` calls `hold15` "held-out"; 14 of the 15 uids are in `dHv2raw`, 12 of 15 in `dH`/`dDP`, 15 of 15 in `dHfull_all`. `PREREG_final_round_robin:279` had explicitly forbidden that word. One-line doc fix.
2. **S1** The end-to-end arms match on tapes (74 v 72) and not on content: Σ reward 118 v 206, **3 v 16 demonstrated task completions**, because `--one-per-ic-best` selects the best of 3 attempts on 89 % of machine ICs (+66 % reward uplift). Disclose in §2.7 and threat #1.
3. **S2** 11 of 13 `holdE_place` evaluation entries start segments in the *human* training set vs 6 of 13 for the machine — an undisclosed asymmetry in a reported cell. (Contact holdE is clean at 2/11 v 1/11.)
4. **S2** `make_phase_banks.py` mixes post-decision flag indices (`k_pick`, `k_contact`) with a state index (`k_placed_v2`): the banked "pick grant" state is one decision *early* (can 1.1 cm lower) and the place phase's +1 lands one decision *late*. Symmetric across arms; a live candidate for the unresolved 3 % entry-restore failures.
5. **S2** The stride-1 re-encoding disagrees with the executed action on 1.26 % of *human* rows (max 1.14) and 0.00 % of machine rows — the leash in `full_env.py:583` bites only the human follower.
6. **S2** "Matched" is tapes only: pick 14 323 v 7 285 rows (1.97×), contact 978 v 569 (1.72×), and every place/carrycontact run trains from 64 (human) v 179 (machine) entry states, including the matched-count rerun. The runs' own logs also show demos are FIFO-evicted at the halfway point, contradicting "prefill never evicted".
7. **S3** `_n*` manifests carry stale `n_pick`; `--force` never clears the target dir; 23–29 % of full-scope grants lie beyond the learner's 300-decision horizon; the commanded-Cartesian path runs 0.030 s of physics per 0.025 s command frame.
8. **Survived:** contract-v1 invariants (0 violations, 6 sets), og4 absent from every phase/full set (three independent locks, verified in the tapes), uniform matched-N subsampling, `total_reward` == count everywhere, `stride1_cap` 0.025 correct with no saturation, `discrete_replay_check`'s normalisation/world/workspace handling and its 17/74, unique naming, no human/machine converter version skew, disjoint polE banks, and all human set counts and uid lists.
9. **Out of scope, S1-adjacent:** none found outside the data pipeline.
