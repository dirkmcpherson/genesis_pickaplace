# P1 — delta_joint open-loop replay: where the demonstrated pick is lost

Date: 2026-08-13. Author: Fable session (local box, CPU only).
Context: FABLE_HANDOFF_2026-08-13.md §8. Answers diagnostic #1 and #2.

---

## 1. Verdict (short)

Three distinct mechanisms are in play. They were confounded in the phase sweep.

1. **Target drift (mechanism a) is the proximate killer of the pick.** The
   delta-integrated joint target separates from the demonstrated joint command
   BEFORE the grasp on the losers, and only AFTER the grasp on the keeper. The
   joints follow the drifted target, so the gripper closes in the wrong place and
   the can is never lifted. World divergence (mechanism c) is a CONSEQUENCE — the
   can error stays at 0.0 mm until hundreds of steps after the target drift starts.
   Tracking lag (mechanism b) does not discriminate: it is bounded by the leash and
   is just as large on the keeper.
2. **A second, independent defect: the inner `GenesisCanEnv` horizon is 1200 and
   `FullTaskEnv` never raises it.** Past step 1200 every `genv.step` runs
   `_nested()`, which steps the sim 100 extra times. This is the #26 phantom-settle
   bug. `collect_all_classified.py` fixes it explicitly (`env.max_steps = 10**9`,
   with uid 300 named in the comment). `full_env.py` does not. 54 of the 72
   resettable replays run past 1200 steps, so 75% of the phase sweep is
   contaminated. Raising the horizon at runtime recovers 2 of the 8 lost picks and
   upgrades the keeper by two stages.
3. **The nested grants are manufactured, by three independent routes.** `genv`
   only computes the honest settled `nested` at its own horizon, so in a replay the
   ONLY other source is the training-only proxy in `full_env.py` (contact + grip
   commanded open + both upright, no settle, no proximity test). uid 305 (labelled
   `placed`) is granted nested by that proxy on a transient, and the env then
   terminates the episode. uids 274 and 316 lose their spurious nested once the
   1200-horizon defect is removed. uid 261 (labelled `no-pick`) manufactures a pick
   and a place either way, because its replay diverges from step 0.

**Truncation is NOT a factor.** No nested- or contact-labelled demo has its
recorded pick frame past the sweep's `max_steps=4000`.

Consequence for the fix: option (i) "larger cap/leash" alone will not do it, and
option (iv) "bump sweep max_steps" is not needed. The load-bearing fixes are the
1200-horizon regression and, for the delta representation itself, option (ii) —
closed-loop re-record — or option (iii) — delta for pick-scope only.

---

## 2. Setup

- Env: `FullTaskEnv(backend='cpu', max_steps=6000, scope='full',
  action_mode='delta_joint', action_repeat=1)`. `pick_z=0.1505`,
  `delta_cap=0.025`, `delta_leash=0.125` (5 x cap).
- Demos: `baselines/episodes_all/<uid>.npz`, encoded by
  `train_sacfd_full.delta_encode_transitions(paths, pick_z, 'full', cap)`.
  Note this encoder calls `relabel_full`, which truncates the transition list at
  the labelled nested frame. That is why `n_enc` < recorded frames on some demos
  (uid 300: 3366 encoded vs 4770 recorded). The recorded pick frame (3097) is
  inside the encoded window, so nothing is cut before the grasp.
- Recorded pick frame = first `i` with `states[i,10] > 0.1505` AND
  `actions[i,6] > GRIP_CLOSED_FRAC` (0.3). This is `relabel_full`'s own predicate.
- Per step the trace records, after the step:
  - **(a) target drift** `A = ||env._dj_target - demo_cmd_i||_inf` (rad)
  - **(b) tracking lag** `B = ||obs[:6] - env._dj_target||_inf` (rad)
  - **(c) world divergence** `C = ||obs[8:11] - states[i+1,8:11]||_2` (m)
  - plus `qerr = ||obs[:6] - states[i+1,:6]||_inf`, the joint error against the
    demo itself (A measures the command, qerr measures the achieved pose).
- Leash logic being instrumented (`full_env.py` `_step_once`):
  `sp = clip(target + a*cap, ARM_LO, ARM_HI)`, then
  `target = qmeas + clip(sp - qmeas, +-0.125)`.
  **B is bounded by the leash by construction**, so the 0.15 rad threshold on B is
  unreachable. Read B against 0.125, not against the 0.15 row.
- Baseline outcomes are the repeat-1 phase sweep
  (`scratchpad/phase_sweep_r1_s{0..5}.log`), which used the DEFAULT inner horizon.
- One run per condition. The CPU sim is deterministic within a process but NOT
  across processes: uid 305 granted nested at step 1122 in one process and 1001 in
  another, same actions and same config. Treat single-demo step indices as +-~10%.
  The box was also carrying six RLPD training runs.

---

## 3. Truncation check (diagnostic #0) — negative

Recorded pick frame vs the sweep's `max_steps=4000`, all nested- and
contact-labelled demos:

| uid | label | frames | recorded pick frame | > 4000? |
|-----|-------|--------|---------------------|---------|
| 295 | contact | 6232 | 3693 | no |
| 300 | nested | 4770 | 3097 | no |
| 328 | nested | 2766 | 1608 | no |
| 248 | nested | 3062 |  964 | no |

Maximum recorded pick frame over all 31 nested/contact demos is 3693. **No pick is
lost to the 4000-step cap.** The only demos with any frames past 4000 are 295
(6232) and 300 (4770), and in both the pick is well inside. Truncation can only
affect their DOWNSTREAM phases, not the pick.

---

## 4. Per-uid divergence trace (diagnostic #1)

Inner horizon RAISED for these three runs, so mechanism (a) is measured without the
phantom-settle confound. Thresholds: 0.01 / 0.05 / 0.15 rad for A, B, qerr;
5 / 20 / 50 mm for C. "step" = index into the encoded action sequence.

### First crossing, by uid

| uid | role | recorded pick frame | A>0.05 | B>0.05 | qerr>0.05 | C>5mm | C>20mm | C>50mm |
|-----|------|--------------------|--------|--------|-----------|-------|--------|--------|
| 300 | LOSER  | 3097 | **2417** | 1675 | 2425 | 2661 | 2669 | 2683 |
| 328 | LOSER  | 1608 | **61**   | 56   | 64   | 708  | 787  | 953  |
| 248 | keeper |  964 | **1023** | 82   | 1031 | 196  | 1097 | never |

A>0.15 rad: uid 300 at 2584, uid 328 at 193, uid 248 at 1102. B never reaches 0.15
on any uid (leash-bounded; maxima 0.124 / 0.134 / 0.115).

### Lead time of each quantity relative to the recorded pick frame

Positive = diverged BEFORE the grasp.

| uid | outcome | A lead | qerr lead | C(20mm) lead | C(50mm) lead |
|-----|---------|--------|-----------|--------------|--------------|
| 300 | pick lost | +680 steps | +672 | +428 | +414 |
| 328 | pick lost | +1547 | +1544 | +821 | +655 |
| 248 | pick kept | **-59** (after) | **-67** (after) | **-133** (after) | never |

### State at the recorded pick frame

| uid | A (rad) | B (rad) | qerr (rad) | C (mm) | replay can_z | demo can_z | replay re-earned the pick? |
|-----|---------|---------|------------|--------|--------------|------------|---------------------------|
| 300 | 0.273 | 0.073 | 0.274 | 87.7 | 0.1005 | 0.1553 | NO (max can_z 0.1049) |
| 328 | 0.226 | 0.078 | 0.240 | 55.3 | 0.1012 | 0.1543 | NO (max can_z 0.1205) |
| 248 | **0.012** | 0.076 | **0.012** | **6.6** | 0.1583 | 0.1549 | YES, at step 962 (2 early) |

`pick_z = 0.1505`. On both losers the can never leaves the table: max can_z 0.105
and 0.121 against a demonstrated 0.261 and 0.256.

**Which quantity diverges first: A (target drift), not C (world).** On both losers
the can-position error is EXACTLY 0.0 mm for the first 2400 (uid 300) and 400 (uid
328) steps — the arm is missing the can in free space, it is not being knocked off
course by contact chaos. C only moves once the (already drifted) arm reaches the
can. B crosses 0.05 earliest on all three uids including the keeper, so it is not
diagnostic.

### How the drift is produced, and why it never heals

Every large single-step jump in A is exactly +-one cap (0.0256-0.0313 rad) and sits
inside a local burst of cap-saturated frames:

| uid | cap-saturated steps | leash-saturated steps | first A-jump burst | saturated frames in +-25 of the burst |
|-----|--------------------|----------------------|--------------------|--------------------------------------|
| 300 | 101 / 3366 (3.0%) | 0 (0.0%) | steps 2446-2552 | 8 -> 31 of 50 |
| 328 | 151 / 2765 (5.5%) | 79 (2.9%) | steps 212-257 | 26 -> 37 of 50 |
| 248 | 58 / 2962 (2.0%) | 0 (0.0%) | steps 1023-1132 (AFTER its pick) | 5 -> 28 of 50 |

After the burst, A goes FLAT and stays there for the rest of the episode:
uid 328 A = 0.2260 with std **0.00000** over its last quarter; uid 248 A = 0.0758
with std 0.00000. This is the structural point. `delta_encode_transitions` encodes
DIFFERENCES of the command (`cmd_t - cmd_{t-1}`), so the integrated target carries
no error feedback. Any frame where the cap or the leash truncates the requested
delta produces a permanent step offset between target and demonstrated command. The
velocity replay re-sends the absolute command every frame and therefore re-converges
after any lag; the delta replay cannot.

The leash is a second truncation source but is minor here: it saturates on 0% of
uid 300's and uid 248's steps and 2.9% of uid 328's.

### Population test of mechanism (a)

Offline emulation, no physics: run the encoder + env integration rule with the
RECORDED joint positions substituted for `qmeas` (i.e. assume perfect tracking) and
measure `|target - cmd|_inf` averaged over the 50 steps up to the recorded pick
frame. All 65 resettable demos that have a recorded pick, split by whether the real
delta replay kept the pick:

| group | n | median drift at grasp | mean |
|-------|---|----------------------|------|
| replay LOST the pick | 32 | **0.0602 rad** | 0.0641 |
| replay KEPT the pick | 33 | **0.0445 rad** | 0.0422 |

Mann-Whitney U, one-sided (lost > kept): **p = 0.0010**.

So the encoder's own command-reproduction error at the grasp — computable with no
simulation at all — predicts which demos lose the pick.

### What did NOT discriminate (tested and rejected)

- **Cap-saturation rate. The handoff's refutation STANDS.** Global rate: lost
  median 4.61% vs kept 3.49%, p = 0.174. Saturation fraction in the fixed 200 steps
  before the pick: lost 0.5% vs kept 1.5%, p = 0.81 (400-step window: p = 0.89).
  An earlier version of this analysis used the MAX local density over the whole
  pre-pick period and found p = 0.0004; that statistic is confounded by episode
  length (a longer pre-pick period gives a larger maximum) and is withdrawn.
  Saturation is the mechanism that CREATES a drift step, but its amount near the
  grasp does not tell you who fails — what matters is which joints the frozen
  offset lands on.
- **Grip clipping.** `collect_all_classified` deliberately executes the RAW
  recorded gripper (which undershoots below 0) because the [0,1] clip cost 12 demos
  their pick; the stored `actions` column is clipped, so the delta replay uses the
  clipped value. But demos with any raw gripper < 0: 12/38 among pick-losers and
  12/34 among pick-keepers. No association. The recorded undershoot is also tiny
  (min -0.96 on a 0..100 scale). Not a discriminator.
- **Episode length alone.** Confirmed as stated in the handoff: uid 248 is 3062
  frames and keeps the pick.

---

## 5. The 1200-step inner horizon (a separate, fixable defect)

`FullTaskEnv.__init__` builds `GenesisCanEnv(...)` with its default
`max_steps=1200` and never changes it. `GenesisCanEnv.step` does:

    done = self._t >= self.max_steps
    if done:
        info['nested'] = self._nested()     # _nested() runs 100 extra scene.step()

`_step_once` ignores the returned `_env_done`, so the replay keeps going and every
step past 1200 costs 100 phantom sim steps. `collect_all_classified.py` contains the
explicit fix and the explicit warning, naming this exact uid:

    env.max_steps = 10 ** 9
    # CRITICAL: default max_steps=1200 is an EVAL horizon. ... each demo >1200
    # frames silently desynced from cmd 1200 on (uid 300: ~357k phantom steps;
    # the 42mm env-vs-replay divergence, #26 successor).

The fix was applied to the collector and never to `full_env.py`. **54 of the 72
resettable demos encode to more than 1200 steps**, so this contaminates 75% of the
phase sweep and every long-demo replay used for the contact/nested counts.

Control: identical replay, `env.genv.max_steps` raised at runtime to 1e9 (attribute
assignment only; no code file was modified).

| uid | label | sweep stage (horizon 1200) | stage with horizon raised | effect |
|-----|-------|---------------------------|---------------------------|--------|
| 312 | nested | no-pick | **picked** (step 1614 vs recorded 1607) | pick RECOVERED |
| 320 | nested | no-pick | **contact** (pick step 1384 vs recorded 1385) | pick RECOVERED, +2 stages |
| 248 | nested | picked | **contact** | +2 stages |
| 274 | placed | nested (manufactured) | placed | spurious nested REMOVED |
| 316 | contact | nested (manufactured) | contact | spurious nested REMOVED |
| 300 | nested | no-pick | no-pick | unchanged |
| 328 | nested | no-pick | no-pick | unchanged |
| 330 | nested | no-pick | no-pick | unchanged |
| 247 | nested | no-pick | no-pick | unchanged |
| 256 | nested | no-pick | no-pick | unchanged |
| 298 | nested | no-pick | no-pick | unchanged |
| 305 | placed | nested (manufactured) | nested (manufactured, proxy) | unchanged |
| 261 | no-pick | placed (manufactured) | placed (manufactured) | unchanged |

Both recovered picks land within 1-7 steps of the recorded pick frame — a faithful
replay, not a lucky regrasp. Note 312 and 320 are exactly the two lost-pick demos
whose recorded pick frame sits past 1200 and whose pre-pick target drift is low.
uid 300 and 328 also pick past 1200, but their drift kills them anyway.

Population correlate (default horizon, repeat-1 sweep): of the 16 resettable demos
whose recorded pick frame is past 1200, 15 lose the pick (94%); of the 49 whose
pick is before 1200, 17 lose it (35%). Fisher exact p = 3.2e-5. The raised-horizon
control shows this correlation is only PARTLY causal — 2 of the 6 tested recover.

---

## 6. Manufactured successes (diagnostic #3)

`genv` sets `info['nested']` only at its own horizon. In an open-loop replay of a
demo shorter than 1200 steps that path never runs, so **every nested grant in the
sweep comes from the training-only proxy** in `full_env._step_once`:

    if info.get('contact') and float(a_phys[6]) < 0.3 and 'nested' not in self._granted:
        if tilt_deg(bottle) < 20 and tilt_deg(goal) < 20:
            info['nested'] = True

Instrumented replay of uid 305 (labelled `placed`, granted `nested` by the sweep),
recording the grip command at the granting step:

| run | nested granted at step | grip command at that step | source | episode end |
|-----|-----------------------|---------------------------|--------|-------------|
| default horizon | 1122 | open (< 0.3) | **proxy** | terminated on nested |
| raised horizon | 1001 | open (< 0.3) | **proxy** | terminated on nested |

Confirmed: the spurious nested is the proxy firing on a transient. The proxy has no
settle and no proximity test — it accepts "touching + released + upright right now",
whereas the honest metric (`_nested()`) settles 100 steps and requires centre
distance <= NESTED_TOUCH_DIST. The env then TERMINATES on it, so the rest of the
demo is never replayed.

All 4 nested grants in the repeat-1 sweep (uids 251, 274, 305, 316) come from demos
longer than 1200 steps, and all 4 stage-gains over the label (261, 274, 305, 316)
likewise. Attributed by the raised-horizon control:

| uid | sweep gain | with horizon raised | attributed to |
|-----|-----------|---------------------|---------------|
| 274 | placed -> nested | placed | phantom-settle perturbation |
| 316 | contact -> nested | contact | phantom-settle perturbation |
| 305 | placed -> nested | nested (proxy) | training-only nested proxy |
| 261 | no-pick -> placed | placed | neither — see below |

uid 261 is a third kind. Its label is `no-pick` (the canonical velocity replay
itself failed to pick), yet the delta replay picks at step 646 and reaches placed,
with or without the horizon fix. Its trace diverges from the FIRST step: can error
13.8 mm at step 0, 52 mm by step 12, target drift > 0.05 rad by step 68, A_max 0.38.
The delta replay of 261 is simply a different episode that happens to succeed. So
the manufacturing has three independent sources: proxy transients, phantom-settle
perturbation, and drift that lands somewhere lucky.

The 121-step disagreement between the two uid-305 runs (1122 vs 1001, identical
actions and config) is worth flagging on its own: the CPU sim is not reproducible
across processes, so single-demo downstream outcomes carry real run-to-run noise.

---

## 7. Verdict per uid

| uid | role | mechanism |
|-----|------|-----------|
| 300 | pick LOST | **(a) target drift.** Drift crosses 0.05 rad 680 steps before the grasp and freezes at 0.273 rad; can error is 0.0 mm until 436 steps before the grasp, so the arm misses in free space. Not truncation (pick frame 3097 < 4000), not the 1200 horizon (still lost when raised). |
| 328 | pick LOST | **(a) target drift, from step 61.** Drift 0.226 rad frozen (std 0.00000) for the whole run; 1547 steps of lead on the grasp. Not the 1200 horizon (still lost when raised). |
| 248 | pick KEPT (control) | Clean through the grasp: drift 0.012 rad and can error 6.6 mm at the pick frame, replay picks at step 962 vs recorded 964. Its drift onset is at step 1023, i.e. 59 steps AFTER the pick, and freezes at 0.0758 rad — which is why it then loses the downstream phases. Under the DEFAULT horizon it also loses 2 stages to the phantom settle (picked vs contact). |
| 312, 320 | pick LOST in the sweep | **1200-horizon phantom settle.** Both recover the pick within 7 steps of the recorded frame once the inner horizon is raised. |
| 330, 247, 256, 298 | pick LOST | Not truncation, not the 1200 horizon (all still lost when raised). Consistent with (a); per-step traces not run. |
| 305 | nested MANUFACTURED | Training-only nested proxy firing on a release transient, then terminating the episode. Survives the horizon fix. |
| 274, 316 | nested MANUFACTURED | Removed by raising the inner horizon, so phantom-settle perturbation. |
| 261 | pick + placed MANUFACTURED | Labelled no-pick; the delta replay diverges from step 0 (13.8 mm can error) into a different, luckier episode. Survives the horizon fix. |

Score for the 8 lost picks: **2 explained by the 1200-step horizon regression,
6 consistent with delta target drift, 0 by truncation.**

---

## 8. What this implies for the fix

Ordered by cost.

1. **Raise the inner horizon in `FullTaskEnv` (do this first, it is a one-line
   regression).** `GenesisCanEnv(...)` should be built with `max_steps` at least as
   large as `FullTaskEnv.max_steps`, mirroring `collect_all_classified.py`. It
   affects TRAINING as well as replay: any full/pick-scope episode longer than 1200
   sim steps is currently paying 100 phantom settle steps per step. At
   `action_repeat=8` and `max_steps=900` this does not bite, but any full-task or
   long-horizon run does. Recovers 2 of 8 lost picks and 2 stages on the keeper.
   NB it also removes one manufacturing path (the spurious nested on 274 and 316).
2. **Do NOT bump the sweep `max_steps` (option iv).** Truncation is ruled out.
3. **Option (i) larger cap/leash: partial at best, and not free.** The drift is
   created by clip events, so a larger cap would reduce them — but the amount of
   saturation near the grasp does not separate losers from keepers (§4), and
   raising the cap widens the action space the policy must explore, which is the
   reason the cap was set to the demos' p99 in the first place. If tried, measure
   it with the offline predictor in §4 before spending sim time: it is free and it
   correlates with the outcome at p = 0.001.
4. **Option (ii), closed-loop re-record, is the principled fix and matches what was
   already done for the cartesian arm (`derive_cartesian_realized`).** Record what
   `FullTaskEnv(action_mode='delta_joint')` ACTUALLY executes, so demo actions and
   demo states are consistent by construction and no integrator offset exists. This
   is the only option that fixes the "frozen offset" property rather than reducing
   its frequency.
5. **Option (iii), delta for pick-scope only, remains valid as an interim.** The
   keeper trace shows drift onset AFTER the grasp on a demo whose pick is at frame
   964; the current pick-scope experiments are unaffected, which is consistent with
   the existing pick baseline (34) being trustworthy.
6. **Separately: do not report proxy-derived nested at all.** The proxy is a
   training reward shim; it has no settle and no proximity test, and it terminates
   the episode. Any replay-measured nested count is a proxy count. The RULE adopted
   in the handoff (report no contact/nested from delta open-loop replay) should
   stand until (1) and (4) land.

---

## 9. Reproduce

All commands from `/home/j/workspace/genesis_pickaplace`, CPU only.

Scripts live in this session's scratchpad (ephemeral):

    S=/tmp/claude-1000/-home-j-workspace-genesis-pickaplace/8ea5848e-5db6-4a43-bc8e-7e682f9837fc/scratchpad

    # 0. truncation check (no sim)
    ./.venv-eval/bin/python $S/p1_trunc_check.py

    # 1. per-step divergence trace, inner horizon raised (arg2 = raise flag)
    ./.venv-eval/bin/python $S/p1_trace.py $S/p1 1 300 328 248
    ./.venv-eval/bin/python $S/p1_trace.py $S/p1 0 261      # default horizon
    ./.venv-eval/bin/python $S/p1_analyze.py $S/p1          # 200-step band table
    ./.venv-eval/bin/python $S/p1_onset.py   $S/p1          # A-jump / saturation onset

    # 2. horizon control + nested-source instrumentation (arg2 = raise flag)
    ./.venv-eval/bin/python $S/p1_ctrl.py $S/p1/ctrl_raised.jsonl 1 \
        300 328 248 312 320 330 247 256 298 274 305 316 261
    ./.venv-eval/bin/python $S/p1_ctrl.py $S/p1/ctrl_default.jsonl 0 305 274 316 261

    # 3. population statistics (no sim)
    ./.venv-eval/bin/python $S/p1_pop.py        # 1200-horizon correlate, Fisher
    ./.venv-eval/bin/python $S/p1_offline.py    # perfect-tracking encoder emulation
    ./.venv-eval/bin/python $S/p1_apick.py      # offline drift-at-grasp predictor
    ./.venv-eval/bin/python $S/p1_sat2.py       # cap saturation, length-controlled
    ./.venv-eval/bin/python $S/p1_satdensity.py # (withdrawn: length-confounded)

Core of the trace, sufficient to rebuild it:

```python
env = FullTaskEnv(backend='cpu', max_steps=6000, scope='full',
                  action_mode='delta_joint', action_repeat=1)
env.genv.max_steps = 10**9                      # remove the phantom-settle confound
trans     = delta_encode_transitions([p], env.pick_z, 'full', env.delta_cap)
acts      = np.stack([t[1] for t in trans])
demo_nobs = np.stack([t[3] for t in trans])                 # states[i+1]
demo_cmd  = np.stack([t[1] for t in relabel_full([p], env.pick_z)[0]])[:, :6]
env.reset(options={'uid': uid})
for i, a in enumerate(acts):
    obs, r, term, trunc, info = env.step(a)
    tgt = np.asarray(env._dj_target); qm = np.asarray(obs[:6], dtype=np.float64)
    A = np.max(np.abs(tgt - demo_cmd[i]))                   # (a) target drift
    B = np.max(np.abs(qm - tgt))                            # (b) tracking lag
    C = np.linalg.norm(obs[8:11] - demo_nobs[i][8:11])      # (c) world divergence
    if term or trunc: break
```

Raw outputs: `$S/p1/` (`trace_{300,328,248}_r1.npz`, `trace_261_r0.npz`,
`ctrl_raised.jsonl`, `ctrl_default.jsonl`, and the `.log` files).

Cost note: at the DEFAULT inner horizon a >1200-step demo costs ~100x its length in
sim steps. On this box (~12 sim steps/s under load) uid 300 would take ~5 h at the
default horizon and ~5 min raised. That cost is itself a symptom of the bug.

---

## 10. Caveats

- One run per demo per condition; no repeats. The sim is not reproducible across
  processes (uid 305: 1122 vs 1001). Step indices are approximate to ~10%.
- The box was running six RLPD training jobs throughout. The repo's own rule is
  that official numbers come from an idle box; these are diagnostic numbers, not
  headline metrics.
- Per-step traces were run for 3 uids only (300, 328, 248 + 261). The verdict for
  330, 247, 256, 298 is by elimination (truncation and horizon both excluded by
  control), not by direct trace.
- 6 of the 22 nested-labelled demos remain UNRESETTABLE and were not touched here.
  That is still open (handoff §8 item 4).

## Addendum (2026-08-13, newbox_supp verification)

The "primary 400-step evals are clean" claim holds ONLY because
`GenesisCanEnv.reset()` sets `self._t = 0` (baselines/genesis_can_env.py:174) —
the inner counter resets per EPISODE. Had `_t` accumulated across episodes within
one eval process, the 1200 boundary would have been crossed from episode 4 onward
and the primary column would have been contaminated too. "400 < 1200" is
conditional on per-episode reset; verified directly, not assumed.

Also in flight: a paired control for the #26 regression's effect size — two ar4
sensitivity evals ran PRE-fix by launch-timing accident (ar4d_s0/s1, launched
~109s before commit 3a7a713 landed in the shared working tree); both are being
re-run POST-fix under identical protocol. Pre-vs-post delta = the phantom-settle
bug's measured impact at the 1600-step horizon. If they agree, the bug is inert
at this horizon and the 54/72 phase-sweep exposure is bounded.
