# RUN-IDENTITY / DUPLICATION AUDIT — 2026-08-17

Scope: all RLPD/SACfD checkpoint sidecars on this box, wandb project
`jambotime/genesis_paper` (279 runs), r2dreamer local run dirs, and the launch
tooling. Trigger: the pair-dH incident (a full wave re-executed the nb wave and
was counted as an independent replication for days; struck in 2fbed2a).

Method: group every run by (semantic config fields, seed, demo_dir). For each
suspect pair, pull the first ~10 logged values of `train/actor_q_mean` (RLPD)
or `train/actor_loss` (SACfD) from wandb and compare. Bit-identical early
history = same computation.

---

## 1. CONFIRMED DUPLICATES

### 1a. (known, re-verified) pair-dH s0-s2 = nb s0-s2 — bit-identical
`dH_RLPD-pair_s0/s1` vs `dH_RLPD-nb_s0/s1`: `train/actor_q_mean` identical to
the last float at every logged step (s0: -0.615589, -0.581207, ... both runs).
Canonical = **nb** (earlier, 08-14 12:47 vs pair 08-14 19:34). Already struck
in the matrix (2fbed2a). This audit confirms the strike is correct.

### 1b. NEW: clean-long s0/s1/s2 (first 100k) = clean s0/s1/s2 — bit-identical
`dR2D_RLPD-clean-long_s{0,1,2}` vs `dR2D_RLPD-clean_s{0,1,2}`: early
`train/actor_q_mean` identical to the last float for all three seeds
(s0: -0.450214, -0.412237, ...). Cause is structural: git diff
2e75366..3d3129f touches ONLY `ms_env.py` / `train_rlpd_ms.py` (ManiSkill) and
docs — the genesis trainer and env are byte-identical — and both waves predate
the demo-RNG fix, so both used demo-RNG 0. Same code, same seeds, same demo
dir, same streams.

**Status: deliberate and correctly registered, but it is a duplicate
computation.** cell_b doc §7 registered the extension with "config identical
... one variable: --steps" and counts only seeds 3-4 as new 100k-ignition
samples. Rules that follow from this finding:
- A clean-long s0-2 checkpoint at 100k IS the clean s0-2 checkpoint. Any eval
  of it is a re-eval of the same policy, never a replication.
- No pooled rate may count clean (n=3) and clean-long s0-2 as separate seeds.
  The current matrix does not; keep it that way.
- Canonical for the 100k cell = **clean** (earlier, 08-16 01:28).

### 1c. LIVE: the n20 wave (running now) re-executes nb, clean, and exp seeds
`dH_RLPD-n20_s0..s7` and `dR2D_RLPD-n20_s0..s7` started 08-17 16:30 UTC on the
cluster. Seed collisions with completed waves, all at the same config and
demo dir (code unchanged: `git diff ee9ca24..HEAD -- baselines/` is empty):

| n20 cell | collides with | stream relation |
|---|---|---|
| dH s0 | nb_s0 (and struck pair_s0) | ALL streams identical (seed-0 demo back-compat) — 3rd execution of this computation |
| dH s1,s2 | nb_s1,s2 | env/init/IC streams identical; only demo-RNG differs (post-fix) |
| dH s3-s7 | exp_s3-s7 (post-fix, same code) | ALL streams identical |
| dR2D s0 | clean_s0 (= clean-long_s0) | ALL streams identical |
| dR2D s1,s2 | clean_s1,s2 | demo-RNG only differs |
| dR2D s3-s6 | exp_dR2D_s3-s6 | ALL streams identical |
| dR2D s3,s4 also | clean-long_s3,s4 @100k | env/init streams identical; demo-RNG differs |

Empirical check: n20 early curves track their partners to ~1e-3
(dH s3: -0.569878 vs -0.571249; dH s4: -0.504409 vs -0.504293) — the same
computation under cross-machine GPU nondeterminism, NOT an independent draw.

**Required decision (one of two, before any n20 number is pooled):**
1. n20 SUPERSEDES: the pooled per-arm rate uses n20 seeds only; nb, clean,
   clean-long@100k, and exp are excluded from that pooled rate (they remain
   citable as their own waves); or
2. n20 seeds 0-7 are DISCARDED from pooling and the top-up runs seeds 8-19
   only (the round doc's own "higher seeds" wording).
Pooling exp + n20 as written double-counts 8 dH cells and 7 dR2D cells —
the pair-dH incident again, at ~5x the size. The run ledger line
"exp = new seeds only (dH s3-7, clean s3-6)" is also wrong on its face for
dR2D s3/s4: clean-long already ran those seeds (registered as the ignition
n=3 -> n=5 extension). Seeds 3,4 in the dR2D 100k-ignition sample must be
counted ONCE (from exactly one wave, chosen and recorded).

No running process was touched by this audit.

---

## 2. SUSPECTS CLEARED

| suspect pair | verdict |
|---|---|
| SACfD 08-10 wave (`dH/dDP_SACfD_s0-7`, same names+wandb-config+seeds as the 08-02/08-08 wave) | CLEARED — deliberate hardened-predicate retrain (PAPER_PLAN 08-10 night). The semantic change was env-side code, invisible in the wandb config (git stamping in writers landed only 08-13, ec0f4c8). Curves start float-close, then diverge. `results_significance.md` uses only the first wave's evals; "0 (16 seeds)" = 8 dH + 8 dDP of the retrain wave — no double count found. |
| `rlpd_dH_s0` vs `rlpd_dH_s0_fixa` | CLEARED — fixa = fixed alpha (ent_coef 0.005), documented in rlpd_audit_2026-08-14.md. Different config; the thin early sidecar just cannot show it. |
| clean-long s3/s4 vs exp_dR2D s3/s4 | CLEARED as literal duplicates — early curves differ beyond float noise (demo-RNG fix changed the demo stream). Still same env seed: correlated draws, see 1c. |
| ar4 / ar8 / ln / hold / mref / pair-dDP waves | CLEARED — each is a unique (config, seed) cell within the RLPD family. |
| ms* RLPD runs (msctl / msdense / msknobs) | CLEARED — rich sidecars (incl. seed + git); configs differ (reward_mode / gamma / utd / horizon). No collisions. |
| old SACfD dirs (sacfd_all*, sacfd_full*, sacfd_broad, sacfd_cart*) | No sidecars exist (pre-sidecar era). No same-name collisions in wandb. Formally unauditable from disk; noted, low stakes (superseded era). |

## 3. r2dreamer run identity

Local runs DO carry sidecar-equivalent metadata: `runs/*/.hydra/config.yaml`
(includes seed). Hashing (config minus logdir) x seed over 35 run dirs finds
two collision groups:
- `genesis_place_v5/v6/v7/v7b/v8/v8b_s0` — six runs, identical config, seed 0.
- `pick_dH_v4_r100_s0` vs `pick_dH_v5_honest_s0` — identical config, seed 0.

These were sequential code-iteration reruns (env/reward edits live in code,
not config), and none is pooled as a replication. But the disk record cannot
PROVE they differ: hydra config stamps no git hash and no demo fingerprint.
**Verdict: r2d local run identity is auditable at (config, seed) level only;
semantic identity is unauditable from disk.** Cluster r2d runs (waves s20-29,
s30-39) are not on this disk at all — unauditable here; their identity rests
on wandb + the sbatch log. Harvest manifests do stamp `git_r2dreamer` (good).

## 4. TOOLING VERDICT: NOT recurrence-proof

A duplicate is preventable iff (config, seed, demo_dir, git) is stamped AND
some check refuses/warns on re-use. Current state:

| tool | stamps | refuses re-use? |
|---|---|---|
| `train_rlpd.py` sidecar | action_mode, action_repeat, delta_ref, backup_entropy, per_member_ln, pick_hold_*, scope, demo_dir, git — but **NOT seed**, and not gamma/utd/ensemble_size/subset_size/demo_batch/steps | NO — `out.mkdir(exist_ok=True)`, no registry, no check |
| `cluster/sbatch_rlpd.sh` | git gate (>= 2fbed2a) + demo-provenance pattern gate | NO — nothing compares against previously executed (config, seed) cells; its own header example (`seq 0 19`) re-executes already-run seeds, which is happening right now (1c) |
| `cluster/sbatch_r2dreamer.sh` | .claim double-submission guard | Only against CONCURRENT writers on one LOGDIR. A sequential re-execution, or the same cell under a new run name, passes. |

The seed — the field whose reuse defined both confirmed incidents — is the one
semantic field no genesis-side sidecar records. It survives only in directory
and run names.

## 5. Minimal prevention proposal (NOT implemented)

1. **Complete the sidecar**: add `seed`, `gamma`, `utd`, `ensemble_size`,
   `subset_size`, `demo_batch`, `steps`, and a demo-dir content fingerprint
   (sha256 over sorted (filename, size)) to the train_rlpd sidecar dict.
   ~6 lines. The fingerprint catches both same-path-different-contents and
   different-path-same-contents.
2. **Identity key + registry**: key = sha256 of the canonicalized
   (semantic-config, seed, demo_fingerprint, git). At trainer startup, append
   one JSON line {key, out_dir, run_name, date} to a committed, append-only
   `baselines/rl/checkpoints/RUN_REGISTRY.jsonl`. If the key already exists,
   REFUSE to start unless `--duplicate-ok "<reason>"` is passed; record the
   reason in the registry line and the sidecar. (Registered same-seed
   extensions like clean-long stay possible — with a written reason, which is
   exactly the paper-trail the incidents lacked. Note git is part of the key,
   so a doc-only commit changes the key; the startup check should therefore
   also WARN on a match of (semantic-config, seed, demo_fingerprint) with
   different git, and refuse only on full-key match — the warn covers the
   nb->pair and exp->n20 cases, both of which crossed doc-only commits.)
3. **r2d**: launcher writes {git, demo_fingerprint} into the run dir next to
   `.hydra/` (one line in the sbatch and local launcher).
4. **Analysis side**: any script that pools seeds reads the registry and
   asserts the pooled cells have distinct (semantic-config-minus-git, seed,
   demo_fingerprint) triples. This makes double-counting mechanically
   impossible rather than discipline-dependent.

Item 2's warn-vs-refuse split is the load-bearing design point: every
confirmed duplicate in this project crossed at least one commit, so a
git-inclusive key alone would have caught none of them.
