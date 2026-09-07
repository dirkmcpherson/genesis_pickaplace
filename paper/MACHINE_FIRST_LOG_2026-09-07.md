# Machine-first arm — execution log (2026-09-07 →)

Registration: `paper/MACHINE_FIRST_PLAN_2026-09-07.md` (+ amendments). Brief: `~/wm_fix_2026-09-03/agent_brief_machine_first.md`. Every number here is from tool output; job ids for everything. Checkout: `.claude/worktrees/machine-first-arm` (branch `worktree-machine-first-arm`), cluster code tree `$LAB/mf_gp` (same branch; data dirs symlinked to the shared checkout).

## 2026-09-07

- 13:30 worktree found 895 commits behind `4dof-cartesian` (no `baselines/rl`, no `cluster/`); rebased onto dd9942b (slip disclosed in plan amendment (a)).
- 13:45 `baselines/mf_measure_phi.py` (local venv, world w3): max|φ(s₀)| train uids 0.4530 (uid 247), rnd300 0.6640, box corners **0.6714** → T1 `return_clamp = 1.6714` registered.
- 13:55 coordinator: preempt QOS saturated → teachers on QOS normal; user priority "if there's capacity try to train SAC" → T2 first.
- 14:20 amendment (a) appended (T2 = `train_rlpd.py --no-demos --pick-shaping on`, 4 × 4e5 decisions; T1 = r2dreamer no-prefill + `env.pick_shaping=true` + clamp 1.6714, 4 × 1e6). Code: `rlpd_sac.py` demo_batch=0 path, `train_rlpd.py --no-demos`, `cluster/mf/mf_sac_teacher.sbatch`.
- 13:52 T2 smoke 3349557 (2k decisions, QOS normal): `[cfg] demo_batch=0/256 pick_shaping=on`, no-demos line, `rlpd_final.zip` + `ckpt_100` written, critic loss 0.009; the launcher's post-train check then died on its own .out-name assumption (`-J` changed it) — fixed (`$SLURM_JOB_NAME`), `EVAL_ONLY=1` mode added; eval stage re-scored post hoc as CPU job 3349694.
- 13:5x T1 smoke 3349599 died at hydra struct mode (`env.pick_shaping` is not a key of `genesis_pick_state`) → override is `+env.pick_shaping=true`; resubmitted 3349626: trained 5k steps, resolved config `pick_shaping: true / return_clamp: 1.6714 / demo_dir: null / actor_dist bounded_normal / act_entropy 3e-5`, a tipped episode scored 0.5 (shaping active), then my shaping-check script crashed on an empty metric list (no `reward_sum` rows at 5k steps) → check made robust; smoke2 = 3349737.
- **14:03 T2 SUBMITTED (QOS normal, `-p gpu`): mf_sac_t2 seeds 0–3 = jobs 3349688 / 3349689 / 3349690 / 3349691, 4e5 decisions each** (all on pax050).
- 14:1x harvest tooling: `cluster/mf/mf_harvest.sbatch` (66 human ICs from the dHv2raw manifest, `--mode sample --ic-mode demo --attempts 3 --verify`, 6 shards, r2d venv, R2DREAMER_ROOT = r2dreamer_fix), `baselines/mf_build_drl.py` (one success per IC, first by attempt; make_matched_sets writer), `sbatch_rlpd.sh` allow-list + `dRL`; `record_demos.R2DTeacher` now feeds `state` for state-obs checkpoints. Loader test on the smoke checkpoint (shard 0, 1 attempt) job 3349744; random-teacher negative control (6 shards, 66 ICs, 1 attempt) job 3349745.
