# Machine-first arm — execution log (2026-09-07 →)

Registration: `paper/MACHINE_FIRST_PLAN_2026-09-07.md` (+ amendments). Brief: `~/wm_fix_2026-09-03/agent_brief_machine_first.md`. Every number here is from tool output; job ids for everything. Checkout: `.claude/worktrees/machine-first-arm` (branch `worktree-machine-first-arm`), cluster code tree `$LAB/mf_gp` (same branch; data dirs symlinked to the shared checkout).

## 2026-09-07

- 13:30 worktree found 895 commits behind `4dof-cartesian` (no `baselines/rl`, no `cluster/`); rebased onto dd9942b (slip disclosed in plan amendment (a)).
- 13:45 `baselines/mf_measure_phi.py` (local venv, world w3): max|φ(s₀)| train uids 0.4530 (uid 247), rnd300 0.6640, box corners **0.6714** → T1 `return_clamp = 1.6714` registered.
- 13:55 coordinator: preempt QOS saturated → teachers on QOS normal; user priority "if there's capacity try to train SAC" → T2 first.
- 14:20 amendment (a) appended (T2 = `train_rlpd.py --no-demos --pick-shaping on`, 4 × 4e5 decisions; T1 = r2dreamer no-prefill + `env.pick_shaping=true` + clamp 1.6714, 4 × 1e6). Code: `rlpd_sac.py` demo_batch=0 path, `train_rlpd.py --no-demos`, `cluster/mf/mf_sac_teacher.sbatch`.
