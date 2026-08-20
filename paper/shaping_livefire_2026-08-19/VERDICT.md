# Shaping live-fire verdicts (2026-08-19 night, dev box)
Harness: record_trace.py (FullTaskEnv, repeat 4) + record_r2d_trace.py
(r2dreamer GenesisPick adapter, repeat 4, reward_scale 100). One process per
env (Genesis single-init). Comparator asserts: dynamics bit-identical
shaped-vs-plain, and reward difference == gamma*phi(s')-phi(s) at the agent
boundary (x100 for the r2d adapter).

PRE-FIX (per-sim-step application): FAIL — max_err 3.96e-3 (gamma 0.997) /
1.32e-3 (gamma 0.999) per agent step = the -(1-gamma)*sum(phi_substeps)
residual (reward for staying FAR from the can).

POST-FIX:
- FullTaskEnv repeat-4 (covers dv3 env-side repeat + RLPD repeat-1):
  max_err 0.00e+00 at gamma 0.997 AND 0.999; shaping fired 60/60 steps. PASS.
- r2dreamer adapter boundary (gamma 0.999, scale 100): max_err 2.38e-07,
  shaping fired 40/40 agent steps, dynamics identical. PASS.
- dv3 live train (exact cluster config stack): ran end-to-end;
  train_return 55-114 with log_picked 0 = potential term flowing. PASS.
- r2d live train through the patched adapter: prefill 268 eps @ scale 100,
  online steps clean. PASS.
