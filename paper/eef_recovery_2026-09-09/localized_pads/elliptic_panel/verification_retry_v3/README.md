# Numerical verification follow-up

The original panel had33 completed full replays and three UID181 stops on a
1e-5 output-scaled reference check. All original logs and manifests are preserved.

The first native-order verifier (v2) completed three of four replays. Its rigid1
run encountered another tangential output-scaled error1.255e-5.181 soft1 exactly
reproduced all14 original saved arrays. These three full replays and one stopped
attempt remain separate from the original panel.

The final verifier (v3) reconstructs the simulator's body/ancestor/DOF order and
compares every reconstructed Jacobian entry independently at1e-6. Its reference
check divides error by the magnitude of the terms entering the velocity sum,
rather than a potentially cancelled output. The input-scaled threshold remains
1e-5. This is a declared change to numerical error scaling, not a task metric or
physics change; old output-scaled and ascending-sum errors remain in the audit.

All five v3 replays completed. Maximum Jacobian entry discrepancy is5.96e-8;
all input-scaled reference/target errors are below2.72e-7. Input/output scales
can differ by85x. The two required controls,181 soft1 and176 soft2, reproduce all
14 archived arrays exactly. This supports floating-point accumulation/cancellation
as the source of the stops, rather than a mismapped contact row. The physical
normal update is unchanged. No installed engine source was edited.

The completed nine-demo comparison is in `merged_summary.json`:

| Configuration | Supplied metric | Strict sequence |
| --- | --- | --- |
| Original fixed | 3/9 | 2/9 |
| Rigid, normal1 | 2/9 | 0/9 |
| Soft, normal1 | 2/9 | 1/9 |
| Rigid, normal2 | 2/9 | 1/9 |
| Soft, normal2 | 2/9 | 2/9 |

All four181 conditions fail arrival and the supported push-to-contact sequence.
No additional strict recovery over original; no adoption. The goal-motion and
historical goal-calibration qualifications in the parent report still apply.
The experiment now totals222 completed full replays across the same12 demos,
including three v2 and five v3 repeats. These repeats are not new independent
recordings. Four numerical-stop attempts across original/v2 are separate.
