# Tip-rule clips, 2026-09-11 (Lane 6)

Nine annotated demonstration clips rendered with the tip rule **DISABLED**, so each tape runs
past the point where the live rule would have ended it and you can see what the rule cuts off.
Full measurement: `paper/TIP_RULE_2026-09-11.md`. The env class was not changed — the threshold
is overridden on the env INSTANCE (`annotate_demos.py --no-tip`), and `build_env` asserts
`FullTaskEnv.TIP_DEG` is still 60.0.

**How to read the new lines.** Below the usual diagnostics row:

```
can tilt  60.7 deg   (tip rule: >60 deg AND grip<0.3)        RULE FIRES @d228
```

The tilt turns cyan above 60° and red at ≥ 80° (horizontal). `RULE FIRES @dN` appears in red
from the decision the LIVE rule would have terminated on, and a red tick marks it on the
progress bar. The terminal card adds `can tilt after the settle NN.N deg` and the firing
decision with its tilt, commanded grip and `in_hand`.

`in_hand` is the tracker's lever test (|tool_xy − can_xy| < 0.025 m) — the same test
`nested_v2` uses. `grip` is the COMMANDED grip, 0 = fully open, and the tip rule's guard is
`grip < 0.3`.

---

## The two clips the user was most likely looking at

| clip | ic | what it shows |
|---|---|---|
| `human_262_fires_in_hand.mp4` | human 262 | The rule fires at **decision 228 of 228** — the tape's last decision — at **60.7°**, with the can still **in the hand** (lever 0.021 m) and grip commanded 0.23. Nothing is cut off. Watch d210 onward: the can is upright on the shelf, the gripper opens around it, the arm withdraws and drags it over. Settles at **77.2°** — the only tape in the whole corpus that does not end flat or upright. |
| `machine_308_fires_at_60.mp4` | machine 308 | Fires at **decision 227 of 229** at **60.4°**, can free. Two decisions are cut. With the rule off the can goes to **91.1°** and settles at 90°. `contact_push` was already granted at d221, so the rule takes no reward. |

## Every firing that happens while the can is still in the gripper (6 of 25)

This is the "fires too early" population that actually exists: at the firing frame the can is
within the held-lever distance of the tool point, so the chip lights on a *leaning release*, not
on a can lying on the shelf.

| clip | ic | fires | tilt | grip | lever (m) | ends |
|---|---|---|---|---|---|---|
| `human_262_fires_in_hand.mp4` | human 262 | d228 / 228 | 60.7° | 0.23 | 0.021 | settles 77.2° |
| `human_235_fires_in_hand.mp4` | human 235 | d138 / 138 | 68.0° | 0.18 | 0.018 | 81.2° in tape, settles 90° |
| `human_239_fires_in_hand.mp4` | human 239 | d255 / 255 | 66.5° | 0.19 | 0.018 | 92.3° in tape, settles 90° |
| `machine_239_fires_in_hand.mp4` | machine 239 | d105 / 105 | 60.2° | 0.28 | 0.002 | 86.3° in tape, settles 90° |
| `machine_250_fires_in_hand.mp4` | machine 250 | d493 / 496 | 62.4° | 0.02 | 0.020 | 86.9° in tape, settles 90° |
| `machine_252_fires_in_hand.mp4` | machine 252 | d409 / 409 | 90.0° | 0.27 | 0.013 | settles 90° (never picked) |

`machine_250` is also the **only tape in all 146** whose can returns upright from past 60° while
free: at d273 it touches 67.7° and at d274 it is back to 18.8°. The rule of record does not fire
there (grip was 0.45), and a 1-decision sustain would remove it under any guard. It then tips
for real at d493.

## Falls the rule NEVER labels (2 of 29)

The other half of the finding: a can lying flat with the fingers commanded closed is invisible
to the rule. These clips have **no** `RULE FIRES` tag — the card says *tip rule never met on
this tape*.

| clip | ic | can is horizontal from | commanded grip there | tape continues for |
|---|---|---|---|---|
| `human_243_never_fires.mp4` | human 243 | d88 / 170 | 0.41 | 82 more decisions |
| `machine_301_never_fires.mp4` | machine 301 | d164 / 600 | 0.39 | **436 more decisions**, to the cap |

---

## What is NOT here

The prompt asked for a clip of **every tape where the current rule fires but the can later
recovers**. That set is **empty**: zero of the 25 tapes recover after the trigger, at any
threshold from 45° to 89° (`paper/TIP_RULE_2026-09-11.md` §4). The in-hand firings and the
misses are shipped instead, because those are the defects the data actually contain.

There are no POLICY clips here either. `paper/TIP_RULE_2026-09-11.md` §8 measures the same
questions on {RLPD} pilot rollouts and finds a different picture: there the rule fires almost
entirely **before the pick** (19 of 22 firings, median decision 12–21 of 300) and **never once
with the can in the gripper** — so the "too early" case these clips show is a property of human
release behaviour, not of anything a policy does. All 22 of those cans end flat.

## Reproduce

```bash
V=~/workspace/genesis_sim2real/venv/bin/python
$V baselines/annotate_demos.py render --in <dHfull_all> --src-dir <src_dHfull_all> \
   --out-dir can_pos_recovery/videos_tip_2026-09-11 --set-name human --no-tip --procs 4 \
   --tapes genesis-100003-016-229.npz genesis-103000-037-139.npz \
           genesis-106000-058-256.npz genesis-100001-014-171.npz
$V baselines/annotate_demos.py render --in <dDPfull_first> --src-dir <src_dDPfull> \
   --out-dir can_pos_recovery/videos_tip_2026-09-11 --set-name machine --no-tip --procs 5 \
   --tapes genesis-105000-050-106.npz genesis-105003-051-497.npz genesis-107003-016-410.npz \
           genesis-105016-055-230.npz genesis-100018-022-601.npz
```

The renderer names a clip by the class it computes WITH THE RULE OFF, so the files were renamed
by hand to say why each one is here; `_rendered_{human,machine}.json` in this directory carries
the unrenamed rows, including `tip_ref_decision`, `tip_ref_tilt_deg`, `tip_ref_grip`,
`tip_ref_in_hand` and `final_tilt_deg` for each tape.
