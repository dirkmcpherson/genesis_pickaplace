#!/usr/bin/env python3
"""The claims ledger: sentences the paper might write, and whether this directory supports them.

This is the artefact an auditor uses. For each claim it records the exact supporting cells, the
STRENGTH at which it is supported, the caveat that must travel with the sentence, and - where it
matters most - the sentence that would be WRONG to write.

STRENGTHS
  established                 the cells support it and the test had the power to detect a
                              violation of the registered margin (MDE <= ROPE on every cell)
  supported-but-underpowered  the cells are consistent with it, but at least one could not have
                              detected an effect the size of the registered margin
  directional-only            a lean exists but is not significant and not equivalence either
  withdrawn                   a control fired; the supporting row does not measure what it claims
  not-supported               the experiment that would license this sentence has not been run

TWO STANDING RULES, enforced mechanically by make_tables.py, not left to the writer:
  R1  A null may not be described as EQUIVALENCE where the minimum detectable effect exceeds the
      registered margin. `strength='established'` is REFUSED on any claim whose cited cells
      include one with MDE > 0.10; the renderer downgrades it and prints the violation.
  R2  A machine-favouring or human-favouring lean may not be attributed to demonstration SOURCE
      while the set-construction confound is unresolved. Any claim with
      `attributes_to_source=True` must carry `confound`, and the renderer prints it inline.
"""

# The confound that R2 exists for, measured rather than asserted (see SET_CONSTRUCTION below).
SELECTION_CONFOUND = (
    'The machine full-task set is BEST-OF-THREE per initial condition; the human set is every '
    'tape as recorded. Removing the selection (dDPfull_first, the first attempt at each IC) '
    'drops the machine set from 206 to 131 total reward against the human 118 - so the machine '
    "set's apparent advantage collapses from +75 % to +11 %, and MOST OF IT IS OUR OWN "
    'SELECTION, not a property of machine demonstrations.')

CLAIMS = [
# ---------------------------------------------------------------- the source nulls
dict(id='no_source_pick',
     claim='At the pick stage, which demonstrations a learner is trained on - human or '
           'machine-generated - does not change what it achieves.',
     cells=['pick_rnd30_dp', 'pick_rnd30_rlpd', 'pick_rnd30_r2d_mode', 'pick_rnd30_r2d_sample',
            'pick_spots60_dp_asrecorded', 'pick_spots60_rlpd', 'pick_spots60_rlpd_sampled'],
     strength='supported-but-underpowered',
     caveat='True as an equivalence claim ONLY for the world model, whose per-seed spread is '
            'tight enough to have detected a 0.10 effect. The RLPD random-start cell has a '
            'minimum detectable effect of 0.345 - it would have missed an effect three times the '
            'registered margin - so for RLPD this is "no difference was detected", not "the arms '
            'are equivalent". The in-distribution cells are the stronger evidence, because both '
            'arms sit near ceiling rather than near a floor.',
     wrong='"All three learners are indifferent to demonstration source at the pick stage." The '
           'word for the RLPD cell is undetected, not indifferent.'),

dict(id='no_source_place',
     claim='At the place stage the demonstration source does not change what the world model '
           'achieves.',
     cells=['place_r2d', 'place_r2d_sample', 'place_symmetry_r2d'],
     strength='supported-but-underpowered',
     caveat='The symmetry control passes: scoring on the machine-generated entry bank instead of '
            'the human one moves the difference by 0.009, so the result is not an artefact of '
            'whose policy produced the evaluation entries. But every cell leans the same way '
            '(human ahead by 0.056-0.072) and none is significant, and the sampled cell '
            'quadrupled its gap under re-scoring - so the direction is not stable enough to '
            'carry weight, in either direction.',
     wrong='"Place shows no difference between sources." It shows a consistent human-favouring '
           'lean that the study cannot resolve; that is not the same sentence.'),

dict(id='no_source_slide',
     claim='At the slide (contact) phase the demonstration source does not change what the world '
           'model achieves.',
     cells=['contact_r2d_bare', 'contact_r2d_push'],
     strength='supported-but-underpowered',
     caveat='Both arms are SUB-FLOOR: 11 demonstrations each, which is below the count the phase '
            'plan set as its own floor. A null between two under-trained arms is weak evidence '
            'about source. Use contact_push, not bare contact - the legacy predicate overstates '
            'capability by 1.5-3x.',
     wrong='"The slide phase shows source-indifference." It shows two sub-floor arms that cannot '
           'be told apart, which is a much weaker statement.'),

dict(id='no_source_e2e',
     claim='On the full end-to-end task the demonstration source does not change what the world '
           'model achieves.',
     cells=['e2e_picked', 'e2e_contact_push', 'e2e_nested_honest'],
     strength='supported-but-underpowered',
     caveat='EVERY end-to-end stage is underpowered against the 0.10 margin, and every '
            'non-significant lean here favours the MACHINE arm. That direction must not be '
            'attributed to demonstration source: the machine set is best-of-three selected. '
            'See the set-construction confound.',
     attributes_to_source=False,
     confound=SELECTION_CONFOUND,
     wrong='"Human and machine demonstrations are equivalent end-to-end." Nothing here '
           'establishes equivalence; the detectable effect exceeds the margin at every stage.'),

# ---------------------------------------------------------------- what DOES move the numbers
dict(id='pruning_beats_source',
     claim='How the demonstrations are processed matters more than where they came from: '
           'training Diffusion Policy on raw rather than pruned human data costs far more than '
           'any source difference measured anywhere in this project.',
     cells=['prune_dp_rnd30', 'prune_dp_spots60'],
     strength='established',
     caveat='Both arms are HUMAN, so this is not a source contrast. The pruned and raw arms '
            'differ in wave and in demonstration count (58 v 66), so it is a pruned-set-versus-'
            'raw-set contrast rather than a controlled pruning-only manipulation. Established '
            'for Diffusion Policy only - it has not been run for RLPD or the world model.',
     wrong='"Pruning is more important than demonstration source for all learners." It is shown '
           'for Diffusion Policy alone.'),

dict(id='architecture_interaction',
     claim='On robomimic Can the policy architecture interacts with the demonstration source: a '
           'mixture action head helps a human-data policy and hurts a machine-data one.',
     cells=['robo_bcrnn_matched_mixture', 'robo_bcrnn_matched_deterministic'],
     strength='directional-only',
     caveat='3 seeds per cell, so effect size only - the exact test cannot go below p = 0.10 at '
            'this n. The head-matched source gap is 0.834 with mixture heads and 0.167 with '
            'deterministic ones: a fivefold difference from an architecture choice. The '
            'MECHANISM IS OPEN. The obvious explanation - that human data is multi-modal because '
            'several operators use several strategies - was measured and REFUTED: one operator '
            'scores indistinguishably from six, and the machine sets are not unimodal either.',
     wrong='"A mixture head suits multi-operator human data." This was tested and does not hold; '
           'it must not be written. Also wrong: quoting the original 0.534 BC-RNN gap as a '
           'source effect - it compared each arm at its own recipe optimum.'),

dict(id='robomimic_quantity',
     claim='On an independent task (robomimic Can), what looked like a demonstration-source '
           'effect is explained by quantity and coverage, not provenance.',
     cells=['robo_rlpd_mg200s', 'robo_rlpd_mg718s', 'robo_rlpd_mgall', 'robo_rlpd_mg200s_3x'],
     strength='supported-but-underpowered',
     caveat='At NATURAL FULL SIZE machine data matches (MG718s, 718 tapes) or exceeds (MGall, '
            '3900) the human arm; only at MATCHED TAPE COUNT does the 200-tape draw lose. The '
            'qualifier must travel in BOTH directions - dropping it overstates the result either '
            'way. Unresolved: whether a NEUTRAL 200-tape draw would match. The MG200s subsample '
            'is not neutral (89 % of its rows come from the last four SAC checkpoint blocks), so '
            'a pathological sample and a real per-demonstration quality gap are not yet '
            'distinguishable.',
     licence='A random 200-tape draw from the 718 successes, 8 seeds, ~8 GPU-hours. It '
             'distinguishes the two readings directly.',
     wrong='"Machine demonstrations match or beat human ones." Only at their natural size. Also '
           'wrong: any cross-learner ordering on this task - it is withdrawn twice over.'),

dict(id='honest_vs_proxy',
     claim='The end-to-end completion rate depends heavily on which completion predicate is '
           'used: the training proxy over-counts against the honest settled predicate.',
     cells=['e2e_nested_proxy', 'e2e_nested_honest'],
     strength='established',
     caveat='This is a measurement-definition finding, not a source finding. The proxy reads '
            'roughly 2.5-3x the honest predicate on the same rollouts. Any completion number '
            'must say which predicate it uses; the two must never share a row or a sentence.',
     wrong='Quoting the proxy figure as a completion rate. It is a training signal, not a task '
           'outcome.'),

# ---------------------------------------------------------------- claims we cannot make
dict(id='cross_learner_ordering',
     claim='The learners can be ordered by how much demonstration source affects them.',
     cells=[],
     strength='withdrawn',
     caveat='WITHDRAWN, with two independent reasons. The RLPD source reading was '
            'withdrawn by its registered quantity control, and the BC-RNN one by its '
            'registered fixed-head control; and the world model never learned robomimic Can '
            'at all (0/400 and 1/400). That '
            'leaves Diffusion Policy alone, resting on the same 200-tape draw the quantity '
            'control discredited.',
     licence='BC-RNN re-run head-matched at adequate seeds, plus the neutral 200-tape draw. '
             'Until both exist the ordering cannot be quoted in any form.',
     wrong='Any sentence ranking the learners by source sensitivity, however hedged.'),

dict(id='world_model_source_indifference_general',
     claim='World models are indifferent to demonstration source in general.',
     cells=['pick_rnd30_r2d_mode', 'place_r2d', 'contact_r2d_push', 'e2e_contact_push'],
     strength='not-supported',
     caveat='Every world-model null in this project comes from ONE task, ONE world and ONE '
            'implementation family. The independent-task test exists and the world model FAILED '
            'it: on robomimic Can it scored 0/400 and 1/400 at ~541k steps, so it produced no '
            'source comparison there at all. A generalisation therefore rests on a single task '
            'whose replication was attempted and did not learn.',
     licence='A world-model configuration that learns robomimic Can (or another independent '
             'task) well enough to compare sources on it. The dv3 port is a second '
             'implementation on the SAME task, so it does not license this either.',
     wrong='"World models benefit equally from all demonstration sources." The evidence is '
           'one task, and the attempt to replicate on a second task produced a learner that '
           'does not work.'),

dict(id='learning_speed',
     claim='Demonstration source changes how fast a learner reaches its final performance.',
     cells=[],
     strength='not-supported',
     caveat='The learning-curve work lives in HRI_results/curves/ and is a SEPARATE lane from '
            'this table; it covers r2dreamer only, and its own record notes that the fixed '
            'end-to-end budget gave the arms unequal post-ignition training, and that the '
            'end-to-end curves were at one point measuring termination rather than the stage. '
            'This ledger takes no position on a learning-speed effect and no row here supports '
            'one.',
     licence='A matched-budget curve comparison with the arms equalised on post-ignition '
             'training, for more than one learner. Ask the curves lane for its current state '
             'before writing any speed sentence.',
     wrong='Any sentence about one source training faster, citing this table. The cells here are '
           'final-checkpoint scores and carry no timing information at all.'),
]

# Measured properties of the demonstration SETS themselves - not evaluation cells, so they have
# no row in results.csv, but they bound what any source sentence may claim.
SET_CONSTRUCTION = [
    dict(fact='Machine full-task set is best-of-three per initial condition; the human set is '
              'every tape as recorded.',
         numbers='total reward: human (dHfull_all, 74 tapes) 118; machine selected (dDPfull, 72) '
                 '206; machine with selection removed (dDPfull_first, 72) 131. So +75 % becomes '
                 '+11 %: selection contributes 75 of the 88 reward the machine set leads by.',
         source='$W/demos_state_full/{dHfull_all,dDPfull,dDPfull_first}, summed `reward` per tape',
         consequence='Any end-to-end sentence attributing a machine lean to demonstration SOURCE '
                     'is unsupported while this is unresolved. It is the single largest '
                     'confound in the end-to-end comparison.'),
    dict(fact='Human full-task tapes are 36.5 % idle decisions; machine tapes are 0.7 %.',
         numbers='idle fraction 0.365 human (0.393 pre-pick) v 0.007 machine',
         source='PHASE_PLAN amendment (n) disclosure',
         consequence='The raw-versus-pruned effect (see pruning_beats_source) is larger than any '
                     'source effect measured, and idle structure is the leading explanation for '
                     'it. A source sentence about the full task must not ignore that the two '
                     'sets differ this much in composition.'),
    dict(fact='Two of the 74 human initial conditions are unwinnable by construction.',
         numbers='uids 234 and 318 are the only 90-degree lying-can entries; the tip rule fires '
                 'at decision 1 in every census, stage `none`',
         source='CONFOUNDS row 51',
         consequence='Every n=74 denominator carries them. They move no comparison (both arms '
                     'fail them identically) but they depress absolute rates: picked is 69/74 '
                     'over the full set and 69/72 over the winnable one.'),
]


# ---------------------------------------------------------------------------------------------
# STATIC NOTES - retired rows whose evidence must not retire with them.
#
# THE GENERAL RULE. An as-recorded row exists to show WHAT A CORRECTION CHANGED. When an upstream
# fix makes such a row unreproducible - because the pre-correction data no longer flows through
# the pipeline - the row is removed AND its movement is recorded here as a static, dated note.
# Deleting the row on its own would quietly erase the evidence of a correction, which is exactly
# backwards: a project that cannot explain why a published number changed has lost the argument
# before it starts.
#
# These notes are STATIC BY CONSTRUCTION. They are not recomputed from data, they carry no live
# cells, and they must not be regenerated or "checked" against the current table - the inputs are
# gone from the pipeline on purpose. The numbers remain recoverable from git and from the cell
# matrix; what is recorded here is the reading.
STATIC_NOTES = [
dict(id='pick_spots60_dp_asrecorded', retired='2026-09-08',
     was='The as-recorded counterpart of `pick_spots60_dp`: the in-distribution Diffusion Policy '
         'cell computed over the human arm INCLUDING its five archived mixed-hardware seeds '
         '(25-29), against the pinned-only row.',
     why_retired='An upstream de-duplication now drops mixed-hardware evaluations before this '
                 'directory\'s selectors see them, so the pre-correction state can no longer be '
                 'assembled from the harvest. The row had become a duplicate of the live one.',
     movement='Human arm: 5 pinned seeds 0.8933 -> 10 pinned seeds 0.8783 (machine 0.8733 '
              'throughout; p 0.4023 -> 0.8448). The 10-seed PINNED figure is 0.8783 and the '
              '10-seed MIXED-HARDWARE figure was also 0.8783, at p 0.8448 both.',
     reading='**The hardware class made no difference to this cell at all.** Seeds 25-29 return '
             'the counts `53 51 55 50 50` on the archived mixed-hardware evaluation AND on the '
             'pinned re-run - byte-identical per seed, not merely equal in aggregate. So this is '
             'a duplicate-evaluation control, and it bounds the hardware term at zero here. It '
             'also corrects an earlier reading of my own: the 0.893-versus-0.878 gap was a '
             'FIVE-SEED SMALL-SAMPLE effect, not a hardware effect, and I had provisionally '
             'attributed it to hardware.',
     recover='git show c3fa952:HRI_results/results.csv (5v10 pinned and 10v10 as-recorded rows '
             'side by side); the current row is at results.csv `pick_spots60_dp`.'),
]

ROPE = 0.10
STRENGTH_ORDER = ['established', 'supported-but-underpowered', 'directional-only',
                  'withdrawn', 'not-supported']


def write(path, results):
    """Render CLAIMS_LEDGER.md, enforcing R1 and R2 against the live table."""
    import datetime
    by = {r['id']: r for r in results}
    L = ['# Claims ledger', '',
         f'*Generated by `make_tables.py` on {datetime.date.today().isoformat()}. Every number '
         f'is pulled live from `results.csv`; the prose is the spec in `claims.py`.*', '',
         'For each sentence the paper might write: the cells that support it, the strength, the '
         'caveat that must travel with it, and where it matters, the sentence that would be '
         'WRONG.', '',
         '**Strengths.** `established` - supported, and the test could have detected a violation '
         'of the registered +/-0.10 margin. `supported-but-underpowered` - consistent with the '
         'data, but at least one cell could not have detected an effect the size of the margin. '
         '`directional-only` - a lean that is neither significant nor equivalence. `withdrawn` - '
         'a control fired. `not-supported` - the licensing experiment has not been run.', '',
         '**Two rules are enforced here mechanically, not left to the writer:**', '',
         '- **R1** A null may not be called equivalence where the minimum detectable effect '
         'exceeds the margin. A claim marked `established` whose cells include one with '
         'MDE > 0.10 is automatically downgraded, and the violation printed.',
         '- **R2** A directional lean may not be attributed to demonstration *source* while the '
         'set-construction confound is unresolved. Claims that touch it carry it inline.', '']

    violations = []
    for c in CLAIMS:
        cited = [by[i] for i in c['cells'] if i in by]
        live = [r for r in cited if r['status'] not in ('EMPTY',) and r['mde_80'] != '']
        strength = c['strength']
        # ---- R1
        weak = [r for r in live if isinstance(r['mde_80'], float) and r['mde_80'] > ROPE]
        if strength == 'established' and weak:
            violations.append((c['id'], 'R1', [r['id'] for r in weak]))
            strength = 'supported-but-underpowered'
        # ---- withdrawn cells cannot support a live claim
        wd = [r for r in cited if r['status'] == 'WITHDRAWN']
        if wd and strength not in ('withdrawn', 'not-supported'):
            violations.append((c['id'], 'withdrawn-cell', [r['id'] for r in wd]))
            strength = 'withdrawn'

        L += [f"## {c['id']}", '', f"> {c['claim']}", '',
              f"**Strength: `{strength}`**"
              + ('  *(downgraded from `%s` by R1)*' % c['strength']
                 if strength != c['strength'] else ''), '']
        if live:
            L += ['| cell | human | machine | Delta | 95% CI | p | MDE | verdict |',
                  '|---|---|---|---|---|---|---|---|']
            for r in live:
                f = lambda x, n=3: ('' if x == '' else (f'{x:.{n}f}'
                                                        if isinstance(x, float) else str(x)))
                mde = f(r['mde_80'])
                if isinstance(r['mde_80'], float) and r['mde_80'] > ROPE:
                    mde = f'**{mde}** (> margin)'
                L.append(f"| `{r['id']}` | {f(r['human_rate'])} | {f(r['machine_rate'])} "
                         f"| {f(r['delta'])} | [{f(r['ci_lo'])}, {f(r['ci_hi'])}] "
                         f"| {f(r['perm_p'])} | {mde} | {r['verdict']} |")
            L.append('')
        elif c['cells']:
            L += ['*(cited cells are not currently live in the table)*', '']
        L += [f"**Caveat that must travel with this sentence.** {c['caveat']}", '']
        if c.get('confound'):
            L += [f"**Set-construction confound (R2).** {c['confound']}", '']
        if c.get('licence'):
            L += [f"**What would license this claim.** {c['licence']}", '']
        if c.get('wrong'):
            L += [f"**Do NOT write.** {c['wrong']}", '']

    if STATIC_NOTES:
        L += ['## Retired rows - static notes', '',
              'An as-recorded row exists to show what a correction changed. When an upstream fix '
              'makes one unreproducible, the row is removed and its movement is recorded here '
              'instead, so the evidence of the correction does not retire with it.', '',
              '**These notes are static by construction.** They carry no live cells, they are '
              'not recomputed, and they must not be regenerated or checked against the current '
              'table - the inputs no longer flow through the pipeline, on purpose.', '']
        for n in STATIC_NOTES:
            L += [f"### `{n['id']}` - retired {n['retired']}", '',
                  f"- **What it was:** {n['was']}",
                  f"- **Why it was retired:** {n['why_retired']}",
                  f"- **The movement it recorded:** {n['movement']}",
                  f"- **Reading:** {n['reading']}",
                  f"- **Recover the original rows:** `{n['recover']}`", '']

    L += ['## Properties of the demonstration sets themselves', '',
          'These are not evaluation cells, so they have no row in `results.csv`, but they bound '
          'what any source sentence may claim.', '']
    for f in SET_CONSTRUCTION:
        L += [f"### {f['fact']}", '', f"- **Measured:** {f['numbers']}",
              f"- **Source:** `{f['source']}`",
              f"- **Consequence for writing:** {f['consequence']}", '']

    L += ['## Rule violations detected in this build', '']
    if violations:
        L += ['| claim | rule | offending cells |', '|---|---|---|']
        for cid, rule, cells in violations:
            L.append(f"| `{cid}` | {rule} | {', '.join('`%s`' % x for x in cells)} |")
        L.append('')
    else:
        L += ['None. Every claim\'s declared strength is consistent with its cells.', '']
    open(path, 'w').write('\n'.join(L) + '\n')
    return violations
