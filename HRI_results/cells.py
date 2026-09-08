#!/usr/bin/env python3
"""The registry of comparisons: WHICH cell is of record, WHERE its per-seed counts come from,
and WHY it is (or is not) provisional.  make_tables.py walks this list; adding a result means
adding an entry here, not editing the table.

Every entry is a human-arm vs machine-arm contrast.  `sel_h` / `sel_m` are filters over the
columns of seed_counts_raw.csv (produced by harvest_cluster.py on the cluster).  A comparison
whose selectors match nothing renders as an EMPTY row with its `empty_reason` -- it is never
estimated, interpolated or back-filled from prose.
"""

# floor=True  -> the statistic sits on a floor for BOTH arms; a null there is an artefact of the
#                floor, not evidence of equivalence, so no p-value and no ROPE are reported.
# provisional -> the cell is expected to move; the string says why.

R = lambda lo, hi: [str(s) for s in range(lo, hi + 1)]

COMPARISONS = [
    # ---------------------------------------------------------------- Genesis: PICK, random starts
    dict(id='pick_rnd30_dp', expect_seeds=10, group='Pick (rnd30)', learner='Diffusion Policy', stat='picked',
         action='sampled', human='dH (pruned, 58)', machine='dDP (58)',
         sel_h=dict(source='cluster:dp_headline', arm='dH', cell='HEADLINE/final/rnd',
                    extra_has='wave=dp_w2final', seeds=R(20, 29)),
         sel_m=dict(source='cluster:dp_headline', arm='dDP', cell='HEADLINE/final/rnd',
                    extra_has='wave=dp_w2final', seeds=R(20, 29)),
         note='LAST checkpoint. DP has no mode cell; sampled is its statistic of record. '
              'Four of ten sweep dirs per arm were pruned by the disk fix, so the counts come '
              'from sweep/HEADLINE.txt; the six surviving sweep.json seeds agree exactly.'),

    dict(id='pick_rnd30_rlpd', expect_seeds=8, group='Pick (rnd30)', learner='RLPD', stat='picked',
         action='mode', human='dHv2raw (raw, 66)', machine='dDPv2 (raw-matched)',
         sel_h=dict(source='cluster:rlpd', arm='dHv2raw', cell='final/rnd',
                    extra_has='wave=rlpd_g99v2fullw3', seeds=R(60, 67)),
         sel_m=dict(source='cluster:rlpd', arm='dDPv2', cell='final/rnd',
                    extra_has='wave=rlpd_g99v2fullw3', seeds=R(50, 57)),
         note='LAST. "mode" here is a true deterministic action (tanh of the mean). One dead '
              'seed per arm (human s65 2/30, machine s55 0/30). This is the CELL_STATUS '
              'comparator; see pick_rnd30_rlpd_frozen for the RESULTS-doc comparator.'),

    dict(id='pick_rnd30_rlpd_frozen', expect_seeds=8, group='Pick (rnd30)', learner='RLPD', stat='picked',
         action='mode', human='dHv2raw (raw, 66)', machine='dDP (frozen, 58)',
         sel_h=dict(source='cluster:rlpd', arm='dHv2raw', cell='final/rnd',
                    extra_has='wave=rlpd_g99v2fullw3', seeds=R(60, 67)),
         sel_m=dict(source='cluster:rlpd', arm='dDP', cell='final/rnd',
                    extra_has='wave=rlpd_g99w3', seeds=R(40, 47)),
         note='SAME human arm, DIFFERENT machine arm: the frozen dDP set. This is the pairing '
              'printed in RESULTS_WM_HUMAN_VS_MACHINE §2 (0.600 v 0.517). Both pairings '
              'reproduce from the cluster; they are different comparators, not a discrepancy.'),

    dict(id='pick_rnd30_r2d_mode', expect_seeds=8, group='Pick (rnd30)', learner='world model (r2dreamer)',
         stat='picked', action='mode', human='dHv2raw (raw, 66)', machine='dDP (58)',
         sel_h=dict(source='cluster:wm', arm='dHv2raw', cell='rnd30', action_mode='mode',
                    statistic='picked', extra='setting=bnormclamp1ent5'),
         sel_m=dict(source='cluster:wm', arm='dDP', cell='rnd30', action_mode='mode',
                    statistic='picked', extra='setting=bnormclamp1ent5'),
         note='LAST, recipe bnormclamp1ent5. "mode" is the world model\'s mode-action cell -- '
              'NOT deterministic: r2dreamer samples a latent inside the policy call.'),

    dict(id='pick_rnd30_r2d_sample', expect_seeds=8, group='Pick (rnd30)', learner='world model (r2dreamer)',
         stat='picked', action='sampled', human='dHv2raw (raw, 66)', machine='dDP (58)',
         sel_h=dict(source='cluster:wm', arm='dHv2raw', cell='rnd30', action_mode='sample',
                    statistic='picked', extra='setting=bnormclamp1ent5'),
         sel_m=dict(source='cluster:wm', arm='dDP', cell='rnd30', action_mode='sample',
                    statistic='picked', extra='setting=bnormclamp1ent5'),
         note='The sampled row. This is the row dv3 must be compared against, not the mode row.'),

    dict(id='pick_rnd30_dv3', expect_seeds=4, group='Pick (rnd30)', learner='world model (dv3)', stat='picked',
         action='sampled', human='dHv2raw (raw, 66)', machine='dDP (58)',
         sel_h=dict(source='cluster:dv3', arm='dHv2raw', cell='rnd30_recov', statistic='picked'),
         sel_m=dict(source='cluster:dv3', arm='dDP', cell='rnd30_recov', statistic='picked'),
         provisional='n = 2 v 2 (G3 seeds 2/3 still running); the permutation test floor at this '
                     'n is p = 0.333, so it has no power. dv3 cells are SAMPLED (the sidecar '
                     '"deterministic" label is a known mislabel) and belong against the '
                     'r2dreamer SAMPLED row, 0.613 v 0.629.',
         note='Second world-model implementation; its value is that the return_clamp diagnosis '
              'reproduces on an independent port, not the number.'),

    # ---------------------------------------------------------------- Genesis: PICK, in-distribution
    dict(id='pick_spots60_dp', group='Pick (spots60, in-distribution)', learner='Diffusion Policy',
         stat='picked', action='sampled', human='dH (pruned, 58)', machine='dDP (58)',
         sel_h=dict(source='cluster:dp', arm='dH', cell='selected_spots60/spots60',
                    extra_has='wave=dp_w2final'),
         sel_m=dict(source='cluster:dp', arm='dDP', cell='selected_spots60/spots60',
                    extra_has='wave=dp_w2final'),
         provisional='Hardware-pinned seeds only. Five human seeds (25-29) were archived to '
                     'selected_spots60_mixedcore pending pinned re-runs, so this row reproduces '
                     'from FIVE human seeds against ten machine seeds. The published 0.878 is '
                     'the ten-seed figure including the archived five.',
         note='Selected checkpoint. In-training-distribution starts: the strongest null in the '
              'project because both arms sit near ceiling rather than near a floor.'),

    dict(id='pick_spots60_dp_asrecorded', expect_seeds=10, group='Pick (spots60, in-distribution)',
         learner='Diffusion Policy', stat='picked', action='sampled',
         human='dH (pruned, 58) incl. archived', machine='dDP (58)',
         sel_h=dict(source='cluster:dp', arm='dH',
                    cell_in=('selected_spots60/spots60', 'selected_spots60_mixedcore/spots60'),
                    extra_has='wave=dp_w2final'),
         sel_m=dict(source='cluster:dp', arm='dDP', cell='selected_spots60/spots60',
                    extra_has='wave=dp_w2final'),
         provisional='Mixes hardware classes WITHIN the human arm (5 pinned + 5 archived '
                     'mixed-core seeds) against an all-one-class machine arm. Reported only '
                     'because it is the figure the docs of record quote (0.878).',
         note='The as-published ten-seed cell, kept next to the pinned-only row so the size of '
              'the hardware term is visible rather than argued about.'),

    dict(id='pick_spots60_rlpd', expect_seeds=8, group='Pick (spots60, in-distribution)', learner='RLPD',
         stat='picked', action='mode', human='dHv2raw (raw, 66)', machine='dDPv2 (raw-matched)',
         sel_h=dict(source='cluster:rlpd', arm='dHv2raw', cell='final_det_spots60/spots60',
                    extra_has='wave=rlpd_g99v2fullw3'),
         sel_m=dict(source='cluster:rlpd', arm='dDPv2', cell='final_det_spots60/spots60',
                    extra_has='wave=rlpd_g99v2fullw3'),
         provisional='Hardware-pinned seeds only: two human seeds (60, 61) are archived to '
                     'final_det_spots60_mixedcore, so this is 6 v 8. The published 0.869 is the '
                     'eight-seed figure including the archived two.',
         note='LAST, deterministic.'),

    dict(id='pick_spots60_r2d', group='Pick (spots60, in-distribution)',
         learner='world model (r2dreamer)', stat='picked', action='mode',
         human='dHv2raw (raw, 66)', machine='dDP (58)',
         sel_h=dict(source='cluster:wm', arm='dHv2raw', cell='spots60', statistic='picked'),
         sel_m=dict(source='cluster:wm', arm='dDP', cell='spots60', statistic='picked'),
         empty_reason='Not run. The world model has no spots60 evaluation; the in-distribution '
                      'row exists for DP and RLPD only.'),

    # ------------------------------------------- PRUNING CONTROL (human raw vs human pruned)
    # Not a source contrast. This is the reference effect size: how much a DATA-HANDLING choice
    # inside the human arm moves Diffusion Policy, for comparison against every source contrast
    # above. Both arms here are human-sourced.
    dict(id='prune_dp_rnd30', group='Pruning control (human pruned vs human raw)',
         learner='Diffusion Policy', stat='picked', action='sampled',
         human='dH PRUNED (58)', machine='dHv2raw RAW (66)',
         sel_h=dict(source='cluster:dp', arm='dH', cell='selected/rnd',
                    extra_has='wave=dp_w2final'),
         sel_m=dict(source='cluster:dp', arm='dHv2raw', cell='selected/rnd',
                    extra_has='wave=dp_v2fullw3'),
         provisional='The two arms differ in wave AND in demonstration count (58 pruned vs 66 '
                     'raw), so this is a pruned-set-vs-raw-set contrast, not a controlled '
                     'pruning-only manipulation. The direction and magnitude are the point.',
         note='Random starts, selected checkpoint. "Delta" here is pruned minus raw.'),

    dict(id='prune_dp_spots60', group='Pruning control (human pruned vs human raw)',
         learner='Diffusion Policy', stat='picked', action='sampled',
         human='dH PRUNED (58)', machine='dHv2raw RAW (66)',
         sel_h=dict(source='cluster:dp', arm='dH',
                    cell_in=('selected_spots60/spots60',
                             'selected_spots60_mixedcore/spots60'),
                    extra_has='wave=dp_w2final'),
         sel_m=dict(source='cluster:dp', arm='dHv2raw', cell='selected_spots60/spots60',
                    extra_has='wave=dp_v2fullw3'),
         provisional='Same wave/count caveat as prune_dp_rnd30. The human pruned arm here is the '
                     'ten-seed as-published set (five pinned + five archived mixed-core), '
                     'matching the in-distribution table this figure was quoted from.',
         note='IN-TRAINING-DISTRIBUTION starts. This is the cell behind the quoted "raw costs '
              '0.19": pruned 0.878 against raw 0.688.'),

    # ---------------------------------------------------------------- Genesis: PLACE
    dict(id='place_r2d', expect_seeds=8, group='Place (matched 39)', learner='world model (r2dreamer)',
         stat='placed_v2', action='mode', human='dH_place (39)', machine='dDP_place_n39 (39)',
         sel_h=dict(source='cluster:wm', phase='place', arm='dH', cell='polE_v2',
                    action_mode='mode', statistic='placed_v2', extra='setting=bnormclamp1ent5'),
         sel_m=dict(source='cluster:wm', phase='place', arm='dDP', cell='polE_v2',
                    action_mode='mode', statistic='placed_v2',
                    extra='setting=bnormclamp1ent5_n39'),
         rescored_from='place_r2d_asrecorded',
         note='NUMBER OF RECORD. The 2026-09-08 re-score on the rebuilt entry bank '
              '(physgrip_2026-09-07), with entry pinning and hardware pinned. Prefer this over '
              'the as-recorded row below, which is kept so the size of the correction is '
              'visible.'),

    dict(id='place_r2d_asrecorded', expect_seeds=8, group='Place (matched 39)',
         learner='world model (r2dreamer)', stat='placed_v2', action='mode',
         human='dH_place (39)', machine='dDP_place_n39 (39)',
         sel_h=dict(source='cluster:wm', phase='place', arm='dH', cell='polE',
                    action_mode='mode', statistic='placed_v2', extra='setting=bnormclamp1ent5'),
         sel_m=dict(source='cluster:wm', phase='place', arm='dDP', cell='polE',
                    action_mode='mode', statistic='placed_v2',
                    extra='setting=bnormclamp1ent5_n39'),
         provisional='SUPERSEDED by place_r2d. Drew bank entries WITH REPLACEMENT and '
                     'substituted failed restores; unpinned hardware. Kept visible only so the '
                     'correction can be inspected.',
         note='The 0.703 v 0.647 published in PHASE_RESULTS 2.y.'),

    dict(id='place_r2d_sample', expect_seeds=8, group='Place (matched 39)',
         learner='world model (r2dreamer)', stat='placed_v2', action='sampled',
         human='dH_place (39)', machine='dDP_place_n39 (39)',
         sel_h=dict(source='cluster:wm', phase='place', arm='dH', cell='polE_v2',
                    action_mode='sample', statistic='placed_v2',
                    extra='setting=bnormclamp1ent5'),
         sel_m=dict(source='cluster:wm', phase='place', arm='dDP', cell='polE_v2',
                    action_mode='sample', statistic='placed_v2',
                    extra='setting=bnormclamp1ent5_n39'),
         rescored_from='place_r2d_sample_asrecorded',
         note='The sampled cell of the same re-score. This is where the correction moves the '
              'DIFFERENCE most: see the re-score movement table.'),

    dict(id='place_r2d_sample_asrecorded', expect_seeds=8, group='Place (matched 39)',
         learner='world model (r2dreamer)', stat='placed_v2', action='sampled',
         human='dH_place (39)', machine='dDP_place_n39 (39)',
         sel_h=dict(source='cluster:wm', phase='place', arm='dH', cell='polE',
                    action_mode='sample', statistic='placed_v2',
                    extra='setting=bnormclamp1ent5'),
         sel_m=dict(source='cluster:wm', phase='place', arm='dDP', cell='polE',
                    action_mode='sample', statistic='placed_v2',
                    extra='setting=bnormclamp1ent5_n39'),
         provisional='SUPERSEDED by place_r2d_sample; same defects as place_r2d_asrecorded.',
         note=''),

    dict(id='place_dp', expect_seeds=8, group='Place (matched 39)', learner='Diffusion Policy', stat='placed_v2',
         action='sampled', human='dH_place (39)', machine='dDP_place_n39 (39)',
         sel_h=dict(source='cluster:phase_clone', phase='place', learner='DiffusionPolicy',
                    run_has='_dH_', cell='fresh_eval_polE_sample', statistic='placed_v2'),
         sel_m=dict(source='cluster:phase_clone', phase='place', learner='DiffusionPolicy',
                    run_has='_dDP_', cell='fresh_eval_polE_sample', statistic='placed_v2'),
         empty_reason='DP place runs have not landed. The 32-run place batch is still filling; '
                      'only RLPD cells exist in $LAB/gp_place so far.'),

    dict(id='place_rlpd', expect_seeds=8, group='Place (matched 39)', learner='RLPD', stat='placed_v2',
         action='mode', human='dH_place (39)', machine='dDP_place_n39 (39)',
         sel_h=dict(source='cluster:phase_clone', phase='place', learner='RLPD',
                    run_has='_dH_', cell='fresh_eval_polE_mode', statistic='placed_v2'),
         sel_m=dict(source='cluster:phase_clone', phase='place', learner='RLPD',
                    run_has='_dDP_', cell='fresh_eval_polE_mode', statistic='placed_v2'),
         provisional='PARTIAL: the 32-run place batch is still filling, so this is far short of '
                     'the registered 8 v 8. Scored on the REBUILT physgrip_2026-09-07 entry '
                     'bank, which is NOT the bank the world-model place cells used.',
         note='Reported alone. See the row header for why it may not be read beside the '
              'world-model place cell.'),

    # ---------------------------------------------------------------- Genesis: CONTACT
    dict(id='contact_r2d_bare', expect_seeds=8, group='Contact (matched 11)', learner='world model (r2dreamer)',
         stat='contact (legacy)', action='mode', human='dH sub-floor (11)',
         machine='dDP_n11 (11)',
         sel_h=dict(source='cluster:wm', phase='contact', arm='dH', cell='polE',
                    action_mode='mode', statistic='contact',
                    extra='setting=bnormclamp1ent5_subfloor'),
         sel_m=dict(source='cluster:wm', phase='contact', arm='dDP', cell='polE',
                    action_mode='mode', statistic='contact',
                    extra='setting=bnormclamp1ent5_n11'),
         provisional='Bare `contact` is the LEGACY predicate and overstates capability by '
                     '1.5-3x against contact_push; carried only so the published number is '
                     'reproducible. Both arms are sub-floor (11 demonstrations each).',
         rescore_in_flight='Computed on the OLD raw-grip, unpinned cells. The pinned re-score '
                           'is PARTIALLY LANDED (2 human / 6 machine seeds of 8 at this build), '
                           'so these inputs are being overwritten, not merely refined.',
         note='Read contact_r2d_push instead.'),

    dict(id='contact_r2d_push', expect_seeds=8, group='Contact (matched 11)', learner='world model (r2dreamer)',
         stat='contact_push', action='mode', human='dH sub-floor (11)', machine='dDP_n11 (11)',
         sel_h=dict(source='cluster:wm', phase='contact', arm='dH', cell='polE_cp',
                    action_mode='mode', statistic='contact_push',
                    extra='setting=bnormclamp1ent5_subfloor'),
         sel_m=dict(source='cluster:wm', phase='contact', arm='dDP', cell='polE_cp',
                    action_mode='mode', statistic='contact_push',
                    extra='setting=bnormclamp1ent5_n11'),
         rescore_in_flight='Computed on the OLD raw-grip, unpinned cells. The pinned re-score '
                           'is PARTIALLY LANDED (2 human / 6 machine seeds of 8 at this build), '
                           'so these inputs are being overwritten, not merely refined.',
         note='contact_push is the DISCRIMINATING statistic of the three contact predicates: it '
              'requires the tool point to push the can, where bare contact does not.'),

    dict(id='contact_dp', group='Contact (matched 11)', learner='Diffusion Policy',
         stat='contact_push', action='sampled', human='dH sub-floor (11)', machine='dDP_n11 (11)',
         sel_h=dict(source='cluster:phase_clone', phase='contact', learner='DiffusionPolicy'),
         sel_m=dict(source='cluster:phase_clone', phase='contact', learner='DiffusionPolicy'),
         empty_reason='Not run. The contact phase was never submitted for DP or RLPD; the 32 '
                      'slide/contact runs are HELD on the predicate decision (PHASE_PLAN (p)).'),

    dict(id='contact_rlpd', group='Contact (matched 11)', learner='RLPD', stat='contact_push',
         action='mode', human='dH sub-floor (11)', machine='dDP_n11 (11)',
         sel_h=dict(source='cluster:phase_clone', phase='contact', learner='RLPD'),
         sel_m=dict(source='cluster:phase_clone', phase='contact', learner='RLPD'),
         empty_reason='Not run (same hold as contact_dp).'),

    # ---------------------------------------------------------------- Genesis: CARRYCONTACT
    dict(id='carry_r2d_bare', expect_seeds=8, group='Carrycontact (matched 21)',
         learner='world model (r2dreamer)', stat='contact (legacy)', action='mode',
         human='dH (21)', machine='dDP_n21 (21)',
         sel_h=dict(source='cluster:wm', phase='carrycontact', arm='dH', cell='polE',
                    action_mode='mode', statistic='contact', extra='setting=bnormclamp1ent5'),
         sel_m=dict(source='cluster:wm', phase='carrycontact', arm='dDP', cell='polE',
                    action_mode='mode', statistic='contact',
                    extra='setting=bnormclamp1ent5_n21'),
         provisional='Legacy bare-contact predicate; see carry_r2d_push. Five polE entries fail '
                     'to restore in this scope for every arm and are counted as failures.',
         rescore_in_flight='Carrycontact is last in the re-score queue and has barely started '
                           '(2 human / 1 machine seed of 8 at this build).',
         note=''),

    dict(id='carry_r2d_push', expect_seeds=8, group='Carrycontact (matched 21)',
         learner='world model (r2dreamer)', stat='contact_push', action='mode',
         human='dH (21)', machine='dDP_n21 (21)',
         sel_h=dict(source='cluster:wm', phase='carrycontact', arm='dH', cell='polE_cp',
                    action_mode='mode', statistic='contact_push',
                    extra='setting=bnormclamp1ent5'),
         sel_m=dict(source='cluster:wm', phase='carrycontact', arm='dDP', cell='polE_cp',
                    action_mode='mode', statistic='contact_push',
                    extra='setting=bnormclamp1ent5_n21'),
         rescore_in_flight='Carrycontact is last in the re-score queue and has barely started '
                           '(2 human / 1 machine seed of 8 at this build).',
         note='The discriminating contact predicate on the carrycontact scope.'),

    # ---------------------------------------------------------------- Genesis: END-TO-END by stage
] + [
    # The pinned end-to-end re-score COMPLETED at 8 v 8 on every stage (2026-09-08). The v2 cell
    # is now the row of record; the as-recorded row is kept beside it so the correction stays
    # visible -- the same pattern as place and spots60.
    dict(id=f'e2e_{sid}{suffix}', expect_seeds=8, group='End-to-end (rnd30)',
         learner='world model (r2dreamer)',
         stat=sname, action='mode', human='dHfull_all (74)', machine='dDPfull (best-per-IC, 72)',
         sel_h=dict(source='cluster:wm', phase='e2e', arm='dHfull', cell=scell,
                    action_mode='mode', statistic=skey,
                    extra='setting=all_bnormclampS8ent5'),
         sel_m=dict(source='cluster:wm', phase='e2e', arm='dDPfull', cell=scell,
                    action_mode='mode', statistic=skey, extra='setting=bnormclampS8ent5'),
         floor=floor, provisional=prov, note=nt,
         **({'rescored_from': f'e2e_{sid}_asrecorded'} if not suffix else {}))
    for sid, skey, sname, floor, base_prov, nt in [
        ('picked', 'picked', 'picked', False, '', 'Stage 1 of the full task.'),
        ('contact_push', 'contact_push', 'contact_push', False, '',
         'The discriminating contact predicate on the full task.'),
        ('nested_honest', 'nested_honest', 'nested (honest)', False, '',
         'The HONEST settled predicate. The published end-to-end nested figure is the TRAINING '
         'PROXY, which over-counts roughly 2.5x; they are different quantities and must not '
         'share a row.'),
        ('nested_proxy', 'nested', 'nested (training proxy)', False,
         'Training proxy, not a task outcome. Shown only to quantify the gap against '
         'nested_honest; never cite it as a success rate.', ''),
        ('slide_success', 'slide_success', 'slide_success', True, '',
         'TASK OUTCOME. No arm learns a true slide: both sit at 0-6 %. A null here is an '
         'artefact of the floor, so no p-value and no ROPE are computed.'),
    ]
    for suffix, scell, prov in (
        ('', 'rnd30_v2', base_prov),
        ('_asrecorded', ('rnd30' if sid in ('picked', 'nested_proxy') else 'rnd30_cp'),
         (base_prov + ' SUPERSEDED by the pinned re-score, which has landed at 8 v 8; kept only '
                      'so the correction is visible.').strip()),
    )
] + [
    dict(id='e2e_dp', group='End-to-end (rnd30)', learner='Diffusion Policy', stat='picked',
         action='sampled', human='dHfull_all (74)', machine='dDPfull (72)',
         sel_h=dict(source='cluster:phase_clone', phase='e2e', learner='DiffusionPolicy'),
         sel_m=dict(source='cluster:phase_clone', phase='e2e', learner='DiffusionPolicy'),
         empty_reason='Runs queued, not landed. 32 e2e runs (e2e_dp_* / e2e_rlpd_*) sit PENDING '
                      'at --nice=9000 behind the rest of the queue.'),

    dict(id='e2e_rlpd', group='End-to-end (rnd30)', learner='RLPD', stat='picked', action='mode',
         human='dHfull_all (74)', machine='dDPfull (72)',
         sel_h=dict(source='cluster:phase_clone', phase='e2e', learner='RLPD', arm='__none__'),
         sel_m=dict(source='cluster:phase_clone', phase='e2e', learner='RLPD', arm='__none__'),
         empty_reason='Runs queued, not landed (same batch as e2e_dp). The only e2e metrics on '
                      'the cluster clone are smoke runs, which are excluded.'),

    # ---------------------------------------------------------------- robomimic Can
    dict(id='robo_rlpd_mg200s', expect_seeds=8, group='robomimic Can', learner='RLPD', stat='success',
         action='mode', human='MH200 (200 human tapes)', machine='MG200s (200-tape subsample)',
         sel_h=dict(source='cluster:robomimic', learner='RLPD', arm='MH200',
                    cell='eval_bank50_mode'),
         sel_m=dict(source='cluster:robomimic', learner='RLPD', arm='MG200s',
                    cell='eval_bank50_mode'),
         provisional='THE SOURCE READING IS WITHDRAWN by its own registered quantity control. '
                     'The gap is a property of THIS 200-tape draw, which is not neutral: 89 % '
                     'of its rows come from the last four SAC checkpoint blocks, so it is both '
                     'small and narrow. Whether a NEUTRAL 200-tape draw would match the human '
                     'arm is UNTESTED, and it is the difference between a pathological sample '
                     'and a real per-demonstration quality gap. See MG718s / MGall.',
         note='LAST, 50 shared bank starts, 8 seeds.'),

    dict(id='robo_rlpd_mg718s', expect_seeds=8, group='robomimic Can', learner='RLPD', stat='success',
         action='mode', human='MH200 (200 human tapes)', machine='MG718s (all 718 successes)',
         sel_h=dict(source='cluster:robomimic', learner='RLPD', arm='MH200',
                    cell='eval_bank50_mode'),
         sel_m=dict(source='cluster:robomimic', learner='RLPD', arm='MG718s_ctl',
                    cell='eval_bank50_mode'),
         note='THE CONTROL THAT FIRED. At its NATURAL FULL SIZE (718 tapes, ~3.6x the human '
              'arm) the machine arm is indistinguishable from the human arm. This does not show '
              'that machine data matches human data per demonstration - only that the source '
              'penalty seen at 200 tapes disappears once the machine arm is allowed its own '
              'size.'),

    dict(id='robo_rlpd_mgall', expect_seeds=8, group='robomimic Can', learner='RLPD', stat='success',
         action='mode', human='MH200 (200 human tapes)', machine='MGall (3900 rollouts)',
         sel_h=dict(source='cluster:robomimic', learner='RLPD', arm='MH200',
                    cell='eval_bank50_mode'),
         sel_m=dict(source='cluster:robomimic', learner='RLPD', arm='MGall_ctl',
                    cell='eval_bank50_mode'),
         note='At FULL SIZE (3900 rollouts, ~19.5x the human arm) the machine arm sits above '
              'the human arm. Read as a statement about quantity and coverage, not about '
              'per-demonstration quality: the comparison is not size-matched in this row.'),

    dict(id='robo_rlpd_mg200s_3x', expect_seeds=8, group='robomimic Can', learner='RLPD', stat='success',
         action='mode', human='MH200 (200 human tapes)', machine='MG200s @ 3x budget (300k)',
         sel_h=dict(source='cluster:robomimic', learner='RLPD', arm='MH200',
                    cell='eval_bank50_mode'),
         sel_m=dict(source='cluster:robomimic', learner='RLPD', arm='MG200s_ext300k',
                    cell='eval_bank50_mode'),
         note='Budget control: 3x training decisions does not close the 200-tape gap.'),

    dict(id='robo_dp', expect_seeds=8, group='robomimic Can', learner='Diffusion Policy', stat='success',
         action='sampled', human='MH200 (200 human tapes)', machine='MG200s (200-tape subsample)',
         sel_h=dict(source='cluster:robomimic', learner='DiffusionPolicy', arm='MH200',
                    cell='eval_bank50_sample'),
         sel_m=dict(source='cluster:robomimic', learner='DiffusionPolicy', arm='MG200s',
                    cell='eval_bank50_sample'),
         provisional='Rests on the SAME 200-tape draw whose source reading RLPD withdrew, and '
                     'inherits the same doubt. It has not been run on MG718s or MGall.',
         note='LAST. There is no PH200 DP arm, so the registered A1 prediction is untestable '
              'as written.'),

    dict(id='robo_bcrnn_mh', expect_seeds=3, group='robomimic Can', learner='BC-RNN', stat='success',
         action='mode', human='MH200 (200 human tapes)', machine='MG200s (200-tape subsample)',
         sel_h=dict(source='cluster:robomimic', learner='BC-RNN', arm='MH200',
                    cell='eval_bank50'),
         sel_m=dict(source='cluster:robomimic', learner='BC-RNN', arm='MG200s',
                    cell='eval_bank50'),
         withdrawn='WITHDRAWN by its own registered fixed-head control (amendment A5). This row compared each arm AT ITS OWN RECIPE OPTIMUM - the human arm with a 5-component mixture head, the machine arm with a deterministic one - so it measured the head choice and the demonstration source together, and cannot be attributed to source. Head-matched cells at adequate seeds do not exist yet, so no re-score can repair it.',
         note='LAST = epoch 2000. Kept visible only so the withdrawn comparison can be '
              'inspected against the head-matched rows below.'),

    dict(id='robo_bcrnn_ph', expect_seeds=3, group='robomimic Can', learner='BC-RNN', stat='success',
         action='mode', human='PH200 (200 proficient-human tapes)',
         machine='MG200s (200-tape subsample)',
         sel_h=dict(source='cluster:robomimic', learner='BC-RNN', arm='PH200',
                    cell='eval_bank50'),
         sel_m=dict(source='cluster:robomimic', learner='BC-RNN', arm='MG200s',
                    cell='eval_bank50'),
         withdrawn='WITHDRAWN by its own registered fixed-head control (amendment A5). This row compared each arm AT ITS OWN RECIPE OPTIMUM - the human arm with a 5-component mixture head, the machine arm with a deterministic one - so it measured the head choice and the demonstration source together, and cannot be attributed to source. Head-matched cells at adequate seeds do not exist yet, so no re-score can repair it.',
         note='Same withdrawal as robo_bcrnn_mh: PH200 also runs a mixture head against a '
              'deterministic-head machine arm.'),

    # --- the registered fixed-head control (A5): the 2x2 that withdraws the row above.
    # human mixture 0.927 / human deterministic 0.560 / machine mixture 0.093 /
    # machine deterministic 0.393  ->  head effect +0.367 human, -0.300 machine,
    # interaction +0.667 with OPPOSITE SIGNS.
    dict(id='robo_bcrnn_matched_mixture', expect_seeds=3, group='robomimic Can',
         learner='BC-RNN', stat='success', action='mode',
         human='MH200, 5-component mixture head', machine='MG200s, 5-component mixture head',
         sel_h=dict(source='cluster:robomimic', learner='BC-RNN', arm='MH200',
                    cell='eval_bank50'),
         sel_m=dict(source='cluster:robomimic', learner='BC-RNN', arm='MG200s_gmm',
                    cell='eval_bank50'),
         effect_size_only=True,
         note='HEAD-MATCHED source gap with MIXTURE heads on both arms. Compare against '
              'robo_bcrnn_matched_deterministic: the same source contrast measured with '
              'deterministic heads is FIVE TIMES smaller. The gap is not a property of the '
              'demonstrations alone.'),

    dict(id='robo_bcrnn_matched_deterministic', expect_seeds=3, group='robomimic Can',
         learner='BC-RNN', stat='success', action='mode',
         human='MH200, deterministic head', machine='MG200s, deterministic head',
         sel_h=dict(source='cluster:robomimic', learner='BC-RNN', arm='MH200_nogmm',
                    cell='eval_bank50'),
         sel_m=dict(source='cluster:robomimic', learner='BC-RNN', arm='MG200s',
                    cell='eval_bank50'),
         effect_size_only=True,
         note='HEAD-MATCHED source gap with DETERMINISTIC heads on both arms. 0.167 against '
              '0.834 for the mixture-head pairing - a fivefold difference produced by an '
              'architecture choice, not by the demonstrations.'),

    dict(id='robo_r2d', expect_seeds=8, group='robomimic Can', learner='world model (r2dreamer)', stat='success',
         action='mode', human='MH200 (200 human tapes)', machine='MG200s (200-tape subsample)',
         sel_h=dict(source='cluster:robomimic', learner='r2dreamer', arm='MH200',
                    cell='eval_bank50_mode'),
         sel_m=dict(source='cluster:robomimic', learner='r2dreamer', arm='MG200s',
                    cell='eval_bank50_mode'),
         floor=True,
         provisional='NEVER READ OUT BEFORE THIS TABLE. Both arms are on the floor at ~541k '
                     'training steps, so the world model has no working configuration on this '
                     'independent task and the cell cannot bear a source comparison.',
         note='Reported as a LEARNABILITY FAILURE, not as a null, and deliberately not as an '
              'empty cell: "we ran it and both arms floored" is a stronger and more informative '
              'statement than "not run". Consequence for the paper: the robomimic leg carries '
              'RLPD, Diffusion Policy and BC-RNN, but NOT a world model, so it cannot support '
              'the source-indifference headline on an independent task - on this task the world '
              'model does not learn at all.'),

    dict(id='robo_rlpd_nodemo', expect_seeds=8, group='robomimic Can', learner='RLPD', stat='success',
         action='mode', human='MH200 (200 human tapes)', machine='no demonstrations (control)',
         sel_h=dict(source='cluster:robomimic', learner='RLPD', arm='MH200',
                    cell='eval_bank50_mode'),
         sel_m=dict(source='cluster:robomimic', learner='RLPD', arm='none',
                    cell='eval_bank50_mode'),
         note='Registered negative control G2b: RLPD with no demonstrations at all. 0/50 on '
              'every seed, so demonstrations of either source are doing real work here.'),
]


# Rendered under the group heading in results.md.
GROUP_NOTES = {
    'robomimic Can':
        '**No learner in this leg currently supports a demonstration-source claim, and the '
        'cross-learner ordering is WITHDRAWN.** It has two independent reasons to be gone, not '
        'one, and both are registered controls that fired:\n\n'
        '1. **RLPD** - the quantity control. At their NATURAL FULL SIZE machine demonstrations '
        'match (MG718s, 718 tapes) or exceed (MGall, 3900) the human arm; only at MATCHED TAPE '
        'COUNT does this particular 200-tape draw lose (0.147 v 0.455). Stating that machine '
        'demonstrations "match or beat" human ones WITHOUT the size qualifier overstates the '
        'result in the opposite direction to the original error, which is no improvement.\n'
        '2. **BC-RNN** - the fixed-head control (A5). The published row compared each arm at its '
        'own recipe optimum, so it measured architecture and source together. See below.\n'
        '3. **The world model** never learned this task at all (0/400 and 1/400 at ~541k '
        'steps), so it cannot enter an ordering either.\n\n'
        'That leaves Diffusion Policy as the only learner whose robomimic row is neither '
        'withdrawn nor self-undermined - and it rests on the SAME 200-tape draw RLPD\'s control '
        'discredited. **The ordering must not be quoted in any form** until BC-RNN is re-run '
        'head-matched at adequate seeds.\n\n'
        '**The BC-RNN head interaction, 3 seeds per cell, final checkpoint, head forced and '
        'confirmed in every config:**\n\n'
        '| arm | mixture head | deterministic head | head effect |\n'
        '|---|---|---|---|\n'
        '| human (MH200) | 0.927 | 0.560 | **+0.367** |\n'
        '| machine (MG200s) | 0.093 | 0.393 | **-0.300** |\n\n'
        'The interaction is **+0.667 with OPPOSITE SIGNS**: the mixture head helps the human arm '
        'and hurts the machine arm. The head-matched source gap is **0.834 with mixture heads '
        'and 0.167 with deterministic ones - a fivefold difference from an architecture choice '
        'alone**. The published 0.534 was neither of those; it was a third quantity, each arm at '
        'its own optimum.\n\n'
        '**Mechanism: open, and one obvious explanation is REFUTED.** The natural reading - that '
        'human data is multi-modal because several operators use several strategies, so a '
        'mixture head suits it - was measured and does not hold: ONE operator scores '
        'indistinguishably from six (+0.126 against +0.127), and the machine sets are not '
        'unimodal either. Human sets do carry about 1.6x more non-Gaussian structure, but the '
        'larger unpredicted difference is that machine action spread is **3x** human. The '
        '"mixture head suits multi-operator data" story must not be used.\n\n'
        '**Still unresolved for the quantity control:** whether a NEUTRAL 200-tape draw would '
        'match the human arm. The MG200s subsample is not neutral - 89 % of its rows come from '
        'the last four SAC checkpoint blocks. A random 200-tape draw from the 718 successes '
        'separates a pathological sample from a real per-demonstration quality gap, for about '
        '8 GPU-hours, and has not been run.',

    'Pruning control (human pruned vs human raw)':
        'Both arms are HUMAN. This is the reference effect size for the table above: a '
        'data-handling choice inside one source moves Diffusion Policy far more than any '
        'source difference measured anywhere in this project.',
}
