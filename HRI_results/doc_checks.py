#!/usr/bin/env python3
"""Published numbers from the documents of record, so every regenerated cell is checked against
the prose rather than trusted.  A mismatch is printed as a FINDING, not silently absorbed.

`expected` marks a difference we already understand (e.g. a row deliberately recomputed on
hardware-pinned seeds only); it is still reported, but as an explained difference.
"""

DOC_OF_RECORD = [
    # --- Genesis pick
    dict(id='pick_rnd30_dp', doc='RESULTS_WM_HUMAN_VS_MACHINE §2', human=0.520, machine=0.467,
         p=0.123),
    dict(id='pick_rnd30_rlpd', doc='CELL_STATUS_2026-09-07 (revised comparator)', human=0.600,
         machine=0.567, p=0.646),
    dict(id='pick_rnd30_rlpd_frozen', doc='RESULTS_WM_HUMAN_VS_MACHINE §2', human=0.600,
         machine=0.517, p=0.485),
    dict(id='pick_rnd30_r2d_mode', doc='RESULTS §2 / MORNING_TABLE §2', human=0.617,
         machine=0.608, p=0.875),
    dict(id='pick_rnd30_r2d_sample', doc='RESULTS §2 / MORNING_TABLE §2', human=0.613,
         machine=0.629, p=0.641),
    dict(id='pick_rnd30_dv3', doc='RESULTS §7 item 9', human=0.700, machine=0.633, p=0.333),

    # --- Genesis pick, in-distribution
    dict(id='pick_spots60_dp_asrecorded', doc='CELL_STATUS_2026-09-07', human=0.878,
         machine=0.873, p=0.845),
    dict(id='pick_spots60_dp', doc='CELL_STATUS_2026-09-07', human=0.878, machine=0.873,
         p=0.845,
         expected='CELL_STATUS quotes the ten-seed human arm; this row is the five '
                  'hardware-pinned seeds only, which is the point of the row.'),
    dict(id='pick_spots60_rlpd', doc='CELL_STATUS_2026-09-07', human=0.869, machine=0.865,
         p=0.873,
         expected='CELL_STATUS quotes the pre-pinning arm; this row is now the full eight '
                  'hardware-pinned seeds (the pinned re-runs have landed), so the two differ by '
                  'the size of the hardware term, 0.869 vs 0.867.'),

    # --- Genesis phases
    dict(id='place_r2d_asrecorded', doc='PHASE_RESULTS §2.y', human=0.703,
         machine=0.647, p=0.227),
    dict(id='contact_r2d_bare', doc='PHASE_RESULTS §3', human=0.593, machine=0.602, p=0.841),
    dict(id='contact_r2d_push', doc='PHASE_RESULTS §3 (2026-09-07 re-score)', human=0.346,
         machine=0.366, p=0.31, tol=0.006),
    dict(id='carry_r2d_bare', doc='PHASE_RESULTS §4', human=0.807, machine=0.796, p=0.348),
    dict(id='carry_r2d_push', doc='PHASE_RESULTS §3 addendum (re-score)', human=0.285,
         machine=0.250, p=0.55, tol=0.006),

    # --- Genesis end-to-end
    dict(id='e2e_picked_asrecorded', doc='PHASE_RESULTS §5.1', human=0.500,
         machine=0.537, p=0.643),
    dict(id='e2e_nested_proxy_asrecorded', doc='PHASE_RESULTS §5.1 (training proxy)',
         human=0.163,
         machine=0.192, p=0.621),

    # --- robomimic Can
    dict(id='robo_rlpd_mg200s', doc='ROBOMIMIC_LOG 13:30 readout', human=0.455, machine=0.147,
         p=0.008),
    dict(id='robo_rlpd_mg718s', doc='ROBOMIMIC_LOG A2 C2', human=0.455, machine=0.475, p=0.886),
    dict(id='robo_rlpd_mgall', doc='ROBOMIMIC_LOG A2 C1', human=0.455, machine=0.610, p=0.192),
    dict(id='robo_rlpd_mg200s_3x', doc='ROBOMIMIC_LOG A2 C3', human=0.455, machine=0.287,
         p=0.190),
    dict(id='robo_dp', doc='ROBOMIMIC_LOG DP readout', human=0.863, machine=0.095, p=0.00016,
         tol=0.002),
    dict(id='robo_bcrnn_mh', doc='ROBOMIMIC_LOG G1 control', human=0.927, machine=0.393),
    dict(id='robo_bcrnn_ph', doc='ROBOMIMIC_LOG G1 control', human=0.920, machine=0.393),
    dict(id='robo_rlpd_nodemo', doc='ROBOMIMIC_LOG G2b', human=0.455, machine=0.000),
]


# A doc entry that points at an `_asrecorded` row is, by construction, quoting a cell that has
# since been re-scored. check_docs() turns each of those into a "document needs updating" line
# naming the new value of record, so a stale document is surfaced rather than silently tolerated.
