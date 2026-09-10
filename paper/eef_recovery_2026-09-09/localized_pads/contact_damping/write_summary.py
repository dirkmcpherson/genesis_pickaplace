"""Write a final readout only after all corrected jobs have terminated."""
from pathlib import Path
import json
D=Path(__file__).resolve().parent
s=json.loads((D/'corrected/summary.json').read_text())
first=json.loads((D/'summary.json').read_text())
assert s['all_terminal'] and s['completed_full_replays']==10
assert first['all_terminal'] and first['completed_full_replays']==2
assert all(r['status']=='complete' for r in s['records'])
assert all(not r['has_full_trace'] for r in first['records'] if r['status']=='execution_failed')
identity=[r for report in [first,s] for r in report['records'] if r.get('exact_reference_arrays')]
assert len(identity)==4 and all(all(r['exact_reference_arrays'].values()) for r in identity)
treatments=[r for r in s['records'] if r['gain']==2]
maximum=max(max(r['damping_audit']['max_relative_error_static_normal_normal_velocity_tangent_velocity']) for r in treatments)
static=max(r['damping_audit']['max_relative_error_static_normal_normal_velocity_tangent_velocity'][0] for r in treatments)
lines=['# Completed damping comparison: candidate rejected','',
       'All ten corrected full replays are complete. Added damping yields no',
       'end-to-end improvement: soft and matching rigid each complete 0/3 original-',
       'placement calibration demos under both the supplied metric and strict',
       'sequence. Original fixed coupling completes 1/3. Both conditional113',
       'treatments also fail supported release. No pad, engine or pose is adopted.','',
       '| Trial and placement | Pads | Final center distance (m) | Final goal movement (mm) | Strict failure |',
       '| --- | --- | ---: | ---: | --- |']
assert all(v['strict_complete']==0 and v['metric_pass']==0 for v in s['original_placement_totals'].values())
for r in treatments:
 lines.append(f"| {r['uid']} {r['placement']} | {r['condition']} | {r['metric']['final_dist']:.4f} | {r['final_goal_shift_mm']:.2f} | {r['sequence']['reason']} |")
lines += ['',
 'The undamped soft233 identity still completes, exactly matching its archived',
 'trace (65.99mm center distance, 9.86mm goal movement). Both damped233 variants',
 'carry through18s, retain simultaneous shelf/hand contact around22s, then tip',
 'by24s. The original fixed233 reference completes with65.81mm center distance',
 'and1.00mm goal movement; its different hand and engine remain disclosed.','',
 'Conditional113 rigid previously settled upright but failed the later push.',
 'At doubled damping, both pad variants briefly encounter the shelf and fall',
 'off. `../conditional_release_comparison.png` compares all four saved trajectories',
 'and was visually inspected. The treatment acts throughout approach and carry,',
 'so it also changes the configuration from which opening starts. Preserving',
 'the elastic coefficient at fixed penetration does not preserve that history.','',
 f'Every treated contact checks the executed engine reference-acceleration function. Maximum static-term relative error is {static:.3g}; maximum error across static/normal-velocity/tangent-velocity checks is {maximum:.3g}. Time constants remain above the2.5ms safety floor. Geometry, friction coefficient, hand parameters, early yaw, full source commands and scoring are unchanged. Paired collision policies match.','',
 'All four identity executions (two preserved from the first implementation',
 'attempt and two in the corrected batch) reproduce every archived NPZ array.',
 'There are12 completed full replays in this investigation. Eight separate',
 'first-attempt treatment jobs failed compilation before stepping and are retained',
 'as implementation failures, not full demonstrations or recovery failures.','',
 'The tested intervention doubles both normal and tangential reference damping.',
 'Its rejection does not settle whether normal-only damping could help. The',
 'source inspection in `../normal_only_capability.json` identifies a possible',
 'isolated constraint-row test; it is not implemented or validated. Any such test',
 'must verify row mapping, exact identity controls and untouched tangent response',
 'before full-task interpretation. Neither a new friction coefficient nor a',
 'success-selected placement is justified by this negative result.','']
(D/'corrected/README.md').write_text('\n'.join(lines))
print('Final report written; 12 complete full replays, eight pre-step implementation failures separate.')
