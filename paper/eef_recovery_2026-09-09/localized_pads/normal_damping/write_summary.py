"""Write the normal-damping report from verified results, including pending jobs."""
from pathlib import Path
import json
D=Path(__file__).resolve().parent;s=json.loads((D/'summary.json').read_text())
finished=[r for r in s['records'] if r['status']=='complete']
identity=[r for r in finished if r['gain']==1];treatment=[r for r in finished if r['gain']==2]
max_error=max(max(r['damping_audit']['max_normal_mapping_tangent_mapping_normal_target_relative_errors']) for r in finished)
assert all(all(r['exact_reference_arrays'].values()) for r in identity)
lines=['# Normal-only pad damping: near-contact slide, no adoption','',
 f"Completed full replays: {len(finished)}/10. All terminal: {s['all_terminal']}. The verified implementation changes only the normal contact reference damping at existing inner-pad regions. The normal elastic term, tangent and all other references, contact parameters, friction coefficient and geometry are unchanged by the hook. Source motions, corrected early yaw and world geometry remain fixed.",'',
 '| Trial / placement | Pads | Gain | Status | Strict result | Final center distance (m) | Goal movement (mm) |',
 '| --- | --- | ---: | --- | --- | ---: | ---: |']
for r in s['records']:
 if r['status']=='complete':
  lines.append(f"| {r['uid']} {r['placement']} | {r['condition']} | {r['gain']:g} | complete | {r['sequence']['reason']} | {r['metric']['final_dist']:.5f} | {r['final_goal_shift_mm']:.3f} |")
 else:lines.append(f"| {r['uid']} {r['placement']} | {r['condition']} | {r['gain']:g} | {r['status']} | — | — | — |")
lines+=['',
 'Soft233 preserves the supplied proximity-metric pass but fails the strict',
 'push-to-contact sequence. After supported release at22.65s, the supported hand',
 'push starts22.86s and moves40.90mm toward the goal by the endpoint. Support',
 'fraction is.935, maximum height error1.37mm and maximum tilt2.54degrees during',
 'that interval. The final geometric surface gap is0.468mm; saved decision states',
 'contain no goal-contact frames. This does not prove absence of every substep',
 'contact. Goal displacement is0.636mm, versus9.863mm for the undamped soft',
 'reference. Reduced simulated goal movement alone is not a calibrated real-world',
 'fidelity measurement. `final_slide_diagnostic.py` retains descriptive measurements',
 'without changing either success criterion.','',
 'The project methods explicitly record user-validated233 as ending in contact.',
 'The strict miss remains a miss. `hardware_and_task_check.md` preserves that',
 'requirement and the manufacturer-document check; no numerical material damping',
 'law was found in the checked hardware sections.','',
 f"Every substep verifies assembled normal/tangent reference rows against the engine function and current Jacobian velocity. Maximum normalized row/target discrepancy across completed runs is {max_error:.3g}; all nonmutation violation counts are zero. {len(identity)} completed identity controls match every archived NPZ array. These forward-only checks do not establish backward/autodiff correctness or calibrated physical material behavior.",'',
 'The matched elliptic soft gain1 / normal gain2 cap-relative rim errors at',
 '14/16/22/25s are17.76/18.64/40.42/27.91 versus18.26/19.14/31.36/26.87px.',
 'Carry alignment is nearly unchanged; the22s release image is closer but remains',
 'substantially mismatched. The gain2 rigid errors are16.54/17.52/33.12/60.46px.',
 'Camera hypotheses and annotations are unchanged and no object pose is fitted',
 'to improve the result. Both new rigid and soft overlays were inspected.','',
 '`233_normal_damping_real_sim.mp4` covers the complete28.86s trial beside two',
 'real cameras, with proximity pass / strict contact test fails in the title.',
 'All161 frames decode and trace/video hashes are verified; timing differences',
 'are at most16.24/20.87ms. Carry, release, slide and endpoint stills were inspected.',
 'It renders saved poses without rerunning physics. The initial unpublished title',
 'was corrected to avoid claiming absence of all substep contacts. The first',
 'title-correction attempt found no ffmpeg drawtext filter; the completed OpenCV',
 'annotation and H264 verification scripts are preserved.','',
 'The broader day-balanced elliptic comparison is prepared in `../elliptic_panel/`.',
 'It separates softness and normal damping across the existing9-demo panel with',
 'four conditions per recording. This expands evidence gathering rather than',
 'relaxing the end-to-end requirement or declaring the calibration near miss a',
 'success. Prior broad validation covered pyramidal contact, not this elliptic',
 'configuration. No new damping level or outcome-selected initial pose is used.','']
(D/'README.md').write_text('\n'.join(lines))
print('Report updated:',len(finished),'complete; terminal',s['all_terminal'])
