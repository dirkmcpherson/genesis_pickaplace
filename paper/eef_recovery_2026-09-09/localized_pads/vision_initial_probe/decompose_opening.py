"""Resolve recorded finger impulses into normal and tangential components."""
from pathlib import Path
import json
import numpy as np

OUT = Path(__file__).resolve().parent / 'release_wrenches'
records = []
for condition in ['rigid', 'soft']:
    data = np.load(OUT / condition / 'release_forces.npz')
    contacts = data['contacts']
    audit = json.loads((OUT / condition / 'release_forces_audit.json').read_text())
    dt = audit['substep_dt_s']
    assert audit['geom_labels']['14'] == 'bottle'
    rows = []
    for lo, hi in [(20.1, 20.16), (20.13, 20.16), (20.15, 20.17)]:
        c = contacts[(contacts[:,0] >= lo) & (contacts[:,0] < hi)
                     & np.isin(contacts[:,3], [11,13])]
        sign = np.where(c[:,2] == 14, 1., -1.)
        normal = -c[:,6,None] * c[:,16:19] * sign[:,None]
        tangent = c[:,19:22] - normal
        rows.append(dict(window_s=[lo,hi],
                         normal_impulse_Ns=(normal.sum(0)*dt).tolist(),
                         tangent_impulse_Ns=(tangent.sum(0)*dt).tolist(),
                         contact_net_work_J=float(c[:,11].sum()*dt),
                         peak_downward_individual_normal_N=float(normal[:,2].min()),
                         peak_downward_individual_tangent_N=float(tangent[:,2].min())))
    records.append(dict(condition=condition, rows=rows))
report = dict(records=records, qualification='Can-normal force reconstructed from recorded normal and Fn, with geometry-side sign. Remaining force is tangential. Contact net work is force on body b dot relative velocity b-a, not actuator work or measured pad energy. Half-open windows; geometry14 is the manipulated can.')
(OUT / 'opening_force_decomposition.json').write_text(json.dumps(report, indent=2))
