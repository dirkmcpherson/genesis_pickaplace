"""Isolated pad contact damping at unchanged elastic stiffness and geometry."""
from pathlib import Path
import argparse
import json
import runpy
import sys
import numpy as np
import quadrants as qd
import genesis as gs
from genesis.utils import geom as gu
import surface_pad_candidate_g14 as material

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('source', type=Path)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--preset', type=Path, required=True)
p.add_argument('--gain', type=float, choices=[1., 2.], required=True)
a = p.parse_args()
Base = material.SurfacePads
instance = None

@qd.data_oriented
class DampedPads(Base):
    def __init__(self, *args, **kwargs):
        global instance
        self.gain = a.gain
        super().__init__(*args, **kwargs)
        self.applied_rows = qd.field(dtype=qd.i32, shape=())
        self.errors = qd.Vector.field(3, dtype=gs.qd_float, shape=())
        self.min_timeconst = qd.field(dtype=gs.qd_float, shape=())
        self.min_timeconst[None] = 1.
        instance = self

    def apply(self):
        super().apply()
        if self.gain != 1.:
            self.damp()

    @qd.kernel
    def damp(self):
        for i in range(self.contacts.n_contacts[0]):
            c = self.contacts.contact_data
            ga, gb = c.geom_a[i,0], c.geom_b[i,0]
            cn, cp, penetration = c.normal[i,0], c.pos[i,0], c.penetration[i,0]
            selected = 0
            for side in qd.static(range(2)):
                g = ga if side == 0 else gb
                if self.enabled[g] and self.compliant[g]:
                    outward = -cn if side == 0 else cn
                    surface = cp + outward * penetration * .5
                    link = self.link_index[g]
                    local = gu.qd_inv_transform_by_trans_quat(surface, self.state.links.pos[link,0], self.state.links.quat[link,0])
                    local_n = gu.qd_inv_transform_by_quat(outward, self.state.links.quat[link,0])
                    r = self.region[g]
                    normal = qd.Vector([r[0],r[1],r[2]])
                    cosine = normal.dot(local_n)
                    thickness = (normal.dot(local)-r[3])/qd.max(cosine,1e-6)
                    if local[0] < -.008 and cosine > .5 and thickness > 0:
                        selected = 1
            if selected:
                old = c.sol_params[i,0]
                new = old
                new[0] = old[0] / self.gain
                new[1] = old[1] * self.gain
                # Use the engine's executed reference-acceleration function to
                # verify stiffness and both velocity terms, not just a formula.
                _, old0 = gu.imp_aref(old, -penetration, 0., -penetration)
                _, new0 = gu.imp_aref(new, -penetration, 0., -penetration)
                _, old1 = gu.imp_aref(old, -penetration, 1., -penetration)
                _, new1 = gu.imp_aref(new, -penetration, 1., -penetration)
                _, oldt = gu.imp_aref(old, -penetration, 1., 0.)
                _, newt = gu.imp_aref(new, -penetration, 1., 0.)
                qd.atomic_max(self.errors[None][0], qd.abs(new0-old0)/qd.max(qd.abs(old0),1e-6))
                qd.atomic_max(self.errors[None][1], qd.abs((new0-new1)-self.gain*(old0-old1))/qd.max(qd.abs(self.gain*(old0-old1)),1e-6))
                qd.atomic_max(self.errors[None][2], qd.abs(newt-self.gain*oldt)/qd.max(qd.abs(self.gain*oldt),1e-6))
                qd.atomic_min(self.min_timeconst[None], new[0])
                qd.atomic_add(self.applied_rows[None], 1)
                c.sol_params[i,0] = new

    def audit(self):
        result = super().audit()
        result['contact_damping'] = dict(gain=self.gain,
            transform='At classified inner-pad contacts after layered softness: tc -> tc/gain, dampratio -> dampratio*gain.',
            scope='All contacting objects; both normal and tangential reference velocity damping. Elastic coefficient and impedance unchanged at fixed penetration. No geometry, friction coefficient, transmission, source motion or initial-pose changes.',
            qualification='Uncalibrated dissipative-contact hypothesis. Coupled tangential damping is explicitly part of the treatment; it is not a pure normal-damping ablation.')
        return result

material.SurfacePads = DampedPads
sys.argv = ['run_g14_hand.py',str(a.source),'--out',str(a.out),'--preset',str(a.preset),'--cone','elliptic','--impratio','10']
try:
    runpy.run_path(str(ROOT/'can_pos_recovery/run_g14_hand.py'), run_name='__main__')
finally:
    if instance is not None:
        errors = instance.errors.to_numpy().tolist()
        count = int(instance.applied_rows[None])
        minimum = float(instance.min_timeconst[None])
        report = dict(gain=a.gain,applied_contact_rows=count,
            max_relative_error_static_normal_normal_velocity_tangent_velocity=errors,
            min_applied_timeconst_s=minimum if count else None,
            minimum_safe_timeconst_s=2*instance.solver._substep_dt,
            qualification='Every treated substep contact checked using installed gu.imp_aref. Static normal term unchanged; both velocity reference terms multiply by gain. Identity control bypasses damping writes.')
        (a.out/'pad_damping_audit.json').write_text(json.dumps(report,indent=2))
        assert max(errors) < 1e-4, report
        if count:
            assert minimum >= 2*instance.solver._substep_dt, report
