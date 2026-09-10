"""Mechanical invariants before any simulation-outcome interpretation."""
import json
from pathlib import Path
import numpy as np
from adaptive_gripper_candidate import Transmission,make_urdf,REPO

model=Transmission();rng=np.random.default_rng(9);errors=[]
for _ in range(100):
    q=rng.uniform(-1,1,4);target=rng.uniform(-1,1,4);eps=1e-6
    gradient=np.array([(model.energy(q+eps*np.eye(4)[j],target)-model.energy(q-eps*np.eye(4)[j],target))/(2*eps) for j in range(4)])
    errors.append(float(np.max(abs(model.torque(q,target)+gradient))))
assert max(errors)<1e-6  # includes actuator saturation and proximal spring reactions
for motor in [0,30,60,90,100]:
    theta=.96-1.05*motor/100;tip=.149-.676*theta
    q=np.array([-theta,theta,tip,tip])
    assert np.max(abs(model.torque(q,q)))<1e-12
theta=.3;tip=.149-.676*theta;q=np.array([-theta,theta,tip,tip])
command_theta=.96-1.05*.9
target=np.array([-command_theta,command_theta,.149-.676*command_theta,.149-.676*command_theta])
torque=model.torque(q,target);assert np.all(torque[2:]<0)
# Right distal-link point 40 mm from its pivot. Its distance from the centerline
# decreases when the distal coordinate decreases: verify curl sign geometrically.
y_pivot=-.030501-.045636*np.sin(theta)+.020423*np.cos(theta)
y0=y_pivot-.04*np.sin(theta+tip);y1=y_pivot-.04*np.sin(theta+tip-.1)
assert abs(y1)<abs(y0)
out=REPO/'paper/eef_recovery_2026-09-09/adaptive_transmission';out.mkdir(exist_ok=True)
info=make_urdf(out/'gen3_lite_2f_adaptive_candidate.urdf')
report=dict(max_energy_gradient_error=max(errors),blocked_proximal_inward_tip_torque=torque[2:].tolist(),
 unloaded_equilibria_checked_motor_percent=[0,30,60,90,100],
 right_distal_point_gap_change_for_negative_0p1rad_m=2*(abs(y1)-abs(y0)),urdf=info)
(out/'mechanics_check.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
