"""Verify unchanged replays, report force balance and locate release impulses."""
from pathlib import Path
import hashlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent
plan=json.loads((R/'plan.json').read_text());execution=json.loads((R/'execution.json').read_text())
assert len(execution)==2 and all(r['returncode']==0 for r in execution)
for f,sha in plan['source_code_sha256'].items():assert hashlib.sha256((R/'executed_sources'/Path(f).name).read_bytes()).hexdigest()==sha
records=[]
for job in plan['jobs']:
    uid=job['uid'];d=R/str(uid);a=np.load(d/f'{uid}_eef_delta.npz');b=np.load(job['reference'])
    assert hashlib.sha256(Path(job['reference']).read_bytes()).hexdigest()==job['reference_sha256']
    assert a.files==b.files;exact={k:bool(np.array_equal(a[k],b[k])) for k in a.files};assert all(exact.values())
    z=np.load(d/'release_forces.npz');s=z['states'];c=z['contacts'];audit=json.loads((d/'release_forces_audit.json').read_text())
    assert audit['observer_calls']==8+24*len(a['trajectory'])
    mass=audit['can_mass_kg'];dt=audit['substep_dt_s'];adjacent=np.isclose(np.diff(s[:,0]),dt)
    acc=np.diff(s[:,4:7],axis=0)[adjacent]/dt;force_acc=s[:-1,10:13][adjacent]/mass+[0,0,-9.81]
    accel_error=np.linalg.norm(acc-force_acc,axis=1)
    windows=[(19.8,20.1),(20.1,20.16),(20.16,20.24),(20.24,20.35),(20.35,20.5)] if uid==113 else [(14,14.1),(20,21),(21,22),(22,25)]
    balance=[]
    for lo,hi in windows:
        rows=s[(s[:,0]>=lo)&(s[:,0]<=hi)];duration=rows[-1,0]-rows[0,0]
        dp=mass*(rows[-1,4:7]-rows[0,4:7]);impulse=(rows[:-1,10:13]+[0,0,-9.81*mass]).sum(axis=0)*dt
        balance.append(dict(window_s=[lo,hi],sampled_duration_s=duration,momentum_change_Ns=dp.tolist(),net_force_integral_Ns=impulse.tolist(),residual_Ns=(dp-impulse).tolist()))
    groups=[]
    labels={int(k):v for k,v in audit['geom_labels'].items()}
    # Full-world builder adds one plane, one shelf box, and one pick table before
    # the robot. Their single-geometry indices are0/1/2; x shelf edge is0.55m.
    labels.update({0:'floor',1:'shelf',2:'pick_table'})
    event_start=19.8 if uid==113 else 20
    for gid in sorted(set(c[:,3].astype(int))):
        rows=c[(c[:,3]==gid)&(c[:,0]>=event_start)];loaded=rows[rows[:,6]>.05]
        if not len(rows):continue
        groups.append(dict(geom=int(gid),label=labels.get(gid,'other'),contact_rows=len(rows),loaded_first_s=float(loaded[0,0]) if len(loaded) else None,loaded_last_s=float(loaded[-1,0]) if len(loaded) else None,contact_impulse_Ns=(rows[:,19:22].sum(axis=0)*dt).tolist(),peak_force_N=float(np.linalg.norm(rows[:,19:22],axis=1).max())))
    records.append(dict(uid=uid,exact_reference_arrays=exact,acceleration_balance_error_m_s2_p50_p95_max=np.percentile(accel_error,[50,95,100]).tolist(),momentum_checks=balance,contact_groups=groups))
    if uid==113:
        fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True,layout='constrained')
        mask=s[:,0]>=19.8;st=s[mask]
        axes[0].plot(st[:,0],1000*(st[:,1]-.55),label='Can center past shelf front edge')
        axes[0].axhline(0,color='black',ls='--',label='Shelf front edge');axes[0].set_ylabel('Horizontal margin (mm)');axes[0].legend()
        axes[1].plot(st[:,0],st[:,4],label='Can horizontal velocity');axes[1].set_ylabel('Velocity x (m/s)');axes[1].legend()
        axes[2].plot(st[:,0],st[:,16],label='Fingers: force x')
        shelf=np.zeros(len(st));other=np.zeros(len(st))
        for i,t in enumerate(st[:,0]):
            at=c[np.isclose(c[:,0],t,atol=1e-8,rtol=0)];shelf[i]=at[at[:,3]==1,19].sum();other[i]=at[at[:,3]==2,19].sum()
        axes[2].plot(st[:,0],shelf,label='Shelf: force x');axes[2].plot(st[:,0],other,label='Pick table: force x')
        axes[2].set(xlabel='Replay time (s)',ylabel='Force x (N)');axes[2].legend()
        for ax in axes:
            ax.axvline(20.1575,color='purple',ls=':',label='Last loaded finger sample')
            ax.axvline(20.21125,color='orange',ls=':');ax.grid(alpha=.2)
        fig.suptitle('113: the shelf edge pushes the released can backward\nPurple: last loaded finger sample; orange: first loaded shelf sample. Original motion and world.')
        fig.savefig(R/'113_edge_impulse.png',dpi=150)
report=dict(records=records,qualification='Every-substep observer, exact full traces. Force signs checked against linear momentum. Individual233 acceleration residuals are larger; report them and do not assume force reporting is exact. The113 shelf impulse dominates the post-release backward motion. No pad, world, motion or scoring change.')
(R/'summary.json').write_text(json.dumps(report,indent=2))
print(json.dumps([{k:v for k,v in r.items() if k not in ['exact_reference_arrays','contact_groups']} for r in records],indent=2))
