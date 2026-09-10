"""Check exact replay and compare release impulse sources at the conditional IC."""
from pathlib import Path
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
OUT = HERE / 'release_wrenches'
plan = json.loads((OUT / 'plan.json').read_text())
execution = json.loads((OUT / 'execution.json').read_text())
assert len(execution) == 2 and all(row['returncode'] == 0 for row in execution)
sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
assert sha(plan['source']) == plan['source_npz_sha256']
assert sha(Path(plan['source']).with_suffix('.json')) == plan['source_metadata_sha256']
for name, expected in plan['source_code_sha256'].items():
    assert sha(OUT / 'executed_sources' / Path(name).name) == expected
fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True, layout='constrained')
records = []
for col, job in enumerate(plan['jobs']):
    condition = job['condition']
    folder = OUT / condition
    assert sha(job['preset']) == job['preset_sha256']
    assert sha(job['reference']) == job['reference_sha256']
    actual = np.load(folder / '113_eef_delta.npz')
    reference = np.load(job['reference'])
    assert actual.files == reference.files
    exact = {key: bool(np.array_equal(actual[key], reference[key])) for key in actual.files}
    assert all(exact.values())
    force = np.load(folder / 'release_forces.npz')
    states, contacts = force['states'], force['contacts']
    audit = json.loads((folder / 'release_forces_audit.json').read_text())
    assert audit['observer_calls'] == 8 + 24 * len(actual['trajectory'])
    dt, mass = audit['substep_dt_s'], audit['can_mass_kg']
    labels = {int(k): v for k, v in audit['geom_labels'].items()}
    labels.update({0: 'floor', 1: 'shelf', 2: 'pick_table'})
    adjacent = np.isclose(np.diff(states[:, 0]), dt)
    acc = np.diff(states[:, 4:7], axis=0)[adjacent] / dt
    predicted = states[:-1, 10:13][adjacent] / mass + [0, 0, -9.81]
    acc_error = np.linalg.norm(acc - predicted, axis=1)
    windows = [(19.8, 20.1), (20.1, 20.16), (20.16, 20.24),
               (20.24, 20.35), (20.35, 20.6)]
    balances = []
    for lo, hi in windows:
        rows = states[(states[:, 0] >= lo) & (states[:, 0] <= hi)]
        momentum = mass * (rows[-1, 4:7] - rows[0, 4:7])
        integral = (rows[:-1, 10:13] + [0, 0, -9.81 * mass]).sum(axis=0) * dt
        balances.append(dict(window_s=[lo, hi], momentum_change_Ns=momentum.tolist(),
                             contact_plus_gravity_integral_Ns=integral.tolist(),
                             residual_Ns=(momentum-integral).tolist()))
    groups = []
    for gid in sorted(set(contacts[:, 3].astype(int))):
        rows = contacts[contacts[:, 3] == gid]
        loaded = rows[rows[:, 6] > .05]
        groups.append(dict(geom=int(gid), label=labels.get(gid, 'other'),
                           loaded_first_s=float(loaded[0, 0]) if len(loaded) else None,
                           loaded_last_s=float(loaded[-1, 0]) if len(loaded) else None,
                           impulse_Ns=(rows[:, 19:22].sum(axis=0)*dt).tolist(),
                           peak_force_N=float(np.linalg.norm(rows[:, 19:22], axis=1).max()),
                           loaded_utilization_p50_p95_max=np.percentile(loaded[:, 8], [50, 95, 100]).tolist() if len(loaded) else None))
    snapshots = []
    for t in [20.04, 20.1, 20.13, 20.16, 20.19, 20.22, 20.25, 20.34, 20.5]:
        row = states[np.argmin(abs(states[:, 0]-t))]
        snapshots.append(dict(time_s=float(row[0]), can_xyz_m=row[1:4].tolist(),
                              velocity_m_s=row[4:7].tolist(), finger_force_N=row[16:19].tolist()))
    record = dict(condition=condition, exact_reference_arrays=exact,
                  acceleration_balance_error_m_s2_p50_p95_max=np.percentile(acc_error, [50,95,100]).tolist(),
                  momentum_checks=balances, contact_groups=groups, snapshots=snapshots)
    records.append(record)
    axes[0, col].set_title(condition + ' pads; conditional image-derived start')
    axes[0, col].plot(states[:, 0], 1000*(states[:, 1]-.55), label='Can center past shelf front')
    axes[0, col].axhline(0, color='black', ls='--')
    axes[1, col].plot(states[:, 0], states[:, 4], label='Can x velocity')
    axes[2, col].plot(states[:, 0], states[:, 16], label='Finger force x')
    for gid, label in [(1, 'Shelf'), (2, 'Pick table')]:
        summed = np.zeros(len(states))
        for row in contacts[contacts[:, 3] == gid]:
            i = int(np.argmin(abs(states[:, 0]-row[0])))
            assert abs(states[i, 0]-row[0]) < 1e-8
            summed[i] += row[19]
        axes[2, col].plot(states[:, 0], summed, label=label+' force x')
    for row in range(3):
        axes[row, col].grid(alpha=.2)
        axes[row, col].legend(fontsize=8)
    axes[2, col].set_xlabel('Replay time (s)')
axes[0,0].set_ylabel('Horizontal margin (mm)')
axes[1,0].set_ylabel('Velocity x (m/s)')
axes[2,0].set_ylabel('Force x (N)')
fig.suptitle('113: exact conditional rigid/soft release comparison\nOriginal source motion; both full tasks fail. Initial XY remains unadopted.')
fig.savefig(OUT / 'release_comparison.png', dpi=150)
report = dict(records=records, all_terminal=True,
              qualification='Every-substep observation; every reference trace array exactly reproduced. Conditional image-only initial pose is not adopted. Geometry labels0/1/2 follow checked floor/shelf/pick-table creation order. Loaded0.05N cutoff is diagnostic only; scores unchanged. Force residuals are reported, not fitted away.')
(OUT / 'summary.json').write_text(json.dumps(report, indent=2))
print(json.dumps([{k:v for k,v in r.items() if k != 'exact_reference_arrays'} for r in records], indent=2))
