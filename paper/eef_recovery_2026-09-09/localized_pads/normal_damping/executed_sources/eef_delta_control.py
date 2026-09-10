"""Explicit world-frame tool displacement control, isolated from frozen controllers.

Action: dx,dy,dz in metres; left-multiplied rotation increment as rotvec radians.
Deltas integrate onto the previous target (zero holds the target). The tool frame
is the URDF tool_frame, including its fixed rotation, not a reset calibration.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def transform(pos, rotation):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = pos
    return t


def from_genesis(pos, quat):
    return transform(pos, Rotation.from_quat(np.asarray(quat)[[1, 2, 3, 0]]).as_matrix())


def to_genesis(t):
    return t[:3, 3], Rotation.from_matrix(t[:3, :3]).as_quat()[[3, 0, 1, 2]]


def pose_delta(previous, following):
    return np.r_[following[:3, 3] - previous[:3, 3],
                 Rotation.from_matrix(following[:3, :3] @ previous[:3, :3].T).as_rotvec()]


def apply_delta(previous, delta):
    a = np.asarray(delta, dtype=float)
    if a.shape != (6,) or not np.isfinite(a).all():
        raise ValueError('Expected six finite world-frame pose increments')
    return transform(previous[:3, 3] + a[:3],
                     Rotation.from_rotvec(a[3:]).as_matrix() @ previous[:3, :3])


class ArmKinematics:
    """URDF FK without changing any simulator state; six named arm joints only."""
    def __init__(self, urdf):
        root = ET.parse(Path(urdf)).getroot()
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain = []
        link = 'tool_frame'
        while link in by_child:
            joint = by_child[link]
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            axis = joint.find('axis')
            axis = np.fromstring(axis.get('xyz', '0 0 1'), sep=' ') if axis is not None else None
            chain.append((joint.get('name'), joint.get('type'),
                          transform(xyz, Rotation.from_euler('xyz', rpy).as_matrix()), axis,
                          link))
            link = joint.find('parent').get('link')
        self.base_link = link
        self.chain = chain[::-1]
        self.wrist_to_tool = np.eye(4)
        after_wrist = False
        for _, kind, origin, _, child in self.chain:
            if after_wrist:
                if kind != 'fixed':
                    raise ValueError('Tool must be rigidly attached to wrist')
                self.wrist_to_tool = self.wrist_to_tool @ origin
            if child == 'end_effector_link':
                after_wrist = True
        if not after_wrist:
            raise ValueError('Missing end_effector_link')

    def tool(self, joints, mount):
        q = dict(zip((f'joint_{i}' for i in range(1, 7)), joints))
        t = np.array(mount, copy=True)
        for name, kind, origin, axis, _ in self.chain:
            t = t @ origin
            if kind != 'fixed':
                if kind not in ('revolute', 'continuous') or name not in q:
                    raise ValueError(f'Unsupported arm joint {name}: {kind}')
                t = t @ transform(np.zeros(3), Rotation.from_rotvec(axis * q[name]).as_matrix())
        return t


class EEFDeltaController:
    def __init__(self, robot, wrist, arm_indices, wrist_to_tool, respect_joint_limit=True):
        self.robot, self.wrist = robot, wrist
        self.arm_indices = arm_indices
        self.respect_joint_limit = respect_joint_limit
        self.tool_to_wrist = np.linalg.inv(wrist_to_tool)
        self.wrist_to_tool = wrist_to_tool
        self.reset()

    @staticmethod
    def array(x):
        return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)

    def measured_tool(self):
        return from_genesis(self.array(self.wrist.get_pos()),
                            self.array(self.wrist.get_quat())) @ self.wrist_to_tool

    def reset(self):
        self.target = self.measured_tool()
        self.seed = self.array(self.robot.get_qpos()).copy()

    def command(self, delta):
        self.target = apply_delta(self.target, delta)
        wrist_target = self.target @ self.tool_to_wrist
        pos, quat = to_genesis(wrist_target)
        q, error = self.robot.inverse_kinematics(
            link=self.wrist, pos=pos, quat=quat, init_qpos=self.seed,
            respect_joint_limit=self.respect_joint_limit,
            dofs_idx_local=self.arm_indices, max_samples=1, max_solver_iters=50,
            pos_tol=1e-5, rot_tol=1e-4, return_error=True)
        self.seed = self.array(q).copy()
        self.last_ik_error = self.array(error).copy()
        return self.seed[self.arm_indices]


class FKPrecisionRefiner:
    """Double-precision local IK correction for recorded Cartesian paths.

    Genesis supplies the branch/initial solution. This refinement uses only the
    desired tool pose and URDF geometry, never the source joint waypoint. It is
    for the explicitly unclipped-target recovery mode; physical limits remain.
    """
    def __init__(self, kinematics, mount):
        self.fk = kinematics
        self.mount = np.asarray(mount)

    def jacobian(self, q):
        t = self.mount.copy()
        points, axes = [], []
        for name, kind, origin, axis, _ in self.fk.chain:
            t = t @ origin
            if kind != 'fixed':
                points.append(t[:3, 3].copy())
                axes.append(t[:3, :3] @ axis)
                t = t @ transform(np.zeros(3), Rotation.from_rotvec(axis*q[int(name.split('_')[-1])-1]).as_matrix())
        jac = np.empty((6, 6))
        for i, (point, axis) in enumerate(zip(points, axes)):
            jac[:3, i] = np.cross(axis, t[:3, 3]-point)
            jac[3:, i] = axis
        return t, jac

    def refine(self, seed, target):
        q = np.asarray(seed, dtype=np.float64).copy()
        for _ in range(10):
            actual, jac = self.jacobian(q)
            error = pose_delta(actual, target)
            if np.linalg.norm(error[:3]) < 1e-11 and np.linalg.norm(error[3:]) < 1e-11:
                return q
            change = np.linalg.lstsq(jac, error, rcond=1e-10)[0]
            change *= min(1., .05/max(np.max(np.abs(change)), 1e-15))
            q += change
        error = pose_delta(self.fk.tool(q, self.mount), target)
        if np.linalg.norm(error[:3]) > 1e-7 or np.linalg.norm(error[3:]) > 1e-7:
            raise RuntimeError(f'Precision IK failed to converge: {error}')
        return q
