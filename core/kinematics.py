import numpy as np


def normalize(vec):
    vec = np.array(vec, dtype=float)
    norm = np.linalg.norm(vec)
    if norm < 1e-12:
        return np.zeros_like(vec)
    return vec / norm


def rotation_matrix_from_rpy_deg(rpy_deg):
    roll, pitch, yaw = np.radians(np.array(rpy_deg, dtype=float))

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def rpy_deg_from_rotation_matrix(rot):
    rot = np.array(rot, dtype=float)
    sy = np.sqrt(rot[0, 0] ** 2 + rot[1, 0] ** 2)
    singular = sy < 1e-9

    if not singular:
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        pitch = np.arctan2(-rot[2, 0], sy)
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
    else:
        roll = np.arctan2(-rot[1, 2], rot[1, 1])
        pitch = np.arctan2(-rot[2, 0], sy)
        yaw = 0.0

    return np.degrees([roll, pitch, yaw])


def transform_from_pose(position=None, rpy_deg=None):
    tf = np.eye(4, dtype=float)
    if rpy_deg is not None:
        tf[:3, :3] = rotation_matrix_from_rpy_deg(rpy_deg)
    if position is not None:
        tf[:3, 3] = np.array(position, dtype=float)
    return tf


def invert_transform(tf):
    tf = np.array(tf, dtype=float)
    inv = np.eye(4, dtype=float)
    inv[:3, :3] = tf[:3, :3].T
    inv[:3, 3] = -(tf[:3, :3].T @ tf[:3, 3])
    return inv


def pose_dict_from_transform(tf):
    tf = np.array(tf, dtype=float)
    return {
        "position": tf[:3, 3].copy(),
        "rpy_deg": rpy_deg_from_rotation_matrix(tf[:3, :3]),
        "transform": tf.copy(),
    }


def rotation_matrix_to_rotvec(rot):
    rot = np.array(rot, dtype=float)
    trace = np.clip((np.trace(rot) - 1.0) * 0.5, -1.0, 1.0)
    angle = np.arccos(trace)
    if angle < 1e-9:
        return np.zeros(3, dtype=float)

    skew = np.array(
        [
            rot[2, 1] - rot[1, 2],
            rot[0, 2] - rot[2, 0],
            rot[1, 0] - rot[0, 1],
        ],
        dtype=float,
    )
    axis = skew / (2.0 * np.sin(angle) + 1e-12)
    return axis * angle


def pose_error(target_tf, current_tf):
    target_tf = np.array(target_tf, dtype=float)
    current_tf = np.array(current_tf, dtype=float)
    pos_err = target_tf[:3, 3] - current_tf[:3, 3]
    rot_delta = target_tf[:3, :3] @ current_tf[:3, :3].T
    rot_err = rotation_matrix_to_rotvec(rot_delta)
    return np.concatenate([pos_err, rot_err])


def compute_standard_dh_matrix(theta_deg, d, a, alpha_deg, q_value=0.0, joint_type="revolute"):
    joint_type = (joint_type or "revolute").lower()
    if joint_type == "prismatic":
        theta_eff_deg = theta_deg
        d_eff = d + q_value
    else:
        theta_eff_deg = theta_deg + q_value
        d_eff = d

    theta = np.radians(theta_eff_deg)
    alpha = np.radians(alpha_deg)

    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d_eff],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def apply_transform(tf, point):
    tf = np.array(tf, dtype=float)
    point = np.array(point, dtype=float)
    return (tf @ np.append(point, 1.0))[:3]
