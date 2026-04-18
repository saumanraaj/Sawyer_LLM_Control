"""
vision_engine/cv/transforms.py — Coordinate Frame Transforms & Calibration Math

Writers: Nguyen Nguyen (Alan), Sauman Raaj

Coordinate System & Physics
----------------------------
OptiTrack (Motive default): Y-up, right-handed. X=right, Y=up, Z=towards cameras.
Robotics (Z-up):            Z-up, right-handed. X=forward, Y=left, Z=up.

Conversion (Y-up to Z-up):
    x_zup =  x_yup
    y_zup = -z_yup
    z_zup =  y_yup

As a rotation matrix:
    R = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]

This is a 90-degree rotation about the X-axis.

Quaternion convention: (x, y, z, w) — Hamilton convention, matching OptiTrack/ROS.

Provides:
  - Quaternion <-> Euler angle conversion (ZYX convention)
  - Rotation matrix <-> quaternion conversion
  - OptiTrack Y-up -> Z-up frame conversion
  - 4x4 homogeneous transform utilities
  - SVD-based rigid registration (Arun's method) for OptiTrack<->robot calibration
"""

import numpy as np
from typing import Tuple, Optional


# --- Quaternion Utilities ---

def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """
    Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw) in radians.
    Uses ZYX (yaw-pitch-roll) convention, standard in robotics.

    Returns (roll, pitch, yaw) where roll=X, pitch=Y, yaw=Z rotation.
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles (roll, pitch, yaw) in radians to quaternion (x, y, z, w).
    Uses ZYX convention (yaw applied first, then pitch, then roll).
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    R = np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])
    return R


def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """
    Convert a 3x3 rotation matrix to quaternion (x, y, z, w).
    Uses Shepperd's method for numerical stability.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return qx, qy, qz, qw


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions q1 * q2.
    Both as (x, y, z, w). Returns (x, y, z, w).
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


# --- Frame Conversions: OptiTrack Y-up <-> Robotics Z-up ---

# Rotation matrix: Y-up to Z-up
#   x_zup =  x_yup
#   y_zup = -z_yup
#   z_zup =  y_yup
R_YUP_TO_ZUP = np.array([
    [1.0,  0.0,  0.0],
    [0.0,  0.0, -1.0],
    [0.0,  1.0,  0.0],
])

# Corresponding quaternion: 90-degree rotation about X-axis
# q = (sin(45), 0, 0, cos(45)) = (sqrt(2)/2, 0, 0, sqrt(2)/2)
Q_YUP_TO_ZUP = np.array([np.sqrt(2)/2, 0.0, 0.0, np.sqrt(2)/2])


def position_yup_to_zup(pos_yup: np.ndarray) -> np.ndarray:
    """Convert position vector from OptiTrack Y-up to robotics Z-up frame."""
    return R_YUP_TO_ZUP @ pos_yup


def quaternion_yup_to_zup(quat_yup: np.ndarray) -> np.ndarray:
    """Convert quaternion from OptiTrack Y-up to robotics Z-up frame."""
    return quaternion_multiply(Q_YUP_TO_ZUP, quat_yup)


# --- Homogeneous Transforms (4x4) ---

def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a 3x3 rotation and 3-vector translation."""
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


def apply_transform(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to 3D point(s). Accepts (3,) or (N, 3)."""
    single = points.ndim == 1
    if single:
        points = points.reshape(1, 3)

    ones = np.ones((points.shape[0], 1))
    pts_h = np.hstack([points, ones])
    result = (T @ pts_h.T).T[:, :3]

    if single:
        return result.flatten()
    return result


def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    Invert a 4x4 rigid transform efficiently.
    For [R|t], the inverse is [R^T | -R^T @ t].
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


# --- SVD-Based Rigid Registration (Arun's Method) ---

def compute_rigid_transform(
    source_points: np.ndarray,
    target_points: np.ndarray,
) -> Tuple[np.ndarray, float]:
    """
    Compute the best-fit rigid transform mapping source_points to target_points.

    Implements Arun's method (1987): least-squares rigid body registration via SVD.
    Given N >= 3 point correspondences, finds R and t minimizing:
        sum ||target_i - (R @ source_i + t)||^2

    Used for OptiTrack <-> robot frame calibration.

    Parameters
    ----------
    source_points : (N, 3) array — points in source frame (e.g., OptiTrack)
    target_points : (N, 3) array — corresponding points in target frame (e.g., robot)

    Returns
    -------
    T : (4, 4) homogeneous transform matrix (source -> target)
    rms_error : float — root-mean-square registration error in meters

    Reference: Arun, Huang, Blostein (1987), IEEE PAMI 9(5), 698-700.
    """
    source_points = np.asarray(source_points, dtype=np.float64)
    target_points = np.asarray(target_points, dtype=np.float64)

    if source_points.shape != target_points.shape:
        raise ValueError(
            f"Point arrays must have same shape. "
            f"Got source={source_points.shape}, target={target_points.shape}"
        )
    if source_points.shape[0] < 3:
        raise ValueError(
            f"Need at least 3 point correspondences. Got {source_points.shape[0]}."
        )

    # Centroids
    centroid_source = source_points.mean(axis=0)
    centroid_target = target_points.mean(axis=0)

    # Center the points
    source_centered = source_points - centroid_source
    target_centered = target_points - centroid_target

    # Cross-covariance matrix and SVD
    H = source_centered.T @ target_centered
    U, S, Vt = np.linalg.svd(H)

    # Rotation (with reflection correction)
    d = np.linalg.det(Vt.T @ U.T)
    sign_matrix = np.diag([1.0, 1.0, np.sign(d)])
    R = Vt.T @ sign_matrix @ U.T

    # Translation
    t = centroid_target - R @ centroid_source

    # RMS error
    transformed = (R @ source_points.T).T + t
    errors = np.linalg.norm(transformed - target_points, axis=1)
    rms_error = float(np.sqrt(np.mean(errors ** 2)))

    T = make_transform(R, t)
    return T, rms_error


# --- Convenience ---

def euler_degrees_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles in degrees. Returns (roll_deg, pitch_deg, yaw_deg)."""
    r, p, y = quaternion_to_euler(qx, qy, qz, qw)
    return np.degrees(r), np.degrees(p), np.degrees(y)
