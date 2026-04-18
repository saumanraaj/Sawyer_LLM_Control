"""
vision_engine/cv/cs100_model.py — CS-100 Calibration Square L-Shape Geometry

The OptiTrack CS-100 Calibration Square has 3 retroreflective markers
arranged in an L-shape:

    Corner marker ── 8cm ──> Short arm marker
         |
        10cm
         |
         v
    Long arm marker

All three markers lie in a plane (the CS-100 plate surface).

When OptiTrack tracks the CS-100 as a rigid body, Motive reports:
  - position: the pivot point (typically the marker centroid)
  - quaternion: the rigid body's orientation

This module computes the world-space positions of all 3 markers from
the rigid body pose + known L-shape geometry. It also validates
tracking accuracy by checking inter-marker distances.
"""

import numpy as np
from typing import Optional, List
from cv.transforms import quaternion_to_rotation_matrix


class CS100Geometry:
    """
    Encodes the CS-100 L-shape marker layout and provides methods
    to compute world-space marker positions from a rigid body pose.

    Parameters
    ----------
    short_arm_length : float
        Distance from corner to short arm marker (meters). Default 0.08.
    long_arm_length : float
        Distance from corner to long arm marker (meters). Default 0.10.
    distance_tolerance_mm : float
        Max allowed deviation from expected distances (mm). Default 2.0.
    """

    def __init__(
        self,
        short_arm_length: float = 0.08,
        long_arm_length: float = 0.10,
        distance_tolerance_mm: float = 2.0,
    ):
        self.short_arm_length = short_arm_length
        self.long_arm_length = long_arm_length
        self.distance_tolerance_mm = distance_tolerance_mm

        # Expected distances
        self.expected_hypotenuse = np.sqrt(
            short_arm_length**2 + long_arm_length**2
        )

        # Marker positions in body-local frame (before pivot offset).
        # Corner at origin, short arm along +X, long arm along +Y.
        self._corner_raw = np.array([0.0, 0.0, 0.0])
        self._short_arm_raw = np.array([short_arm_length, 0.0, 0.0])
        self._long_arm_raw = np.array([0.0, long_arm_length, 0.0])

        # Motive sets the rigid body pivot at the marker centroid by default.
        centroid = np.mean(
            [self._corner_raw, self._short_arm_raw, self._long_arm_raw],
            axis=0,
        )

        # Body-frame offsets from pivot (centroid)
        self.corner_local = self._corner_raw - centroid
        self.short_arm_local = self._short_arm_raw - centroid
        self.long_arm_local = self._long_arm_raw - centroid

        # Stack for vectorized transforms: (3, 3) array
        self.markers_local = np.array([
            self.corner_local,
            self.short_arm_local,
            self.long_arm_local,
        ], dtype=np.float64)

    def compute_marker_positions(
        self,
        position: np.ndarray,
        quaternion: np.ndarray,
    ) -> np.ndarray:
        """
        Compute world-space positions of the 3 markers.

        Parameters
        ----------
        position : np.ndarray, shape (3,)
            Rigid body position (pivot point) from OptiTrack.
        quaternion : np.ndarray, shape (4,)
            Rigid body orientation (x, y, z, w) from OptiTrack.

        Returns
        -------
        markers : np.ndarray, shape (3, 3)
            World positions: [corner, short_arm, long_arm].
        """
        R = quaternion_to_rotation_matrix(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )
        # Rotate local offsets and translate by rigid body position
        markers_world = (R @ self.markers_local.T).T + position
        return markers_world

    def validate_geometry(self, marker_positions: np.ndarray) -> dict:
        """
        Check inter-marker distances against expected values.

        Parameters
        ----------
        marker_positions : np.ndarray, shape (3, 3)
            World positions: [corner, short_arm, long_arm].

        Returns
        -------
        result : dict
            Keys: short_arm_dist_m, long_arm_dist_m, hypotenuse_dist_m,
                  short_arm_error_mm, long_arm_error_mm, hypotenuse_error_mm,
                  is_valid.
        """
        corner = marker_positions[0]
        short_arm = marker_positions[1]
        long_arm = marker_positions[2]

        d_short = float(np.linalg.norm(short_arm - corner))
        d_long = float(np.linalg.norm(long_arm - corner))
        d_hyp = float(np.linalg.norm(long_arm - short_arm))

        err_short = abs(d_short - self.short_arm_length) * 1000.0
        err_long = abs(d_long - self.long_arm_length) * 1000.0
        err_hyp = abs(d_hyp - self.expected_hypotenuse) * 1000.0

        is_valid = all(
            e < self.distance_tolerance_mm
            for e in [err_short, err_long, err_hyp]
        )

        return {
            "short_arm_dist_m": d_short,
            "long_arm_dist_m": d_long,
            "hypotenuse_dist_m": d_hyp,
            "short_arm_error_mm": err_short,
            "long_arm_error_mm": err_long,
            "hypotenuse_error_mm": err_hyp,
            "is_valid": is_valid,
        }

    def get_l_frame_axes(
        self,
        position: np.ndarray,
        quaternion: np.ndarray,
    ) -> dict:
        """
        Compute the L-shape's local coordinate frame in world coordinates.

        Returns
        -------
        axes : dict
            origin: corner marker world position
            x_axis: unit vector from corner toward short arm (8cm)
            y_axis: unit vector from corner toward long arm (10cm)
            z_axis: surface normal (x_axis × y_axis)
        """
        markers = self.compute_marker_positions(position, quaternion)
        corner = markers[0]
        short_arm = markers[1]
        long_arm = markers[2]

        x_axis = short_arm - corner
        x_axis = x_axis / np.linalg.norm(x_axis)

        y_axis = long_arm - corner
        y_axis = y_axis / np.linalg.norm(y_axis)

        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)

        return {
            "origin": corner,
            "x_axis": x_axis,
            "y_axis": y_axis,
            "z_axis": z_axis,
        }

    def estimate_floor_plane(
        self,
        positions: List[np.ndarray],
        quaternions: List[np.ndarray],
        camera_origin: Optional[np.ndarray] = None,
    ) -> dict:
        """
        Estimate the floor plane from multiple samples of the CS-100
        placed flat on the floor.

        Parameters
        ----------
        positions : list of np.ndarray, each shape (3,)
            Rigid body positions over N frames.
        quaternions : list of np.ndarray, each shape (4,)
            Rigid body orientations over N frames.
        camera_origin : np.ndarray or None
            Camera position in world frame. Defaults to [0, 0, 0]
            (V120:Trio middle camera ≈ OptiTrack origin).

        Returns
        -------
        result : dict
            floor_z, floor_point, floor_normal, camera_to_floor_depth,
            flatness_deg, marker_validation, num_samples, position_std.
        """
        if camera_origin is None:
            camera_origin = np.array([0.0, 0.0, 0.0])

        all_markers = []
        all_normals = []

        for pos, quat in zip(positions, quaternions):
            markers = self.compute_marker_positions(pos, quat)
            all_markers.append(markers)

            axes = self.get_l_frame_axes(pos, quat)
            all_normals.append(axes["z_axis"])

        all_markers = np.array(all_markers)  # (N, 3, 3)
        all_normals = np.array(all_normals)  # (N, 3)

        # Average marker positions and floor normal
        avg_markers = np.mean(all_markers, axis=0)  # (3, 3)
        avg_normal = np.mean(all_normals, axis=0)
        avg_normal = avg_normal / np.linalg.norm(avg_normal)

        # Floor point = centroid of averaged markers
        floor_point = np.mean(avg_markers, axis=0)
        floor_z = float(floor_point[2])

        # Flatness check: angle between floor normal and Z-up
        z_up = np.array([0.0, 0.0, 1.0])
        cos_angle = abs(np.dot(avg_normal, z_up))
        flatness_deg = float(np.degrees(np.arccos(np.clip(cos_angle, -1, 1))))

        # Camera to floor depth (vertical)
        camera_to_floor_depth = float(abs(camera_origin[2] - floor_z))

        # Validate geometry on averaged markers
        validation = self.validate_geometry(avg_markers)

        # Position standard deviation
        all_positions = np.array([pos for pos in positions])
        position_std = float(np.mean(np.std(all_positions, axis=0)))

        return {
            "floor_z": floor_z,
            "floor_point": floor_point.tolist(),
            "floor_normal": avg_normal.tolist(),
            "camera_to_floor_depth": camera_to_floor_depth,
            "flatness_deg": flatness_deg,
            "marker_validation": validation,
            "num_samples": len(positions),
            "position_std_mm": position_std * 1000.0,
        }
