"""
vision_engine/cv/depth_estimator.py — Floor Depth Estimation

Estimates the depth (distance) from the V120:Trio middle camera to the
floor plane using the CS-100 Calibration Square placed flat on the floor.

Workflow:
  1. Place CS-100 flat on the floor under the V120:Trio
  2. Record N frames of pose data (position + quaternion)
  3. Compute marker positions using known L-shape geometry
  4. Validate inter-marker distances (8cm, 10cm)
  5. Estimate floor plane (average position + surface normal)
  6. Depth = distance from camera origin to floor plane

The V120:Trio middle camera is approximately at the OptiTrack coordinate
origin (after Motive's wanding calibration). The floor position is
determined by where the CS-100 markers are.
"""

import numpy as np
from typing import Optional
from cv.cs100_model import CS100Geometry


class FloorDepthEstimator:
    """
    Accumulates CS-100 pose samples and computes camera-to-floor depth.

    Parameters
    ----------
    cs100_model : CS100Geometry
        The L-shape geometry model.
    camera_origin : np.ndarray or None
        Camera position in OptiTrack frame. Defaults to [0, 0, 0].
    """

    def __init__(
        self,
        cs100_model: CS100Geometry,
        camera_origin: Optional[np.ndarray] = None,
    ):
        self.cs100_model = cs100_model
        self.camera_origin = (
            camera_origin if camera_origin is not None
            else np.array([0.0, 0.0, 0.0])
        )

        self._positions = []
        self._quaternions = []
        self._result = None

    @property
    def num_samples(self) -> int:
        return len(self._positions)

    def reset(self) -> None:
        """Clear all recorded samples."""
        self._positions.clear()
        self._quaternions.clear()
        self._result = None

    def record_sample(
        self,
        position: np.ndarray,
        quaternion: np.ndarray,
    ) -> None:
        """
        Record one frame of CS-100 pose data.

        Parameters
        ----------
        position : np.ndarray, shape (3,)
        quaternion : np.ndarray, shape (4,)
        """
        self._positions.append(position.copy())
        self._quaternions.append(quaternion.copy())

    def compute(self, min_samples: int = 30) -> dict:
        """
        Compute the floor plane and camera-to-floor depth.

        Parameters
        ----------
        min_samples : int
            Minimum number of samples required.

        Returns
        -------
        result : dict
            floor_z, floor_point, floor_normal, camera_to_floor_depth,
            flatness_deg, marker_validation, num_samples, position_std_mm.

        Raises
        ------
        RuntimeError
            If fewer than min_samples have been recorded.
        """
        if len(self._positions) < min_samples:
            raise RuntimeError(
                f"Need at least {min_samples} samples, "
                f"got {len(self._positions)}."
            )

        self._result = self.cs100_model.estimate_floor_plane(
            self._positions,
            self._quaternions,
            camera_origin=self.camera_origin,
        )
        return self._result

    @property
    def depth(self) -> Optional[float]:
        """Camera-to-floor depth in meters, or None if not computed."""
        if self._result is None:
            return None
        return self._result["camera_to_floor_depth"]

    @property
    def floor_z(self) -> Optional[float]:
        """Floor Z coordinate in OptiTrack frame, or None."""
        if self._result is None:
            return None
        return self._result["floor_z"]

    def print_report(self) -> None:
        """Print a human-readable report of the depth estimation."""
        if self._result is None:
            print("[DepthEstimator] No results. Call compute() first.")
            return

        r = self._result
        v = r["marker_validation"]

        print(f"\n{'='*60}")
        print("Floor Depth Estimation Results")
        print(f"{'='*60}")
        print(f"\n  Camera-to-floor depth: {r['camera_to_floor_depth']:.4f} m "
              f"({r['camera_to_floor_depth']*100:.1f} cm)")
        print(f"  Floor Z coordinate:    {r['floor_z']:.4f} m")
        print(f"  Floor normal:          ({r['floor_normal'][0]:.4f}, "
              f"{r['floor_normal'][1]:.4f}, {r['floor_normal'][2]:.4f})")
        print(f"  Flatness:              {r['flatness_deg']:.1f}° "
              f"(should be < 5°)")

        print(f"\n  Geometry validation:")
        print(f"    Short arm (8cm): {v['short_arm_dist_m']*100:.2f} cm "
              f"(error: {v['short_arm_error_mm']:.1f} mm)")
        print(f"    Long arm (10cm): {v['long_arm_dist_m']*100:.2f} cm "
              f"(error: {v['long_arm_error_mm']:.1f} mm)")
        print(f"    Hypotenuse:      {v['hypotenuse_dist_m']*100:.2f} cm "
              f"(error: {v['hypotenuse_error_mm']:.1f} mm)")
        print(f"    Valid: {'YES' if v['is_valid'] else 'NO'}")

        print(f"\n  Samples:    {r['num_samples']}")
        print(f"  Pos. std:   {r['position_std_mm']:.2f} mm")
        print(f"{'='*60}\n")
