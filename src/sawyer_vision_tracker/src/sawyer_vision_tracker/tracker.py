"""Multi-object tracking with persistent IDs via Hungarian algorithm."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment

from sawyer_vision_tracker.detector import Detection
from sawyer_vision_tracker.utils import ema_smooth, euclidean_distance


@dataclass
class TrackedObject:
    """A tracked object with persistent ID across frames."""

    id: int
    centroid: tuple[float, float]           # smoothed pixel position
    raw_centroid: tuple[float, float]       # unfiltered
    position_3d: tuple[float, float, float] | None
    color_name: str
    color_bgr: tuple[int, int, int]
    bbox: tuple[int, int, int, int]
    disappeared: int
    trajectory: deque = field(default_factory=deque)


class ObjectTracker:
    """
    Centroid-based multi-object tracker.

    Assigns persistent integer IDs using the Hungarian algorithm for
    optimal matching between frames. Handles brief disappearances
    and detection flicker.
    """

    def __init__(self, config: dict) -> None:
        self._max_disappeared = config.get("max_disappeared", 30)
        self._max_distance = config.get("max_distance", 80)
        self._alpha = config.get("smoothing_alpha", 0.4)
        self._traj_len = config.get("trajectory_length", 60)

        self._next_id: int = 0
        self._objects: dict[int, TrackedObject] = {}

    def update(self, detections: list[Detection]) -> list[TrackedObject]:
        """
        Update tracker with new detections.
        Returns list of currently tracked objects.
        """
        if len(self._objects) == 0:
            for det in detections:
                self._register(det)
            return list(self._objects.values())

        if len(detections) == 0:
            self._mark_all_disappeared()
            return list(self._objects.values())

        obj_ids = list(self._objects.keys())
        obj_centroids = [self._objects[oid].centroid for oid in obj_ids]
        det_centroids = [det.centroid for det in detections]

        n_obj = len(obj_centroids)
        n_det = len(det_centroids)
        cost = np.zeros((n_obj, n_det), dtype=np.float64)
        for i, oc in enumerate(obj_centroids):
            for j, dc in enumerate(det_centroids):
                cost[i, j] = euclidean_distance(oc, dc)

        row_idx, col_idx = linear_sum_assignment(cost)

        matched_rows: set[int] = set()
        matched_cols: set[int] = set()

        for r, c in zip(row_idx, col_idx):
            if cost[r, c] > self._max_distance:
                continue
            matched_rows.add(r)
            matched_cols.add(c)
            self._update_object(obj_ids[r], detections[c])

        for i in range(n_obj):
            if i not in matched_rows:
                oid = obj_ids[i]
                self._objects[oid].disappeared += 1

        for j in range(n_det):
            if j not in matched_cols:
                self._register(detections[j])

        to_remove = [
            oid
            for oid, obj in self._objects.items()
            if obj.disappeared > self._max_disappeared
        ]
        for oid in to_remove:
            del self._objects[oid]

        return list(self._objects.values())

    def _register(self, detection: Detection) -> None:
        cx, cy = float(detection.centroid[0]), float(detection.centroid[1])
        traj = deque(maxlen=self._traj_len)
        traj.append((cx, cy))

        self._objects[self._next_id] = TrackedObject(
            id=self._next_id,
            centroid=(cx, cy),
            raw_centroid=(cx, cy),
            position_3d=None,
            color_name=detection.color_name,
            color_bgr=detection.color_bgr,
            bbox=detection.bbox,
            disappeared=0,
            trajectory=traj,
        )
        self._next_id += 1

    def _update_object(self, obj_id: int, detection: Detection) -> None:
        obj = self._objects[obj_id]
        raw_cx = float(detection.centroid[0])
        raw_cy = float(detection.centroid[1])

        smooth_cx = ema_smooth(raw_cx, obj.centroid[0], self._alpha)
        smooth_cy = ema_smooth(raw_cy, obj.centroid[1], self._alpha)

        obj.centroid = (smooth_cx, smooth_cy)
        obj.raw_centroid = (raw_cx, raw_cy)
        obj.bbox = detection.bbox
        obj.color_name = detection.color_name
        obj.color_bgr = detection.color_bgr
        obj.disappeared = 0
        obj.trajectory.append((smooth_cx, smooth_cy))

    def _mark_all_disappeared(self) -> None:
        to_remove = []
        for oid, obj in self._objects.items():
            obj.disappeared += 1
            if obj.disappeared > self._max_disappeared:
                to_remove.append(oid)
        for oid in to_remove:
            del self._objects[oid]
