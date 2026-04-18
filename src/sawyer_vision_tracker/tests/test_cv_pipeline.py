#!/usr/bin/env python3
"""
Standalone CV pipeline tests for sawyer_vision_tracker.
No ROS required — tests detector, tracker, and utils directly.

Usage:
    python3 test_cv_pipeline.py [-v]
"""

import sys
import os
import math
import unittest

import cv2
import numpy as np

# ── path setup so we can import the package without a ROS workspace ──────────
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR   = os.path.dirname(_TESTS_DIR)
_SRC_DIR   = os.path.join(_PKG_DIR, "src")
sys.path.insert(0, _SRC_DIR)

from sawyer_vision_tracker.utils    import euclidean_distance, ema_smooth
from sawyer_vision_tracker.detector import Detector, Detection
from sawyer_vision_tracker.tracker  import ObjectTracker

# ── shared configs (mirror vision_tracker.yaml) ───────────────────────────────
DETECTOR_CFG = {
    "hsv_ranges": [
        {
            "name": "blue",
            "lower": [100, 120, 70],
            "upper": [130, 255, 255],
            "color_bgr": [255, 0, 0],
        },
        {
            "name": "red",
            "lower": [0, 120, 70],
            "upper": [10, 255, 255],
            "lower2": [170, 120, 70],
            "upper2": [180, 255, 255],
            "color_bgr": [0, 0, 255],
        },
        {
            "name": "green",
            "lower": [35, 120, 70],
            "upper": [85, 255, 255],
            "color_bgr": [0, 255, 0],
        },
    ],
    "min_contour_area": 500,
    "morph_kernel_size": 5,
    "gaussian_blur": 5,
}

TRACKER_CFG = {
    "max_disappeared": 5,
    "max_distance": 80,
    "smoothing_alpha": 0.4,
    "trajectory_length": 20,
}

# ── helpers ───────────────────────────────────────────────────────────────────
H, W = 480, 640
BLOB_SIZE = 90  # large enough to survive blur + morph erosion and stay > 500 px²


def make_blob_frame(cx: int, cy: int, hsv: tuple, h: int = H, w: int = W,
                    size: int = BLOB_SIZE) -> np.ndarray:
    """
    Return a BGR frame with one solid-colour blob centred at (cx, cy).
    The blob colour is specified in HSV and converted to BGR.
    """
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    x0 = max(0, cx - size // 2)
    y0 = max(0, cy - size // 2)
    x1 = min(w, x0 + size)
    y1 = min(h, y0 + size)
    patch_h, patch_w = y1 - y0, x1 - x0
    patch_hsv = np.full((patch_h, patch_w, 3), hsv, dtype=np.uint8)
    patch_bgr = cv2.cvtColor(patch_hsv, cv2.COLOR_HSV2BGR)
    frame[y0:y1, x0:x1] = patch_bgr
    return frame


def make_detection(cx: int, cy: int, color: str = "blue") -> Detection:
    """Return a synthetic Detection for tracker unit tests."""
    half = 20
    pts = np.array([
        [[cx - half, cy - half]],
        [[cx + half, cy - half]],
        [[cx + half, cy + half]],
        [[cx - half, cy + half]],
    ], dtype=np.int32)
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(mask, [pts.reshape(-1, 2)], 255)
    return Detection(
        centroid=(cx, cy),
        bbox=(cx - half, cy - half, half * 2, half * 2),
        contour=pts,
        area=float((half * 2) ** 2),
        color_name=color,
        color_bgr=(255, 0, 0),
        mask=mask,
    )


# ═══════════════════════════════════════════════════════════════════════════════
# 1. Utility functions
# ═══════════════════════════════════════════════════════════════════════════════
class TestUtils(unittest.TestCase):

    def test_euclidean_distance_3_4_5(self):
        self.assertAlmostEqual(euclidean_distance((0, 0), (3, 4)), 5.0)

    def test_euclidean_distance_same_point(self):
        self.assertAlmostEqual(euclidean_distance((7, 3), (7, 3)), 0.0)

    def test_euclidean_distance_horizontal(self):
        self.assertAlmostEqual(euclidean_distance((0, 0), (10, 0)), 10.0)

    def test_euclidean_distance_vertical(self):
        self.assertAlmostEqual(euclidean_distance((0, 0), (0, 5)), 5.0)

    def test_ema_alpha_1_returns_current(self):
        self.assertAlmostEqual(ema_smooth(42.0, 0.0, 1.0), 42.0)

    def test_ema_alpha_0_returns_previous(self):
        self.assertAlmostEqual(ema_smooth(42.0, 7.0, 0.0), 7.0)

    def test_ema_alpha_half(self):
        self.assertAlmostEqual(ema_smooth(10.0, 0.0, 0.5), 5.0)

    def test_ema_weighted(self):
        # 0.4 * 10 + 0.6 * 5 = 7.0
        self.assertAlmostEqual(ema_smooth(10.0, 5.0, 0.4), 7.0)

    def test_ema_convergence(self):
        # Repeated application should converge to the current value when alpha > 0
        val = 0.0
        for _ in range(200):
            val = ema_smooth(100.0, val, 0.3)
        self.assertGreater(val, 99.9)


# ═══════════════════════════════════════════════════════════════════════════════
# 2. Detector
# ═══════════════════════════════════════════════════════════════════════════════
class TestDetector(unittest.TestCase):

    def setUp(self):
        self.det = Detector(DETECTOR_CFG)

    # ── color detection ───────────────────────────────────────────────────────
    def test_detect_blue_blob(self):
        frame = make_blob_frame(cx=300, cy=240, hsv=(115, 220, 220))
        dets = self.det.detect(frame)
        blues = [d for d in dets if d.color_name == "blue"]
        self.assertEqual(len(blues), 1, "Expected exactly 1 blue detection")

    def test_detect_red_blob_low_hue(self):
        # Red at low hue (near 0)
        frame = make_blob_frame(cx=320, cy=240, hsv=(5, 200, 200))
        dets = self.det.detect(frame)
        reds = [d for d in dets if d.color_name == "red"]
        self.assertEqual(len(reds), 1, "Expected exactly 1 red detection (low hue)")

    def test_detect_red_blob_high_hue(self):
        # Red at high hue (near 180, wrap-around range)
        frame = make_blob_frame(cx=320, cy=240, hsv=(175, 200, 200))
        dets = self.det.detect(frame)
        reds = [d for d in dets if d.color_name == "red"]
        self.assertEqual(len(reds), 1, "Expected exactly 1 red detection (high hue)")

    def test_detect_green_blob(self):
        frame = make_blob_frame(cx=300, cy=240, hsv=(60, 200, 200))
        dets = self.det.detect(frame)
        greens = [d for d in dets if d.color_name == "green"]
        self.assertEqual(len(greens), 1, "Expected exactly 1 green detection")

    def test_black_frame_no_detections(self):
        frame = np.zeros((H, W, 3), dtype=np.uint8)
        dets = self.det.detect(frame)
        self.assertEqual(len(dets), 0, "Black frame should yield no detections")

    def test_white_frame_no_detections(self):
        # White in HSV is low saturation → not matched by our high-saturation ranges
        frame = np.full((H, W, 3), 255, dtype=np.uint8)
        dets = self.det.detect(frame)
        self.assertEqual(len(dets), 0, "White frame should yield no detections")

    def test_tiny_blob_below_min_area_ignored(self):
        # 15×15 = 225 px² < min_contour_area 500
        frame = make_blob_frame(cx=300, cy=240, hsv=(115, 220, 220), size=15)
        dets = self.det.detect(frame)
        blues = [d for d in dets if d.color_name == "blue"]
        self.assertEqual(len(blues), 0, "Blob below min area should be ignored")

    # ── centroid accuracy ─────────────────────────────────────────────────────
    def test_centroid_accuracy_blue(self):
        cx_true, cy_true = 300, 240
        frame = make_blob_frame(cx=cx_true, cy=cy_true, hsv=(115, 220, 220))
        dets = self.det.detect(frame)
        blues = [d for d in dets if d.color_name == "blue"]
        self.assertEqual(len(blues), 1)
        cx_det, cy_det = blues[0].centroid
        self.assertAlmostEqual(cx_det, cx_true, delta=5,
                               msg="Centroid X should be within 5 px of truth")
        self.assertAlmostEqual(cy_det, cy_true, delta=5,
                               msg="Centroid Y should be within 5 px of truth")

    def test_centroid_accuracy_green(self):
        cx_true, cy_true = 200, 150
        frame = make_blob_frame(cx=cx_true, cy=cy_true, hsv=(60, 200, 200))
        dets = self.det.detect(frame)
        greens = [d for d in dets if d.color_name == "green"]
        self.assertEqual(len(greens), 1)
        cx_det, cy_det = greens[0].centroid
        self.assertAlmostEqual(cx_det, cx_true, delta=5)
        self.assertAlmostEqual(cy_det, cy_true, delta=5)

    # ── Detection fields ──────────────────────────────────────────────────────
    def test_detection_fields_present(self):
        frame = make_blob_frame(cx=300, cy=240, hsv=(115, 220, 220))
        dets = self.det.detect(frame)
        d = [x for x in dets if x.color_name == "blue"][0]
        self.assertIsInstance(d.centroid, tuple)
        self.assertIsInstance(d.bbox, tuple)
        self.assertIsInstance(d.area, float)
        self.assertIsNotNone(d.contour)
        self.assertEqual(d.mask.shape, (H, W))

    def test_detection_area_positive(self):
        frame = make_blob_frame(cx=300, cy=240, hsv=(115, 220, 220))
        dets = self.det.detect(frame)
        for d in dets:
            self.assertGreater(d.area, 0)

    # ── combined mask ─────────────────────────────────────────────────────────
    def test_get_combined_mask_empty(self):
        combined = Detector.get_combined_mask([], (H, W))
        self.assertEqual(combined.sum(), 0)

    def test_get_combined_mask_nonzero(self):
        frame = make_blob_frame(cx=300, cy=240, hsv=(115, 220, 220))
        dets = self.det.detect(frame)
        combined = Detector.get_combined_mask(dets, (H, W))
        self.assertGreater(combined.sum(), 0)

    # ── two blobs same color ──────────────────────────────────────────────────
    def test_two_blue_blobs_detected(self):
        frame  = make_blob_frame(cx=150, cy=240, hsv=(115, 220, 220))
        frame2 = make_blob_frame(cx=490, cy=240, hsv=(115, 220, 220))
        # Merge the two frames (union of blobs)
        combined = cv2.bitwise_or(frame, frame2)
        dets = self.det.detect(combined)
        blues = [d for d in dets if d.color_name == "blue"]
        self.assertEqual(len(blues), 2, "Should detect 2 separate blue blobs")


# ═══════════════════════════════════════════════════════════════════════════════
# 3. Tracker
# ═══════════════════════════════════════════════════════════════════════════════
class TestTracker(unittest.TestCase):

    def setUp(self):
        self.tracker = ObjectTracker(TRACKER_CFG)

    def _fresh_tracker(self):
        return ObjectTracker(TRACKER_CFG)

    # ── registration ──────────────────────────────────────────────────────────
    def test_new_detection_gets_id(self):
        det = [make_detection(300, 240)]
        tracked = self.tracker.update(det)
        self.assertEqual(len(tracked), 1)
        self.assertEqual(tracked[0].id, 0)

    def test_two_detections_get_different_ids(self):
        dets = [make_detection(100, 200), make_detection(400, 200)]
        tracked = self.tracker.update(dets)
        ids = {obj.id for obj in tracked}
        self.assertEqual(len(ids), 2, "Two detections must get different IDs")

    # ── persistence ───────────────────────────────────────────────────────────
    def test_same_position_keeps_id(self):
        det = [make_detection(300, 240)]
        first  = self.tracker.update(det)
        second = self.tracker.update(det)
        self.assertEqual(first[0].id, second[0].id,
                         "Same position across frames must keep the same ID")

    def test_nearby_position_keeps_id(self):
        t = self._fresh_tracker()
        t.update([make_detection(300, 240)])
        # move by 30 px (< max_distance 80)
        tracked = t.update([make_detection(330, 250)])
        self.assertEqual(tracked[0].id, 0)

    def test_far_jump_registers_new_id(self):
        # Move 200 px — beyond max_distance (80), so tracker treats it as new
        t = self._fresh_tracker()
        t.update([make_detection(100, 240)])
        tracked = t.update([make_detection(350, 240)])
        # Original object disappears; new one registered → could be id 0 (gone) and 1
        ids = {obj.id for obj in tracked}
        # The old object might still be alive with disappeared=1; the new one is a new id
        # The key check: total tracked objects includes a newer id
        self.assertTrue(max(ids) >= 1, "Large jump should register a new ID")

    # ── disappearance ─────────────────────────────────────────────────────────
    def test_disappeared_increments_when_no_detection(self):
        t = self._fresh_tracker()
        t.update([make_detection(300, 240)])
        tracked = t.update([])  # no detections
        self.assertEqual(len(tracked), 1)
        self.assertEqual(tracked[0].disappeared, 1)

    def test_object_removed_after_max_disappeared(self):
        # max_disappeared = 5
        t = self._fresh_tracker()
        t.update([make_detection(300, 240)])
        for _ in range(6):  # one more than threshold
            tracked = t.update([])
        self.assertEqual(len(tracked), 0, "Object should be removed after max_disappeared frames")

    def test_object_revived_before_removal(self):
        t = self._fresh_tracker()
        t.update([make_detection(300, 240)])
        t.update([])  # disappeared = 1
        t.update([])  # disappeared = 2
        tracked = t.update([make_detection(305, 240)])  # re-detected nearby
        self.assertEqual(tracked[0].disappeared, 0)

    # ── smoothing ────────────────────────────────────────────────────────────
    def test_ema_smoothing_applied(self):
        """Centroid should be smoothed, not equal to raw detection centroid."""
        t = self._fresh_tracker()
        t.update([make_detection(300, 240)])
        tracked = t.update([make_detection(340, 280)])
        obj = tracked[0]
        # raw = 340, prev = 300; smooth_x = 0.4*340 + 0.6*300 = 316
        # raw = 280, prev = 240; smooth_y = 0.4*280 + 0.6*240 = 256
        self.assertAlmostEqual(obj.centroid[0], 316.0, delta=1.0)
        self.assertAlmostEqual(obj.centroid[1], 256.0, delta=1.0)

    def test_raw_centroid_matches_detection(self):
        t = self._fresh_tracker()
        t.update([make_detection(300, 240)])
        tracked = t.update([make_detection(340, 280)])
        obj = tracked[0]
        self.assertAlmostEqual(obj.raw_centroid[0], 340.0)
        self.assertAlmostEqual(obj.raw_centroid[1], 280.0)

    # ── trajectory ───────────────────────────────────────────────────────────
    def test_trajectory_grows_with_updates(self):
        t = self._fresh_tracker()
        t.update([make_detection(300, 240)])
        self.assertEqual(len(t._objects[0].trajectory), 1)
        t.update([make_detection(305, 240)])
        self.assertEqual(len(t._objects[0].trajectory), 2)

    def test_trajectory_maxlen(self):
        t = self._fresh_tracker()
        t.update([make_detection(300, 240)])
        for i in range(50):  # trajectory_length is 20
            t.update([make_detection(300 + i, 240)])
        traj = t._objects[0].trajectory
        self.assertLessEqual(len(traj), TRACKER_CFG["trajectory_length"])


# ═══════════════════════════════════════════════════════════════════════════════
# 4. Coordinate Projection Math (no TF2 / ROS)
#    Tests the pinhole projection formula used in CoordinateConverter.pixel_to_base
# ═══════════════════════════════════════════════════════════════════════════════
class TestCoordinateProjectionMath(unittest.TestCase):
    """
    Validate the pixel → camera-frame projection math independently.
    The full CoordinateConverter also does a TF2 transform, which requires ROS.
    We test only the projection formula here.
    """

    FX    = 554.3827128226441
    FY    = 554.3827128226441
    CX    = 320.5
    CY    = 240.5
    FIXED_Z = 0.7

    def _project(self, u: float, v: float):
        x = (u - self.CX) * self.FIXED_Z / self.FX
        y = (v - self.CY) * self.FIXED_Z / self.FY
        return x, y, self.FIXED_Z

    def test_center_pixel_maps_to_optical_axis(self):
        x, y, z = self._project(self.CX, self.CY)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(z, self.FIXED_Z, places=9)

    def test_unit_offset_right(self):
        # u = cx + fx → x_cam = fx * z / fx = z
        x, y, z = self._project(self.CX + self.FX, self.CY)
        self.assertAlmostEqual(x, self.FIXED_Z, places=6)
        self.assertAlmostEqual(y, 0.0, places=9)

    def test_unit_offset_down(self):
        x, y, z = self._project(self.CX, self.CY + self.FY)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, self.FIXED_Z, places=6)

    def test_left_right_symmetry(self):
        x_right, _, _ = self._project(self.CX + 100, self.CY)
        x_left,  _, _ = self._project(self.CX - 100, self.CY)
        self.assertAlmostEqual(x_right, -x_left, places=9)

    def test_top_bottom_symmetry(self):
        _, y_below, _ = self._project(self.CX, self.CY + 100)
        _, y_above, _ = self._project(self.CX, self.CY - 100)
        self.assertAlmostEqual(y_below, -y_above, places=9)

    def test_scale_with_fixed_z(self):
        # Doubling z doubles the projected X offset
        u = self.CX + 100
        x1, _, _ = self._project(u, self.CY)
        # Manually project with 2x z
        x2 = (u - self.CX) * (self.FIXED_Z * 2) / self.FX
        self.assertAlmostEqual(x2, 2 * x1, places=9)

    def test_known_value(self):
        # u=420.5 (100 px right of CX), v=CY, z=0.7
        # x = 100 * 0.7 / 554.38 ≈ 0.1263 m
        x, y, _ = self._project(420.5, self.CY)
        expected_x = 100 * self.FIXED_Z / self.FX
        self.assertAlmostEqual(x, expected_x, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)


# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    unittest.main(verbosity=2)
