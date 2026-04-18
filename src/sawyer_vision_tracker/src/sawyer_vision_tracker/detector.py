"""Object detection via HSV color segmentation and ArUco markers."""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class Detection:
    """A single detected object in a frame."""

    centroid:     tuple[int, int]           # (cx, cy) in pixels
    bbox:         tuple[int, int, int, int] # (x, y, w, h)
    contour:      np.ndarray
    area:         float
    color_name:   str
    color_bgr:    tuple[int, int, int]
    mask:         np.ndarray                # binary mask for this object
    # Shape quality metrics — populated by detect_blue_cube(), 1.0 default otherwise
    solidity:     float = 1.0              # area / convex-hull area  (0–1, higher = more convex)
    aspect_ratio: float = 1.0              # bbox w/h  (1.0 = square)


class Detector:
    """Detect objects using HSV color segmentation and/or ArUco markers."""

    def __init__(self, config: dict) -> None:
        # ── generic HSV segmentation ─────────────────────────────────────────
        self._hsv_ranges = config.get("hsv_ranges", [])
        self._min_area   = config.get("min_contour_area", 500)
        self._blur_k     = config.get("gaussian_blur", 5)
        morph_k = config.get("morph_kernel_size", 5)
        self._morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morph_k, morph_k)
        )

        # ── blue-cube specific config ─────────────────────────────────────────
        bc = config.get("blue_cube", {})
        self._bc_lower = np.array(
            bc.get("hsv_lower", [95, 100, 60]), dtype=np.uint8
        )
        self._bc_upper = np.array(
            bc.get("hsv_upper", [135, 255, 255]), dtype=np.uint8
        )
        self._bc_min_area       = bc.get("min_area",            1000)
        self._bc_max_area       = bc.get("max_area",           40000)
        self._bc_min_solidity   = bc.get("min_solidity",         0.80)
        self._bc_max_aspect_err = bc.get("max_aspect_ratio_err", 0.40)

        bc_morph_k = bc.get("morph_kernel_size", morph_k)
        self._bc_morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (bc_morph_k, bc_morph_k)
        )
        self._bc_blur_k = bc.get("gaussian_blur", self._blur_k)

        # ── ArUco setup ───────────────────────────────────────────────────────
        aruco_cfg = config.get("aruco", {})
        self._aruco_enabled = aruco_cfg.get("enabled", False)
        if self._aruco_enabled:
            dict_name = aruco_cfg.get("dictionary", "DICT_4X4_50")
            dict_id   = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)
            self._aruco_dict     = cv2.aruco.getPredefinedDictionary(dict_id)
            self._aruco_params   = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(
                self._aruco_dict, self._aruco_params
            )

    # ── blue cube detector ────────────────────────────────────────────────────

    def detect_blue_cube(self, frame: np.ndarray) -> Detection | None:
        """
        Detect a single blue cube on a flat surface.

        On top of HSV colour segmentation, two shape filters reject noise:

          solidity     = contour_area / convex_hull_area
                         A cube's top face is convex → solidity close to 1.
                         Irregular blobs (reflections, shadows) score low.

          aspect_ratio = bbox_width / bbox_height
                         A cube viewed from above is nearly square → close to 1.
                         Long thin shapes are rejected.

        Returns the single largest contour that passes all filters, or None.
        """
        h, w = frame.shape[:2]
        blurred = cv2.GaussianBlur(frame, (self._bc_blur_k, self._bc_blur_k), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self._bc_lower, self._bc_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._bc_morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._bc_morph_kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best: Detection | None = None
        best_area = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self._bc_min_area or area > self._bc_max_area:
                continue

            # ── solidity filter ───────────────────────────────────────────────
            hull      = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0:
                continue
            solidity = area / hull_area
            if solidity < self._bc_min_solidity:
                continue

            # ── aspect-ratio filter ───────────────────────────────────────────
            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect = bw / bh if bh > 0 else 0.0
            if abs(aspect - 1.0) > self._bc_max_aspect_err:
                continue

            # ── keep largest passing candidate ────────────────────────────────
            if area <= best_area:
                continue
            best_area = area

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx_px = int(M["m10"] / M["m00"])
            cy_px = int(M["m01"] / M["m00"])

            obj_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(obj_mask, [cnt], -1, 255, cv2.FILLED)

            best = Detection(
                centroid=(cx_px, cy_px),
                bbox=(x, y, bw, bh),
                contour=cnt,
                area=area,
                color_name="blue_cube",
                color_bgr=(0, 220, 255),    # yellow annotation on blue object
                mask=obj_mask,
                solidity=solidity,
                aspect_ratio=aspect,
            )

        return best

    def get_blue_mask(self, frame: np.ndarray) -> np.ndarray:
        """Return the binary HSV mask used by detect_blue_cube (for visualisation)."""
        blurred = cv2.GaussianBlur(frame, (self._bc_blur_k, self._bc_blur_k), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask    = cv2.inRange(hsv, self._bc_lower, self._bc_upper)
        mask    = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._bc_morph_kernel)
        mask    = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._bc_morph_kernel)
        return mask

    # ── generic multi-colour detector (unchanged) ─────────────────────────────

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Detect objects via HSV colour segmentation (multi-colour)."""
        h, w = frame.shape[:2]
        blurred = cv2.GaussianBlur(frame, (self._blur_k, self._blur_k), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        detections: list[Detection] = []

        for color_cfg in self._hsv_ranges:
            name      = color_cfg["name"]
            color_bgr = tuple(color_cfg["color_bgr"])

            lower = np.array(color_cfg["lower"], dtype=np.uint8)
            upper = np.array(color_cfg["upper"], dtype=np.uint8)
            mask  = cv2.inRange(hsv, lower, upper)

            if "lower2" in color_cfg and "upper2" in color_cfg:
                mask2 = cv2.inRange(
                    hsv,
                    np.array(color_cfg["lower2"], dtype=np.uint8),
                    np.array(color_cfg["upper2"], dtype=np.uint8),
                )
                mask = cv2.bitwise_or(mask, mask2)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._morph_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph_kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self._min_area:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                x, y, bw, bh = cv2.boundingRect(cnt)

                obj_mask = np.zeros((h, w), dtype=np.uint8)
                cv2.drawContours(obj_mask, [cnt], -1, 255, cv2.FILLED)

                detections.append(
                    Detection(
                        centroid=(cx, cy),
                        bbox=(x, y, bw, bh),
                        contour=cnt,
                        area=area,
                        color_name=name,
                        color_bgr=color_bgr,
                        mask=obj_mask,
                    )
                )

        return detections

    def detect_aruco(self, frame: np.ndarray) -> list[Detection]:
        """Detect ArUco markers and return as Detection objects."""
        if not self._aruco_enabled:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)

        if ids is None:
            return []

        h, w = frame.shape[:2]
        detections: list[Detection] = []

        for i, marker_corners in enumerate(corners):
            pts       = marker_corners[0].astype(np.int32)
            marker_id = int(ids[i][0])

            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))

            x, y, bw, bh = cv2.boundingRect(pts)
            area         = float(cv2.contourArea(pts))

            obj_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(obj_mask, [pts], 255)

            detections.append(
                Detection(
                    centroid=(cx, cy),
                    bbox=(x, y, bw, bh),
                    contour=pts.reshape(-1, 1, 2),
                    area=area,
                    color_name=f"aruco_{marker_id}",
                    color_bgr=(0, 255, 255),
                    mask=obj_mask,
                )
            )

        return detections

    @staticmethod
    def get_combined_mask(
        detections: list[Detection], shape: tuple[int, int]
    ) -> np.ndarray:
        """OR all detection masks into a single binary mask."""
        combined = np.zeros(shape, dtype=np.uint8)
        for det in detections:
            combined = cv2.bitwise_or(combined, det.mask)
        return combined
