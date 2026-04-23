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
        self._bc_min_extent     = bc.get("min_extent",           0.60)
        # Reject blue blobs outside the expected tabletop image region.
        self._bc_min_center_y_frac = bc.get("min_center_y_frac", 0.45)
        self._bc_max_center_y_frac = bc.get("max_center_y_frac", 1.00)
        # Candidate scoring (higher is better). Distance term stabilizes lock.
        self._bc_score_area_w = bc.get("score_area_weight", 1.0)
        self._bc_score_shape_w = bc.get("score_shape_weight", 1.0)
        self._bc_score_dist_w = bc.get("score_distance_weight", 1.5)
        self._bc_score_dist_norm_px = bc.get("score_distance_norm_px", 120.0)
        self._bc_enable_fallback = bc.get("enable_fallback", True)
        self._bc_fallback_min_area_ratio = bc.get("fallback_min_area_ratio", 0.90)
        self._bc_fallback_max_jump_ratio = bc.get("fallback_max_jump_ratio", 1.20)
        self._bc_last_centroid: tuple[float, float] | None = None
        self._bc_last_debug: dict = {}

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
        best_score = float("-inf")
        fallback: Detection | None = None
        fallback_score = float("-inf")
        mask_px = int(mask.sum() / 255)
        roi_candidate_count = 0

        def build_detection(
            cnt: np.ndarray,
            area: float,
            x: int,
            y: int,
            bw: int,
            bh: int,
            cx_px: int,
            cy_px: int,
            solidity: float,
            aspect: float,
        ) -> Detection:
            obj_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(obj_mask, [cnt], -1, 255, cv2.FILLED)
            return Detection(
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

        # Keep candidate areas close to expected cube area; this avoids tiny noise blobs.
        soft_min_area = max(150.0, float(self._bc_min_area) * 0.85)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < soft_min_area or area > self._bc_max_area:
                continue

            # ── shape metrics ─────────────────────────────────────────────────
            hull      = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0:
                continue
            solidity = area / hull_area

            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect = bw / bh if bh > 0 else 0.0

            bbox_area = float(bw * bh)
            if bbox_area <= 0:
                continue
            extent = area / bbox_area

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx_px = int(M["m10"] / M["m00"])
            cy_px = int(M["m01"] / M["m00"])

            cy_frac = cy_px / float(h)
            if (
                cy_frac < self._bc_min_center_y_frac
                or cy_frac > self._bc_max_center_y_frac
            ):
                continue
            roi_candidate_count += 1

            det_obj = build_detection(
                cnt, area, x, y, bw, bh, cx_px, cy_px, solidity, aspect
            )

            area_n = min(area / max(float(self._bc_min_area), 1.0), 2.0)
            aspect_n = max(0.0, 1.0 - abs(aspect - 1.0))
            shape_n = (solidity + extent + aspect_n) / 3.0
            dist_n = 0.0
            if self._bc_last_centroid is not None:
                dx = cx_px - self._bc_last_centroid[0]
                dy = cy_px - self._bc_last_centroid[1]
                dist_px = float(np.hypot(dx, dy))
                dist_n = min(dist_px / max(self._bc_score_dist_norm_px, 1.0), 2.0)
            score = (
                self._bc_score_area_w * area_n
                + self._bc_score_shape_w * shape_n
                - self._bc_score_dist_w * dist_n
            )

            strict_ok = (
                area >= self._bc_min_area
                and solidity >= self._bc_min_solidity
                and abs(aspect - 1.0) <= self._bc_max_aspect_err
                and extent >= self._bc_min_extent
            )
            if strict_ok and score > best_score:
                best_score = score
                best = det_obj

            # Keep a softer fallback candidate in case strict checks reject all.
            # This helps avoid "no tracking" when perspective/lighting distorts shape.
            fallback_ok = (
                self._bc_enable_fallback
                and area >= max(150.0, self._bc_min_area * self._bc_fallback_min_area_ratio)
                and solidity >= max(0.55, self._bc_min_solidity - 0.10)
                and abs(aspect - 1.0) <= (self._bc_max_aspect_err + 0.25)
                and extent >= max(0.30, self._bc_min_extent - 0.10)
                and (
                    self._bc_last_centroid is None
                    or dist_n <= self._bc_fallback_max_jump_ratio
                )
            )
            if fallback_ok and score > fallback_score:
                fallback_score = score
                fallback = det_obj

        chosen = best if best is not None else fallback
        if chosen is not None:
            self._bc_last_centroid = (
                float(chosen.centroid[0]),
                float(chosen.centroid[1]),
            )
            chosen_score = best_score if best is not None else fallback_score
        else:
            chosen_score = float("-inf")

        self._bc_last_debug = {
            "mask_px": mask_px,
            "contours_total": len(contours),
            "roi_candidates": roi_candidate_count,
            "chosen": chosen is not None,
            "chosen_score": chosen_score,
            "chosen_centroid": chosen.centroid if chosen is not None else None,
            "chosen_area": float(chosen.area) if chosen is not None else 0.0,
        }
        return chosen

    def get_last_blue_debug(self) -> dict:
        """Return debug info from the last detect_blue_cube call."""
        return dict(self._bc_last_debug)

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
