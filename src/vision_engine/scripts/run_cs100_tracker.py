#!/usr/bin/env python3
"""
scripts/run_cs100_tracker.py — CS-100 3D Tracker with Table Depth Calibration

Three-phase workflow:
  Phase 0: TABLE CALIBRATION — determine camera-to-table depth
  Phase 1: RECORDING — capture full 6DOF pose at ~120Hz
  Phase 2: PLAYBACK — multi-view timeline scrubber

Display layout:
  ┌────────────────────────┬─────────────────┐
  │   TOP-DOWN VIEW (XY)   │  SIDE VIEW (XZ) │
  │   L-shape + trail      │  Height + tilt  │
  ├────────────────────────┴─────────────────┤
  │  INFO BAR: Depth | Height | Tilt | RPY   │
  ├──────────────────────────────────────────┤
  │  TIMELINE SCRUBBER                       │
  └──────────────────────────────────────────┘

Controls (Calibration)
----------------------
    Enter       Start calibrating (CS-100 must be flat on table)
    Q/Esc       Skip calibration

Controls (Recording)
--------------------
    Space       Pause/resume recording
    Q/Esc       Stop recording early → enter playback

Controls (Playback)
-------------------
    Click/drag  Scrub on the timeline bar
    Left/Right  Step forward/backward one frame
    Up/Down     Skip 10 frames forward/backward
    Home(A)/End(E)  Jump to start/end
    Space       Play/pause auto-playback
    R           Re-record (restart)
    S           Save current frame to output/
    Q/Esc       Quit

Usage
-----
    cd vision_engine
    python scripts/run_cs100_tracker.py                          # Full workflow
    python scripts/run_cs100_tracker.py --skip-calibration       # Skip table cal
    python scripts/run_cs100_tracker.py --no-optitrack --skip-calibration  # Demo
    python scripts/run_cs100_tracker.py --duration 10            # 10s recording
"""

import sys
import os
import time
import argparse
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from utils import load_config
from cv.optitrack_client import OptiTrackClient, RigidBodyState
from cv.cs100_model import CS100Geometry
from cv.depth_estimator import FloorDepthEstimator
from cv.transforms import quaternion_to_euler


# ──────────────────────────────────────────────────────────────────────────────
# Data Structures
# ──────────────────────────────────────────────────────────────────────────────

class RecordedFrame:
    """A single recorded frame with full 3D pose and derived quantities."""
    __slots__ = (
        "timestamp", "position", "quaternion", "marker_positions",
        "surface_normal", "height_above_table", "tilt_angle_deg",
    )

    def __init__(self, timestamp, position, quaternion, marker_positions,
                 surface_normal=None, height_above_table=0.0,
                 tilt_angle_deg=0.0):
        self.timestamp = timestamp
        self.position = position
        self.quaternion = quaternion
        self.marker_positions = marker_positions
        self.surface_normal = surface_normal
        self.height_above_table = height_above_table
        self.tilt_angle_deg = tilt_angle_deg


class TableCalibration:
    """Stores table plane calibration results."""

    def __init__(self):
        self.table_point = np.array([0.0, 0.0, 0.0])
        self.table_normal = np.array([0.0, 0.0, 1.0])
        self.table_z = 0.0
        self.camera_to_table_depth = 0.0
        self.is_calibrated = False

    def height_of(self, point):
        """Signed height of a point above the table plane."""
        return float(np.dot(point - self.table_point, self.table_normal))

    def tilt_of(self, surface_normal):
        """Angle (degrees) between a surface normal and the table normal."""
        cos_a = np.clip(abs(np.dot(surface_normal, self.table_normal)), -1, 1)
        return float(np.degrees(np.arccos(cos_a)))


def compute_frame_3d(cs100, frame, table_cal):
    """Fill in surface_normal, height, and tilt for a frame."""
    axes = cs100.get_l_frame_axes(frame.position, frame.quaternion)
    frame.surface_normal = axes["z_axis"]
    if table_cal.is_calibrated:
        frame.height_above_table = table_cal.height_of(frame.position)
        frame.tilt_angle_deg = table_cal.tilt_of(frame.surface_normal)
    else:
        frame.height_above_table = frame.position[2]
        frame.tilt_angle_deg = 0.0


# ──────────────────────────────────────────────────────────────────────────────
# Coordinate Mappers
# ──────────────────────────────────────────────────────────────────────────────

class TopDownMapper:
    """Maps 3D world (X, Y) to pixel coordinates in a canvas region."""

    def __init__(self, region_w, region_h, offset_x=0, offset_y=0,
                 margin=0.06):
        self.region_w = region_w
        self.region_h = region_h
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.margin = margin
        self.x_min = -0.3
        self.x_max = 0.3
        self.y_min = -0.3
        self.y_max = 0.3

    def fit_to_data(self, frames, padding=0.05):
        if not frames:
            return
        pts = np.array([f.position[:2] for f in frames])
        # Include marker positions for better bounds
        for f in frames:
            if f.marker_positions is not None:
                pts = np.vstack([pts, f.marker_positions[:, :2]])
        x_min, y_min = pts.min(axis=0)
        x_max, y_max = pts.max(axis=0)
        max_range = max(x_max - x_min, y_max - y_min, 0.1) + padding * 2
        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2
        self.x_min = cx - max_range / 2
        self.x_max = cx + max_range / 2
        self.y_min = cy - max_range / 2
        self.y_max = cy + max_range / 2

    def to_pixel(self, x, y):
        uw = self.region_w * (1 - 2 * self.margin)
        uh = self.region_h * (1 - 2 * self.margin)
        px = int(self.margin * self.region_w +
                 (x - self.x_min) / max(self.x_max - self.x_min, 1e-6) * uw)
        py = int(self.margin * self.region_h +
                 (1.0 - (y - self.y_min) / max(self.y_max - self.y_min, 1e-6)) * uh)
        return (px + self.offset_x, py + self.offset_y)


class SideViewMapper:
    """Maps world (X, Z) to pixel coordinates for the side view panel."""

    def __init__(self, region_w, region_h, offset_x, offset_y, margin=0.08):
        self.region_w = region_w
        self.region_h = region_h
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.margin = margin
        self.x_min = -0.3
        self.x_max = 0.3
        self.z_min = -0.05  # Slightly below table
        self.z_max = 0.4    # Up to ~40cm above table

    def fit_to_data(self, frames, table_z=0.0, padding=0.05):
        if not frames:
            return
        xs = [f.position[0] for f in frames]
        zs = [f.position[2] for f in frames]
        # Include marker Z positions
        for f in frames:
            if f.marker_positions is not None:
                for m in f.marker_positions:
                    zs.append(m[2])
                    xs.append(m[0])

        x_min, x_max = min(xs), max(xs)
        z_min, z_max = min(zs), max(zs)

        # Ensure table is visible
        z_min = min(z_min, table_z - 0.02)
        z_max = max(z_max, table_z + 0.1)

        x_range = max(x_max - x_min, 0.1) + padding * 2
        z_range = max(z_max - z_min, 0.1) + padding * 2
        cx = (x_min + x_max) / 2
        self.x_min = cx - x_range / 2
        self.x_max = cx + x_range / 2
        self.z_min = z_min - padding
        self.z_max = z_min + z_range  # Don't center Z — keep table at bottom

    def to_pixel(self, x, z):
        uw = self.region_w * (1 - 2 * self.margin)
        uh = self.region_h * (1 - 2 * self.margin)
        px = int(self.margin * self.region_w +
                 (x - self.x_min) / max(self.x_max - self.x_min, 1e-6) * uw)
        # Z increases upward, pixel Y increases downward
        py = int(self.margin * self.region_h +
                 (1.0 - (z - self.z_min) / max(self.z_max - self.z_min, 1e-6)) * uh)
        return (px + self.offset_x, py + self.offset_y)


# ──────────────────────────────────────────────────────────────────────────────
# Colors (BGR)
# ──────────────────────────────────────────────────────────────────────────────

COL_CORNER = (0, 255, 255)      # Yellow
COL_SHORT  = (0, 140, 255)      # Orange
COL_LONG   = (0, 255, 0)        # Green
COL_LINE   = (180, 180, 180)    # Light gray
COL_HUD    = (200, 200, 200)    # Text
COL_GRID   = (40, 40, 40)       # Grid
COL_BAR_BG = (50, 50, 50)       # Timeline background
COL_BAR_FG = (0, 180, 0)        # Timeline filled
COL_CURSOR = (0, 220, 255)      # Timeline cursor
COL_TABLE  = (140, 100, 60)     # Table surface line (brownish)
COL_CAMERA = (255, 200, 0)      # Camera indicator (cyan)
COL_NORMAL = (255, 100, 255)    # Surface normal arrow (magenta)
COL_DIVIDER = (60, 60, 60)      # Panel dividers

MARKER_COLORS = [COL_CORNER, COL_SHORT, COL_LONG]
MARKER_LABELS = ["C", "8cm", "10cm"]


def tilt_color(tilt_deg):
    """Return BGR color based on tilt angle: green→yellow→red."""
    if tilt_deg < 10.0:
        t = tilt_deg / 10.0
        # Green to yellow
        return (0, 255, int(255 * t))
    elif tilt_deg < 30.0:
        t = (tilt_deg - 10.0) / 20.0
        # Yellow to red
        return (0, int(255 * (1 - t)), 255)
    else:
        return (0, 0, 255)


# ──────────────────────────────────────────────────────────────────────────────
# Layout Constants
# ──────────────────────────────────────────────────────────────────────────────

TIMELINE_H = 50
INFO_BAR_H = 55
SIDE_FRAC = 0.35  # Right panel fraction of width


# ──────────────────────────────────────────────────────────────────────────────
# Drawing Functions — Top-Down View
# ──────────────────────────────────────────────────────────────────────────────

def draw_grid_topdown(canvas, mapper, scene_h):
    """Draw background grid lines in the top-down view region."""
    x_range = mapper.x_max - mapper.x_min
    step = 0.05 if x_range < 0.5 else 0.1

    gx = mapper.x_min
    while gx <= mapper.x_max:
        px, _ = mapper.to_pixel(gx, 0)
        py_top = mapper.offset_y
        py_bot = mapper.offset_y + mapper.region_h
        if mapper.offset_x <= px <= mapper.offset_x + mapper.region_w:
            cv2.line(canvas, (px, py_top), (px, min(py_bot, scene_h)),
                     COL_GRID, 1)
        gx += step

    gy = mapper.y_min
    while gy <= mapper.y_max:
        _, py = mapper.to_pixel(0, gy)
        if mapper.offset_y <= py < scene_h:
            cv2.line(canvas, (mapper.offset_x, py),
                     (mapper.offset_x + mapper.region_w, py), COL_GRID, 1)
        gy += step

    # Origin crosshair
    ox, oy = mapper.to_pixel(0, 0)
    if mapper.offset_y <= oy < scene_h:
        cv2.line(canvas, (max(ox - 12, mapper.offset_x), oy),
                 (min(ox + 12, mapper.offset_x + mapper.region_w), oy),
                 (60, 60, 60), 1)
        cv2.line(canvas, (ox, max(oy - 12, mapper.offset_y)),
                 (ox, min(oy + 12, scene_h)), (60, 60, 60), 1)


def draw_trail_topdown(canvas, mapper, frames, end_idx, scene_h):
    """Draw fading trail with height-based color in the top-down view."""
    if end_idx < 1:
        return
    n = min(end_idx + 1, len(frames))

    for i in range(1, n):
        p0 = mapper.to_pixel(frames[i - 1].position[0],
                              frames[i - 1].position[1])
        p1 = mapper.to_pixel(frames[i].position[0], frames[i].position[1])
        if p0[1] >= scene_h or p1[1] >= scene_h:
            continue

        alpha = int(60 + 195 * (i / n))
        # Tint by height: higher = more blue
        h = abs(frames[i].height_above_table)
        blue = min(int(h * 2000), 200)
        color = (alpha + blue, alpha, max(alpha - blue, 40))
        thickness = 1 if i < n * 0.7 else 2
        cv2.line(canvas, p0, p1, color, thickness)


def draw_lshape_topdown(canvas, mapper, frame, scene_h):
    """Draw the L-shape with filled triangle colored by tilt."""
    if frame is None or frame.marker_positions is None:
        return

    mpx = [mapper.to_pixel(m[0], m[1]) for m in frame.marker_positions]

    # Check bounds
    for px, py in mpx:
        if py >= scene_h:
            return

    # Filled triangle (semi-transparent via overlay)
    tri_pts = np.array(mpx, dtype=np.int32)
    overlay = canvas.copy()
    fill_col = tilt_color(frame.tilt_angle_deg)
    cv2.fillPoly(overlay, [tri_pts], fill_col)
    cv2.addWeighted(overlay, 0.25, canvas, 0.75, 0, canvas)

    # Triangle edges
    cv2.line(canvas, mpx[0], mpx[1], COL_LINE, 1, cv2.LINE_AA)
    cv2.line(canvas, mpx[0], mpx[2], COL_LINE, 1, cv2.LINE_AA)
    cv2.line(canvas, mpx[1], mpx[2], (100, 100, 100), 1, cv2.LINE_AA)

    # Marker dots
    for i, (px, py) in enumerate(mpx):
        cv2.circle(canvas, (px, py), 7, MARKER_COLORS[i], -1, cv2.LINE_AA)
        cv2.circle(canvas, (px, py), 7, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, MARKER_LABELS[i], (px + 10, py - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, MARKER_COLORS[i], 1,
                    cv2.LINE_AA)

    # Surface normal projected onto XY (short arrow from centroid)
    if frame.surface_normal is not None:
        cx, cy = mapper.to_pixel(frame.position[0], frame.position[1])
        # Normal projection onto XY plane
        nx, ny = frame.surface_normal[0], frame.surface_normal[1]
        scale = 25  # pixels
        ex = int(cx + nx * scale)
        ey = int(cy - ny * scale)  # Y flipped
        cv2.arrowedLine(canvas, (cx, cy), (ex, ey), COL_NORMAL, 2,
                        cv2.LINE_AA, tipLength=0.3)

    # Center dot
    cpx, cpy = mapper.to_pixel(frame.position[0], frame.position[1])
    cv2.circle(canvas, (cpx, cpy), 3, (255, 255, 255), -1, cv2.LINE_AA)


# ──────────────────────────────────────────────────────────────────────────────
# Drawing Functions — Side View
# ──────────────────────────────────────────────────────────────────────────────

def draw_grid_side(canvas, side_mapper, scene_h, table_cal):
    """Draw background grid and table reference line in side view."""
    ox = side_mapper.offset_x
    rw = side_mapper.region_w

    # Vertical divider
    cv2.line(canvas, (ox, 0), (ox, scene_h), COL_DIVIDER, 1)

    # Grid
    x_range = side_mapper.x_max - side_mapper.x_min
    step = 0.05 if x_range < 0.5 else 0.1

    gx = side_mapper.x_min
    while gx <= side_mapper.x_max:
        px, _ = side_mapper.to_pixel(gx, 0)
        if ox <= px <= ox + rw:
            cv2.line(canvas, (px, side_mapper.offset_y), (px, scene_h),
                     COL_GRID, 1)
        gx += step

    z_range = side_mapper.z_max - side_mapper.z_min
    z_step = 0.05 if z_range < 0.4 else 0.1
    gz = side_mapper.z_min
    while gz <= side_mapper.z_max:
        _, py = side_mapper.to_pixel(0, gz)
        if side_mapper.offset_y <= py < scene_h:
            cv2.line(canvas, (ox, py), (ox + rw, py), COL_GRID, 1)
        gz += z_step

    # Table surface line (thick, dashed)
    tz = table_cal.table_z if table_cal.is_calibrated else 0.0
    _, tpy = side_mapper.to_pixel(0, tz)
    if side_mapper.offset_y <= tpy < scene_h:
        # Draw dashed line
        dash_len = 8
        x = ox
        while x < ox + rw:
            x_end = min(x + dash_len, ox + rw)
            cv2.line(canvas, (x, tpy), (x_end, tpy), COL_TABLE, 2)
            x += dash_len * 2
        cv2.putText(canvas, "TABLE", (ox + 5, tpy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, COL_TABLE, 1)

    # Camera indicator at origin (top of view)
    _, cam_py = side_mapper.to_pixel(0, 0)
    cam_px = ox + rw // 2
    if side_mapper.offset_y <= cam_py < scene_h:
        cv2.drawMarker(canvas, (cam_px, cam_py), COL_CAMERA,
                       cv2.MARKER_DIAMOND, 10, 2)
        cv2.putText(canvas, "CAM", (cam_px + 8, cam_py + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, COL_CAMERA, 1)

    # Panel label
    cv2.putText(canvas, "SIDE (XZ)", (ox + 5, side_mapper.offset_y + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)


def draw_lshape_side(canvas, side_mapper, frame, scene_h, table_cal):
    """Draw the L-shape projected onto XZ plane in the side view."""
    if frame is None or frame.marker_positions is None:
        return

    ox = side_mapper.offset_x
    rw = side_mapper.region_w

    # Project markers onto XZ
    mpx = [side_mapper.to_pixel(m[0], m[2]) for m in frame.marker_positions]

    # Check bounds
    for px, py in mpx:
        if py >= scene_h or px < ox or px > ox + rw:
            return

    # Filled triangle
    tri_pts = np.array(mpx, dtype=np.int32)
    overlay = canvas.copy()
    fill_col = tilt_color(frame.tilt_angle_deg)
    cv2.fillPoly(overlay, [tri_pts], fill_col)
    cv2.addWeighted(overlay, 0.3, canvas, 0.7, 0, canvas)

    # Triangle edges
    cv2.line(canvas, mpx[0], mpx[1], COL_LINE, 1, cv2.LINE_AA)
    cv2.line(canvas, mpx[0], mpx[2], COL_LINE, 1, cv2.LINE_AA)
    cv2.line(canvas, mpx[1], mpx[2], (100, 100, 100), 1, cv2.LINE_AA)

    # Marker dots
    for i, (px, py) in enumerate(mpx):
        cv2.circle(canvas, (px, py), 5, MARKER_COLORS[i], -1, cv2.LINE_AA)
        cv2.circle(canvas, (px, py), 5, (255, 255, 255), 1, cv2.LINE_AA)

    # Surface normal arrow (in XZ plane)
    if frame.surface_normal is not None:
        cpx, cpz = side_mapper.to_pixel(frame.position[0], frame.position[2])
        nx, nz = frame.surface_normal[0], frame.surface_normal[2]
        scale = 25
        ex = int(cpx + nx * scale)
        ez = int(cpz - nz * scale)  # Z flipped (up in world = up in pixel)
        cv2.arrowedLine(canvas, (cpx, cpz), (ex, ez), COL_NORMAL, 2,
                        cv2.LINE_AA, tipLength=0.3)

    # Height measurement line (from centroid down to table)
    tz = table_cal.table_z if table_cal.is_calibrated else 0.0
    cpx, cpz = side_mapper.to_pixel(frame.position[0], frame.position[2])
    _, tpz = side_mapper.to_pixel(0, tz)
    if abs(cpz - tpz) > 5:  # Only if visually meaningful
        cv2.line(canvas, (cpx, cpz), (cpx, tpz), (100, 180, 255), 1,
                 cv2.LINE_AA)
        mid_y = (cpz + tpz) // 2
        h_cm = frame.height_above_table * 100
        cv2.putText(canvas, f"{h_cm:.1f}cm", (cpx + 5, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 180, 255), 1)

    # Center dot
    cv2.circle(canvas, (cpx, cpz), 3, (255, 255, 255), -1, cv2.LINE_AA)


def draw_trail_side(canvas, side_mapper, frames, end_idx, scene_h):
    """Draw trail in the side view (XZ projection)."""
    if end_idx < 1:
        return
    n = min(end_idx + 1, len(frames))
    ox = side_mapper.offset_x
    rw = side_mapper.region_w

    for i in range(1, n):
        p0 = side_mapper.to_pixel(frames[i - 1].position[0],
                                   frames[i - 1].position[2])
        p1 = side_mapper.to_pixel(frames[i].position[0],
                                   frames[i].position[2])
        if (p0[1] >= scene_h or p1[1] >= scene_h or
                p0[0] < ox or p1[0] > ox + rw):
            continue
        alpha = int(40 + 160 * (i / n))
        cv2.line(canvas, p0, p1, (alpha, alpha, alpha), 1)


# ──────────────────────────────────────────────────────────────────────────────
# Drawing Functions — Info Bar & Timeline
# ──────────────────────────────────────────────────────────────────────────────

def draw_info_bar(canvas, frame, table_cal, scene_h, w):
    """Draw the info bar between the scene views and the timeline."""
    bar_top = scene_h
    bar_bot = scene_h + INFO_BAR_H

    # Background
    cv2.rectangle(canvas, (0, bar_top), (w, bar_bot), (25, 25, 30), -1)
    cv2.line(canvas, (0, bar_top), (w, bar_top), COL_DIVIDER, 1)
    cv2.line(canvas, (0, bar_bot), (w, bar_bot), COL_DIVIDER, 1)

    y1 = bar_top + 18
    y2 = bar_top + 40

    # Camera → Table depth (always shown)
    if table_cal.is_calibrated:
        depth_m = table_cal.camera_to_table_depth
        cv2.putText(canvas, f"Cam->Table: {depth_m:.3f}m ({depth_m*100:.1f}cm)",
                    (10, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.42, COL_CAMERA, 1)
    else:
        cv2.putText(canvas, "Cam->Table: -- (not calibrated)",
                    (10, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (100, 100, 100), 1)

    if frame is not None:
        # Height above table
        h_cm = frame.height_above_table * 100
        h_col = (100, 200, 255) if abs(h_cm) < 2 else (100, 255, 200)
        cv2.putText(canvas, f"Height: {h_cm:+.1f}cm",
                    (280, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.42, h_col, 1)

        # Tilt angle
        tilt = frame.tilt_angle_deg
        t_col = tilt_color(tilt)
        cv2.putText(canvas, f"Tilt: {tilt:.1f}deg",
                    (430, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.42, t_col, 1)

        # Position
        p = frame.position
        cv2.putText(canvas,
                    f"Pos: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})m",
                    (10, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.38, COL_HUD, 1)

        # Roll / Pitch / Yaw
        q = frame.quaternion
        roll, pitch, yaw = quaternion_to_euler(q[0], q[1], q[2], q[3])
        roll_d, pitch_d, yaw_d = np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
        cv2.putText(canvas,
                    f"R:{roll_d:+.1f}  P:{pitch_d:+.1f}  Y:{yaw_d:+.1f} deg",
                    (380, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (160, 160, 180), 1)


def draw_timeline(canvas, frames, cursor_idx, playing, mode_label, w, h):
    """Draw the timeline scrubber bar at the bottom."""
    bar_top = h - TIMELINE_H
    n = max(len(frames), 1)

    # Background
    cv2.rectangle(canvas, (0, bar_top), (w, h), (30, 30, 30), -1)
    cv2.line(canvas, (0, bar_top), (w, bar_top), COL_DIVIDER, 1)

    bar_x0 = 15
    bar_x1 = w - 15
    bar_y = bar_top + 20
    bar_h = 10

    # Background bar
    cv2.rectangle(canvas, (bar_x0, bar_y), (bar_x1, bar_y + bar_h),
                  COL_BAR_BG, -1)

    # Filled portion
    if n > 1:
        fill_w = int((bar_x1 - bar_x0) * cursor_idx / (n - 1))
        cv2.rectangle(canvas, (bar_x0, bar_y),
                      (bar_x0 + fill_w, bar_y + bar_h), COL_BAR_FG, -1)

    # Cursor handle
    cx = bar_x0 + (int((bar_x1 - bar_x0) * cursor_idx / (n - 1)) if n > 1
                    else 0)
    cv2.rectangle(canvas, (cx - 3, bar_y - 3), (cx + 3, bar_y + bar_h + 3),
                  COL_CURSOR, -1)

    # Time labels
    if frames:
        t0 = frames[0].timestamp
        t_cur = frames[min(cursor_idx, len(frames) - 1)].timestamp - t0
        t_end = frames[-1].timestamp - t0
        cv2.putText(canvas, f"{t_cur:.2f}s", (bar_x0, bar_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, COL_HUD, 1)
        cv2.putText(canvas, f"{t_end:.2f}s", (bar_x1 - 40, bar_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, COL_HUD, 1)

    # Frame counter
    cv2.putText(canvas, f"Frame {cursor_idx}/{n - 1}",
                (bar_x0, bar_top + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)

    # Mode + play state
    play_icon = "||" if playing else ">"
    cv2.putText(canvas, f"{mode_label}  [{play_icon}]",
                (bar_x1 - 160, bar_top + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, COL_HUD, 1)


def draw_hud_labels(canvas, mode, w):
    """Draw mode label and help text."""
    y = 15
    if mode == "recording":
        cv2.putText(canvas, "RECORDING", (w - 130, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 220), 1)
    elif mode == "playback":
        cv2.putText(canvas, "PLAYBACK", (w - 120, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 180, 0), 1)
    elif mode == "calibrating":
        cv2.putText(canvas, "CALIBRATING", (w - 150, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    # Panel label
    cv2.putText(canvas, "TOP (XY)", (5, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)


# ──────────────────────────────────────────────────────────────────────────────
# Composite Render
# ──────────────────────────────────────────────────────────────────────────────

def render_frame(canvas, top_mapper, side_mapper, frames, cursor_idx,
                 mode, playing, table_cal):
    """Render the full multi-view visualization."""
    h, w = canvas.shape[:2]
    scene_h = h - TIMELINE_H - INFO_BAR_H
    canvas[:] = 20  # Dark background

    # Top-down view (left panel)
    draw_grid_topdown(canvas, top_mapper, scene_h)
    if frames:
        frame = frames[min(cursor_idx, len(frames) - 1)]
        draw_trail_topdown(canvas, top_mapper, frames, cursor_idx, scene_h)
        draw_lshape_topdown(canvas, top_mapper, frame, scene_h)
    else:
        frame = None

    # Side view (right panel)
    draw_grid_side(canvas, side_mapper, scene_h, table_cal)
    if frames:
        draw_trail_side(canvas, side_mapper, frames, cursor_idx, scene_h)
        draw_lshape_side(canvas, side_mapper, frame, scene_h, table_cal)

    # HUD labels
    draw_hud_labels(canvas, mode, w)

    # Info bar
    draw_info_bar(canvas, frame, table_cal, scene_h, w)

    # Timeline
    draw_timeline(canvas, frames, cursor_idx, playing,
                  "REC" if mode == "recording" else "PLAY", w, h)


# ──────────────────────────────────────────────────────────────────────────────
# Demo Mode
# ──────────────────────────────────────────────────────────────────────────────

def generate_demo_frames(cs100, table_cal, duration=5.0, fps=120.0):
    """Generate demo frames with simulated tilt and height changes."""
    frames = []
    n = int(duration * fps)
    t0 = time.time()

    for i in range(n):
        t = i / fps
        # Figure-8 XY motion
        cx = 0.15 * np.sin(t * 0.8)
        cy = 0.10 * np.sin(t * 1.6)

        # Height oscillation: lifts off table and returns
        cz = table_cal.table_z + 0.02 + 0.12 * max(0, np.sin(t * 0.6))

        # Tilt: roll and pitch oscillate
        roll = 0.3 * np.sin(t * 1.2)   # ~17 deg max
        pitch = 0.2 * np.sin(t * 0.9)  # ~11 deg max
        yaw = t * 0.5

        # Build quaternion from Euler (simplified)
        cr, sr = np.cos(roll / 2), np.sin(roll / 2)
        cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
        cy_a, sy = np.cos(yaw / 2), np.sin(yaw / 2)

        qw = cr * cp * cy_a + sr * sp * sy
        qx = sr * cp * cy_a - cr * sp * sy
        qy = cr * sp * cy_a + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy_a

        pos = np.array([cx, cy, cz])
        quat = np.array([qx, qy, qz, qw])
        markers = cs100.compute_marker_positions(pos, quat)

        frame = RecordedFrame(t0 + t, pos, quat, markers)
        compute_frame_3d(cs100, frame, table_cal)
        frames.append(frame)

    return frames


# ──────────────────────────────────────────────────────────────────────────────
# Timeline Mouse Interaction
# ──────────────────────────────────────────────────────────────────────────────

class TimelineDragger:
    """Handles mouse click/drag on the timeline bar."""

    def __init__(self, canvas_w, canvas_h):
        self.canvas_w = canvas_w
        self.canvas_h = canvas_h
        self.dragging = False
        self.bar_x0 = 15
        self.bar_x1 = canvas_w - 15
        self.bar_top = canvas_h - TIMELINE_H

    def update_size(self, w, h):
        self.canvas_w = w
        self.canvas_h = h
        self.bar_x0 = 15
        self.bar_x1 = w - 15
        self.bar_top = h - TIMELINE_H

    def hit_test(self, x, y):
        return (y >= self.bar_top and self.bar_x0 <= x <= self.bar_x1)

    def x_to_index(self, x, n_frames):
        if n_frames <= 1:
            return 0
        frac = (x - self.bar_x0) / max(self.bar_x1 - self.bar_x0, 1)
        frac = np.clip(frac, 0.0, 1.0)
        return int(frac * (n_frames - 1))


# ──────────────────────────────────────────────────────────────────────────────
# Phase 0: Table Calibration
# ──────────────────────────────────────────────────────────────────────────────

def calibrate_table(client, body_name, cs100, canvas, window_name):
    """
    Calibrate the table plane by recording CS-100 placed flat on the table.
    Returns a TableCalibration object.
    """
    table_cal = TableCalibration()
    h, w = canvas.shape[:2]
    num_samples = 60

    print("\n[Calibration] Place CS-100 flat on the table.")
    print("[Calibration] Press ENTER to start calibrating, or Q to skip.\n")

    # Show waiting screen
    while True:
        canvas[:] = 20
        cv2.putText(canvas, "TABLE DEPTH CALIBRATION",
                    (w // 2 - 160, h // 2 - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, COL_CAMERA, 2)
        cv2.putText(canvas, "Place CS-100 FLAT on the table surface.",
                    (w // 2 - 200, h // 2 - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_HUD, 1)
        cv2.putText(canvas, "Press ENTER to calibrate, Q to skip.",
                    (w // 2 - 180, h // 2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 100), 1)

        # Show live body status
        bodies = client.get_rigid_bodies()
        if bodies:
            matched = find_body(bodies, body_name)
            if matched:
                body = bodies[matched]
                p = body.position
                cv2.putText(canvas,
                            f"'{matched}' detected at "
                            f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})",
                            (w // 2 - 200, h // 2 + 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 255, 0), 1)
            else:
                cv2.putText(canvas, f"Body '{body_name}' not found!",
                            (w // 2 - 140, h // 2 + 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 0, 255), 1)
        else:
            cv2.putText(canvas, "No bodies detected...",
                        (w // 2 - 100, h // 2 + 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 200), 1)

        cv2.imshow(window_name, canvas)
        key = cv2.waitKey(100) & 0xFF
        if key == 13 or key == 10:  # Enter
            break
        elif key == ord('q') or key == 27:
            print("[Calibration] Skipped.")
            return table_cal

    # Record calibration samples
    estimator = FloorDepthEstimator(cs100)
    sample_count = 0

    print(f"[Calibration] Recording {num_samples} samples...")
    start = time.time()

    while sample_count < num_samples and time.time() - start < 10.0:
        bodies = client.get_rigid_bodies()
        matched = find_body(bodies, body_name) if bodies else None

        if matched:
            body = bodies[matched]
            pos = body.position
            quat = body.quaternion
            if (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))
                    and np.linalg.norm(pos) < 100.0
                    and np.linalg.norm(quat) > 0.5):
                estimator.record_sample(pos, quat)
                sample_count += 1

        # Show progress
        canvas[:] = 20
        pct = sample_count / num_samples
        bar_w = int(pct * (w - 100))
        cv2.rectangle(canvas, (50, h // 2 - 10),
                      (50 + bar_w, h // 2 + 10), COL_BAR_FG, -1)
        cv2.rectangle(canvas, (50, h // 2 - 10),
                      (w - 50, h // 2 + 10), COL_HUD, 1)
        cv2.putText(canvas, f"Calibrating... {sample_count}/{num_samples}",
                    (w // 2 - 100, h // 2 - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_CAMERA, 1)
        cv2.imshow(window_name, canvas)
        cv2.waitKey(8)

    if sample_count < 10:
        print(f"[Calibration] FAILED: only {sample_count} samples. "
              f"Is the CS-100 visible?")
        return table_cal

    # Compute
    result = estimator.compute(min_samples=min(10, sample_count))
    estimator.print_report()

    table_cal.table_z = result["floor_z"]
    table_cal.table_point = np.array(result["floor_point"])
    table_cal.table_normal = np.array(result["floor_normal"])
    table_cal.camera_to_table_depth = result["camera_to_floor_depth"]
    table_cal.is_calibrated = True

    # Validate
    v = result["marker_validation"]
    if not v["is_valid"]:
        print("[Calibration] WARNING: Marker geometry validation failed!")
        print(f"  Short arm: {v['short_arm_dist_m']*100:.2f}cm "
              f"(err: {v['short_arm_error_mm']:.1f}mm)")
        print(f"  Long arm:  {v['long_arm_dist_m']*100:.2f}cm "
              f"(err: {v['long_arm_error_mm']:.1f}mm)")

    if result["flatness_deg"] > 5.0:
        print(f"[Calibration] WARNING: CS-100 not flat! "
              f"Flatness: {result['flatness_deg']:.1f}deg (should be < 5)")

    print(f"\n[Calibration] Camera → Table depth: "
          f"{table_cal.camera_to_table_depth:.4f}m "
          f"({table_cal.camera_to_table_depth*100:.1f}cm)")
    print(f"[Calibration] Table Z: {table_cal.table_z:.4f}m")
    print(f"[Calibration] Table normal: ({table_cal.table_normal[0]:.3f}, "
          f"{table_cal.table_normal[1]:.3f}, {table_cal.table_normal[2]:.3f})")

    return table_cal


# ──────────────────────────────────────────────────────────────────────────────
# Phase 1: Recording
# ──────────────────────────────────────────────────────────────────────────────

def find_body(bodies, body_name):
    """Flexible rigid body name matching."""
    if not bodies:
        return None
    for name in bodies:
        if (name == body_name or
            name.lower() == body_name.lower() or
            body_name.lower().replace("-", "").replace("_", "") in
                name.lower().replace("-", "").replace("_", "")):
            return name
    return None


def record_live(client, body_name, cs100, table_cal, duration,
                canvas, top_mapper, side_mapper, window_name, dragger):
    """Record frames from OptiTrack with live multi-view preview."""
    frames = []
    start_time = time.time()
    paused = False
    h, w = canvas.shape[:2]

    print(f"[Tracker] Recording for {duration:.1f}s...")

    while True:
        now = time.time()
        elapsed = now - start_time

        # Get data
        bodies = client.get_rigid_bodies()
        matched = find_body(bodies, body_name) if bodies else None

        if matched:
            body = bodies[matched]
            pos = body.position
            quat = body.quaternion

            if (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))
                    and np.linalg.norm(pos) < 100.0
                    and np.linalg.norm(quat) > 0.5):

                markers = cs100.compute_marker_positions(pos, quat)
                frame = RecordedFrame(now, pos.copy(), quat.copy(), markers)
                compute_frame_3d(cs100, frame, table_cal)

                if not paused:
                    frames.append(frame)
                    if len(frames) % 60 == 0:
                        top_mapper.fit_to_data(frames, padding=0.05)
                        side_mapper.fit_to_data(frames,
                                                table_z=table_cal.table_z)

        # Render
        idx = len(frames) - 1 if frames else 0
        render_frame(canvas, top_mapper, side_mapper, frames, idx,
                     "recording", not paused, table_cal)

        # Recording progress overlay
        cv2.putText(canvas,
                    f"{elapsed:.1f}s / {duration:.0f}s  ({len(frames)} frames)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COL_HUD, 1)

        cv2.imshow(window_name, canvas)

        if elapsed >= duration:
            print(f"[Tracker] Recording complete: {len(frames)} frames")
            break

        key = cv2.waitKey(8) & 0xFF
        if key == ord('q') or key == 27:
            print(f"[Tracker] Stopped early: {len(frames)} frames "
                  f"in {elapsed:.1f}s")
            break
        elif key == ord(' '):
            paused = not paused
            print(f"[Tracker] {'Paused' if paused else 'Recording'}")

    if frames:
        top_mapper.fit_to_data(frames, padding=0.05)
        side_mapper.fit_to_data(frames, table_z=table_cal.table_z)

    return frames


# ──────────────────────────────────────────────────────────────────────────────
# Phase 2: Playback
# ──────────────────────────────────────────────────────────────────────────────

def playback(frames, canvas, top_mapper, side_mapper, window_name,
             dragger, table_cal, auto_save):
    """Interactive timeline scrubber with multi-view playback."""
    if not frames:
        print("[Tracker] No frames to play back.")
        return "quit"

    n = len(frames)
    cursor = 0
    playing = False
    play_speed = 1.0
    play_t0 = None
    play_cursor0 = 0
    snapshot_count = 0
    h, w = canvas.shape[:2]

    duration = frames[-1].timestamp - frames[0].timestamp
    print(f"[Tracker] Playback: {n} frames, {duration:.2f}s")

    if auto_save:
        snapshot_count += 1
        render_frame(canvas, top_mapper, side_mapper, frames, n - 1,
                     "playback", False, table_cal)
        path = f"output/trace_{snapshot_count:04d}.png"
        cv2.imwrite(path, canvas)
        print(f"[Tracker] Auto-saved to {path}")

    mouse_state = {"down": False}

    def on_mouse(event, x, y, flags, param):
        nonlocal cursor, playing
        if event == cv2.EVENT_LBUTTONDOWN:
            if dragger.hit_test(x, y):
                mouse_state["down"] = True
                cursor = dragger.x_to_index(x, n)
                playing = False
        elif event == cv2.EVENT_MOUSEMOVE:
            if mouse_state["down"] and dragger.hit_test(x, y):
                cursor = dragger.x_to_index(x, n)
        elif event == cv2.EVENT_LBUTTONUP:
            mouse_state["down"] = False

    cv2.setMouseCallback(window_name, on_mouse)

    while True:
        # Auto-play
        if playing and not mouse_state["down"]:
            if play_t0 is None:
                play_t0 = time.time()
                play_cursor0 = cursor
            elapsed_play = (time.time() - play_t0) * play_speed
            if duration > 0:
                target_t = (frames[play_cursor0].timestamp -
                            frames[0].timestamp + elapsed_play)
                for i in range(play_cursor0, n):
                    if frames[i].timestamp - frames[0].timestamp >= target_t:
                        cursor = i
                        break
                else:
                    cursor = n - 1
                    playing = False
                    play_t0 = None
            else:
                cursor = min(cursor + 1, n - 1)
                if cursor >= n - 1:
                    playing = False

        cursor = max(0, min(cursor, n - 1))

        render_frame(canvas, top_mapper, side_mapper, frames, cursor,
                     "playback", playing, table_cal)

        # Help text
        scene_h = h - TIMELINE_H - INFO_BAR_H
        cv2.putText(canvas,
                    "Timeline | Arrows | Space=play | R=re-record | "
                    "S=save | Q=quit",
                    (10, scene_h - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (70, 70, 70), 1)

        cv2.imshow(window_name, canvas)
        key = cv2.waitKey(16) & 0xFF

        if key == ord('q') or key == 27:
            break
        elif key == ord(' '):
            playing = not playing
            if playing:
                play_t0 = None
        elif key == 81 or key == 2:  # Left
            cursor = max(0, cursor - 1)
            playing = False
        elif key == 83 or key == 3:  # Right
            cursor = min(n - 1, cursor + 1)
            playing = False
        elif key == 80 or key == 0:  # Up — skip 10
            cursor = min(n - 1, cursor + 10)
            playing = False
        elif key == 82 or key == 1:  # Down — skip 10 back
            cursor = max(0, cursor - 10)
            playing = False
        elif key == ord('a'):  # Home
            cursor = 0
            playing = False
        elif key == ord('e'):  # End
            cursor = n - 1
            playing = False
        elif key == ord('s'):
            snapshot_count += 1
            path = f"output/trace_{snapshot_count:04d}.png"
            cv2.imwrite(path, canvas)
            print(f"[Tracker] Saved to {path}")
        elif key == ord('r'):
            return "rerecord"

    return "quit"


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="CS-100 3D tracker with table depth calibration")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "..",
                             "config", "scene_config.yaml"))
    parser.add_argument("--no-optitrack", action="store_true",
                        help="Demo mode (no OptiTrack)")
    parser.add_argument("--skip-calibration", action="store_true",
                        help="Skip table depth calibration")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Recording duration in seconds (default: 5)")
    parser.add_argument("--width", type=int, default=1000,
                        help="Window width")
    parser.add_argument("--height", type=int, default=750,
                        help="Window height")
    parser.add_argument("--save", action="store_true",
                        help="Auto-save trace image after recording")
    parser.add_argument("--body", default=None,
                        help="Rigid body name (default: from config)")
    args = parser.parse_args()

    config = load_config(args.config)
    objects_cfg = config.get("objects", {})

    # Body name
    body_name = args.body
    if body_name is None:
        cal_cfg = config.get("calibration", {})
        body_name = cal_cfg.get("tool_body", "Rigid_3_Balls")

    # CS-100 geometry
    cs100_cfg = objects_cfg.get(body_name, {})
    if not cs100_cfg:
        for name, cfg in objects_cfg.items():
            if cfg.get("render_as") == "cs100_lshape":
                cs100_cfg = cfg
                break

    cs100 = CS100Geometry(
        short_arm_length=cs100_cfg.get("short_arm_length", 0.08),
        long_arm_length=cs100_cfg.get("long_arm_length", 0.10),
    )

    # Canvas and layout
    W, H = args.width, args.height
    scene_h = H - TIMELINE_H - INFO_BAR_H
    side_w = int(W * SIDE_FRAC)
    top_w = W - side_w

    canvas = np.zeros((H, W, 3), dtype=np.uint8)
    top_mapper = TopDownMapper(top_w, scene_h, offset_x=0, offset_y=0)
    side_mapper = SideViewMapper(side_w, scene_h, offset_x=top_w, offset_y=0)
    dragger = TimelineDragger(W, H)

    os.makedirs("output", exist_ok=True)
    window_name = "CS-100 3D Tracker"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    print(f"""
========================================================================
  CS-100 3D Tracker — Table Depth Calibration + Tilt Tracking
========================================================================
  Body: {body_name}
  Duration: {args.duration:.0f}s
  Calibration: {'SKIP' if args.skip_calibration else 'ENABLED'}

  Phase 0: Table depth calibration
  Phase 1: Record motion ({args.duration:.0f}s)
  Phase 2: Multi-view playback with timeline
========================================================================
""")

    # Connect to OptiTrack
    client = None
    table_cal = TableCalibration()

    if not args.no_optitrack:
        optitrack_cfg = config.get("optitrack", {})
        client = OptiTrackClient(
            server_ip=optitrack_cfg.get("server_ip", "192.168.0.101"),
            local_ip=optitrack_cfg.get("local_ip", "0.0.0.0"),
            multicast_ip=optitrack_cfg.get("multicast_ip", "239.255.42.99"),
            command_port=optitrack_cfg.get("command_port", 1510),
            data_port=optitrack_cfg.get("data_port", 1511),
        )
        client.start()
        time.sleep(1.5)

        print(f"[Tracker] NatNet: {client.natnet_version}, "
              f"Server: {client.server_app_name or 'no response'}")

        # Wait for body
        found = False
        for _ in range(50):
            bodies = client.get_rigid_bodies()
            matched = find_body(bodies, body_name) if bodies else None
            if matched:
                body_name = matched
                found = True
                break
            if bodies and not found:
                body_name = list(bodies.keys())[0]
                found = True
                break
            time.sleep(0.1)

        if found:
            print(f"[Tracker] Using: '{body_name}'")
        else:
            print("[Tracker] No bodies detected, switching to demo mode.")
            args.no_optitrack = True

        # Phase 0: Table calibration
        if not args.no_optitrack and not args.skip_calibration:
            table_cal = calibrate_table(client, body_name, cs100,
                                        canvas, window_name)
    else:
        if args.skip_calibration:
            # Demo mode default table
            table_cal.table_z = -0.5
            table_cal.table_point = np.array([0.0, 0.0, -0.5])
            table_cal.table_normal = np.array([0.0, 0.0, 1.0])
            table_cal.camera_to_table_depth = 0.5
            table_cal.is_calibrated = True

    # Main loop (supports re-recording)
    while True:
        if args.no_optitrack:
            print("[Tracker] Generating demo data...")
            frames = generate_demo_frames(cs100, table_cal, args.duration)
            top_mapper.fit_to_data(frames, padding=0.05)
            side_mapper.fit_to_data(frames, table_z=table_cal.table_z)
        else:
            frames = record_live(client, body_name, cs100, table_cal,
                                 args.duration, canvas, top_mapper,
                                 side_mapper, window_name, dragger)

        if not frames:
            print("[Tracker] No frames recorded.")
            break

        result = playback(frames, canvas, top_mapper, side_mapper,
                          window_name, dragger, table_cal, args.save)
        if result == "rerecord":
            print("[Tracker] Re-recording...")
            continue
        break

    cv2.destroyAllWindows()
    if client:
        client.stop()
    print("[Tracker] Done.")


if __name__ == "__main__":
    main()
