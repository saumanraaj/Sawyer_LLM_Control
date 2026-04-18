#!/usr/bin/env python3
"""
Live ZED stereo viewer — single blue cube, calibration-friendly.
No SDK or ROS required.

Two windows
  Left  — blue cube detection with live diagnostics
  Right — raw right eye (stereo reference)

Controls
  Q / Esc   quit
  S         save snapshot
  H         toggle blue HSV mask overlay
  F         toggle shape-filter enforcement
              OFF → any blue blob shown (use for HSV tuning)
              ON  → only cube-shaped blobs shown
  Click     print HSV at that pixel + auto-suggest HSV range
  [ / ]     decrease / increase min_solidity  by 0.05
  ; / '     decrease / increase max_aspect_err by 0.05
"""

import sys
import os
import time
import cv2
import numpy as np

# ── package path ──────────────────────────────────────────────────────────────
_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
_PKG_SRC = os.path.join(os.path.dirname(_SCRIPTS), "src")
sys.path.insert(0, _PKG_SRC)

from sawyer_vision_tracker.detector import Detector
from sawyer_vision_tracker.tracker  import ObjectTracker

# ── initial config (tune hsv_lower/upper via click + H mask) ─────────────────
DEFAULT_HSV_LOWER = [95,  80, 40]    # wide start — tighten after clicking cube
DEFAULT_HSV_UPPER = [135, 255, 255]

DETECTOR_CFG = {
    "blue_cube": {
        "hsv_lower":            DEFAULT_HSV_LOWER,
        "hsv_upper":            DEFAULT_HSV_UPPER,
        "min_area":             800,
        "max_area":             200000,
        "min_solidity":         0.75,
        "max_aspect_ratio_err": 0.50,
        "morph_kernel_size":    7,
        "gaussian_blur":        5,
    },
}

TRACKER_CFG = {
    "max_disappeared":   20,
    "max_distance":      80,
    "smoothing_alpha":   0.35,
    "trajectory_length": 40,
}

# ZED HD720 left-eye intrinsics (cx/cy updated for 1280×720)
FX, FY = 700.0, 700.0    # approximate — calibrate with ZED SDK for accuracy
CX, CY = 640.5, 360.5
FIXED_Z = 0.70

FONT  = cv2.FONT_HERSHEY_SIMPLEX
WHITE = (255, 255, 255)
BLACK = (0,   0,   0)
CYAN  = (0,   220, 255)   # yellow in BGR — visible on blue objects
GREY  = (160, 160, 160)
GREEN = (80,  220, 80)
RED   = (60,  60,  220)
ORG   = (0,   165, 255)   # orange — shape-filter rejected


# ── helpers ───────────────────────────────────────────────────────────────────

def lbl(img, text, x, y, fg=WHITE, bg=BLACK, scale=0.50, thick=1):
    cv2.putText(img, text, (x + 1, y + 1), FONT, scale, bg,  thick + 1)
    cv2.putText(img, text, (x,     y    ), FONT, scale, fg,  thick)


def project_xy(u, v):
    x = (u - CX) * FIXED_Z / FX
    y = (v - CY) * FIXED_Z / FY
    return x, y, FIXED_Z


def make_detector(state):
    cfg = dict(DETECTOR_CFG)
    cfg["blue_cube"] = dict(DETECTOR_CFG["blue_cube"])
    cfg["blue_cube"]["hsv_lower"]            = list(state["hsv_lower"])
    cfg["blue_cube"]["hsv_upper"]            = list(state["hsv_upper"])
    cfg["blue_cube"]["min_solidity"]         = state["min_solidity"]
    cfg["blue_cube"]["max_aspect_ratio_err"] = state["max_aspect_err"]
    return Detector(cfg)


# ── raw contour diagnostics (before shape filter) ─────────────────────────────

def get_raw_candidates(detector, frame):
    """Return all blue-mask contours with shape metrics, no filtering."""
    mask     = detector.get_blue_mask(frame)
    mask_px  = int(mask.sum() / 255)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 100:
            continue
        hull      = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity  = area / hull_area if hull_area > 0 else 0
        x, y, bw, bh = cv2.boundingRect(cnt)
        aspect    = bw / bh if bh > 0 else 0
        cands.append(dict(
            cnt=cnt, area=area, sol=solidity,
            ar=aspect, bbox=(x, y, bw, bh),
        ))
    return mask_px, cands


def draw_candidates(canvas, cands, state):
    """
    Draw all raw blue contours.
    Green  → passes shape filter
    Orange → fails shape (area / solidity / aspect)
    """
    for c in cands:
        passes_area = state["min_area"] <= c["area"] <= state["max_area"]
        passes_sol  = c["sol"]  >= state["min_solidity"]
        passes_ar   = abs(c["ar"] - 1.0) <= state["max_aspect_err"]
        ok = passes_area and passes_sol and passes_ar

        color = GREEN if ok else ORG
        cv2.drawContours(canvas, [c["cnt"]], -1, color, 1)

        x, y, bw, bh = c["bbox"]
        label_parts = []
        if not passes_area:
            label_parts.append(f"a={int(c['area'])}")
        if not passes_sol:
            label_parts.append(f"s={c['sol']:.2f}")
        if not passes_ar:
            label_parts.append(f"ar={c['ar']:.2f}")

        reason = ",".join(label_parts) if label_parts else "ok"
        lbl(canvas, reason, x, max(y - 4, 12), fg=color, scale=0.35)


# ── cube annotation ───────────────────────────────────────────────────────────

def draw_cube(canvas, det, tracked_obj):
    col = CYAN
    x, y, w, h = det.bbox
    cx, cy = det.centroid

    cv2.rectangle(canvas, (x, y), (x + w, y + h), col, 2)

    tick = 14
    for px, py, dx, dy in [
        (x,     y,     1,  1), (x + w, y,     -1,  1),
        (x,     y + h, 1, -1), (x + w, y + h, -1, -1),
    ]:
        cv2.line(canvas, (px, py), (px + dx * tick, py), col, 2)
        cv2.line(canvas, (px, py), (px, py + dy * tick), col, 2)

    cv2.line(canvas,   (cx - 16, cy), (cx + 16, cy), col, 2)
    cv2.line(canvas,   (cx, cy - 16), (cx, cy + 16), col, 2)
    cv2.circle(canvas, (cx, cy), 5, col, -1)

    if tracked_obj is not None:
        pts = list(tracked_obj.trajectory)
        for i in range(1, len(pts)):
            alpha = i / max(len(pts), 1)
            p1 = (int(pts[i - 1][0]), int(pts[i - 1][1]))
            p2 = (int(pts[i][0]),     int(pts[i][1]))
            cv2.line(canvas, p1, p2,
                     tuple(int(c * alpha) for c in col),
                     max(1, int(alpha * 2)))

    # info panel
    x_cam, y_cam, z_cam = project_xy(cx, cy)
    lines = [
        "BLUE CUBE",
        f"area:   {int(det.area):,} px",
        f"solid:  {det.solidity:.2f}",
        f"aspect: {det.aspect_ratio:.2f}",
        f"cam X:  {x_cam:+.3f} m",
        f"cam Y:  {y_cam:+.3f} m",
        f"cam Z:  {z_cam:.3f} m (fixed)",
    ]
    lh, pad = 18, 5
    pw, ph  = 175, len(lines) * lh + pad * 2
    px0, py0 = max(0, x - 2), max(0, y - ph - 4)
    roi = canvas[py0:py0 + ph, px0:px0 + pw]
    if roi.size > 0:
        canvas[py0:py0 + ph, px0:px0 + pw] = cv2.addWeighted(
            roi, 0.3, np.zeros_like(roi), 0.7, 0)
    for i, line in enumerate(lines):
        fg = CYAN if i == 0 else (WHITE if i < 5 else GREEN)
        lbl(canvas, line, px0 + pad, py0 + pad + (i + 1) * lh,
            fg=fg, scale=0.42)


# ── HUD ──────────────────────────────────────────────────────────────────────

def draw_hud(canvas, det, mask_px, n_cands, n_pass, state, fps):
    fw = canvas.shape[1]
    bar = np.zeros((56, fw, 3), dtype=np.uint8)

    # line 1: detection status
    if det is not None:
        l1_txt = f"DETECTED  solid={det.solidity:.2f}  ar={det.aspect_ratio:.2f}  area={int(det.area)}"
        l1_col = GREEN
    elif n_pass > 0:
        l1_txt = f"SHAPE FILTER BLOCKING  ({n_pass} candidates passing shape but tracker dropped)"
        l1_col = ORG
    elif n_cands > 0:
        l1_txt = f"SHAPE FILTER REJECTING {n_cands} blue blobs  — press F to disable, [ ] ; ' to relax"
        l1_col = ORG
    elif mask_px > 0:
        l1_txt = f"HSV MASK HIT ({mask_px} px) but all contours too small  — lower min_area?"
        l1_col = ORG
    else:
        l1_txt = "NO BLUE PIXELS  — click cube in frame, see terminal for HSV, update config"
        l1_col = RED

    lbl(bar, l1_txt, 6, 17, fg=l1_col, scale=0.46)

    # line 2: current params
    sf_txt  = "ShapeFilter:ON " if state["shape_filter"] else "ShapeFilter:OFF"
    mask_txt = f"MaskPx:{mask_px}  Blobs:{n_cands}(pass:{n_pass})"
    hsv_txt  = f"HSV L={state['hsv_lower']} U={state['hsv_upper']}"
    lbl(bar, f"{sf_txt}  {mask_txt}  sol>={state['min_solidity']:.2f}  ar_err<={state['max_aspect_err']:.2f}",
        6, 34, fg=GREY, scale=0.38)
    lbl(bar, f"{hsv_txt}  FPS:{fps:.0f}",
        6, 50, fg=GREY, scale=0.38)

    # shortcuts
    shortcuts = "[H]mask [F]shape [S]snap [Q]quit  click=HSV  []/;' shape thresholds"
    lbl(bar, shortcuts, fw - 460, 50, fg=(100, 100, 100), scale=0.35)

    return np.vstack([canvas, bar])


# ── right eye ─────────────────────────────────────────────────────────────────

def draw_right(frame, has_cube):
    out = frame.copy()
    h, w = out.shape[:2]
    lbl(out, "RIGHT EYE — stereo reference", 8, 26, fg=GREY, scale=0.52)
    lbl(out, f"{w}x{h}", 8, h - 10, fg=GREY, scale=0.42)
    cx, cy = w // 2, h // 2
    cv2.line(out, (cx - 20, cy), (cx + 20, cy), (60, 60, 60), 1)
    cv2.line(out, (cx, cy - 20), (cx, cy + 20), (60, 60, 60), 1)
    if has_cube:
        lbl(out, "cube visible in left", 8, 52, fg=GREEN, scale=0.48)
    return out


# ── click-to-inspect HSV ──────────────────────────────────────────────────────
_click_frame = None

def on_mouse(event, x, y, *_):
    global _click_frame
    if event == cv2.EVENT_LBUTTONDOWN and _click_frame is not None:
        left = _click_frame
        if 0 <= x < left.shape[1] and 0 <= y < left.shape[0]:
            bgr  = left[y, x].tolist()
            hsv  = cv2.cvtColor(
                np.array([[bgr]], dtype=np.uint8), cv2.COLOR_BGR2HSV
            )[0][0].tolist()
            H, S, V = hsv
            tol_h, tol_s, tol_v = 12, 60, 60
            lo = [max(0, H - tol_h), max(0, S - tol_s), max(0, V - tol_v)]
            hi = [min(179, H + tol_h), 255, 255]
            print(f"\n[HSV click] ({x},{y})  BGR={bgr}  HSV=H:{H} S:{S} V:{V}")
            print(f"  → Suggested config in vision_tracker.yaml:")
            print(f"      hsv_lower: {lo}")
            print(f"      hsv_upper: {hi}")
            print("  Press R in the viewer to apply this range live.\n")
            # Store for 'R' key press
            on_mouse._suggested = (lo, hi)

on_mouse._suggested = None


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    global _click_frame

    # ── negotiate highest available resolution ────────────────────────────────
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        sys.exit("[ERROR] Cannot open /dev/video0 — ZED plugged in?")

    preferred = [(2560, 720, "HD720"), (3840, 1080, "FHD"), (1344, 376, "VGA")]
    chosen_label = "unknown"
    for target_w, target_h, label in preferred:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  target_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_h)
        ok, probe = cap.read()
        if ok and probe.shape[1] == target_w:
            chosen_label = label
            break

    ok, probe = cap.read()
    if not ok:
        sys.exit("[ERROR] Could not read from ZED.")

    full_h, full_w = probe.shape[:2]
    eye_w, eye_h   = full_w // 2, full_h
    print(f"ZED: {full_w}x{full_h} [{chosen_label}]  → each eye {eye_w}x{eye_h}")
    print("Click on the blue cube in the LEFT window → terminal prints HSV + suggested range.")
    print("Then press R to apply, H to see mask, F to toggle shape filter.")

    # ── live tuning state ─────────────────────────────────────────────────────
    state = {
        "hsv_lower":    list(DEFAULT_HSV_LOWER),
        "hsv_upper":    list(DEFAULT_HSV_UPPER),
        "min_solidity": DETECTOR_CFG["blue_cube"]["min_solidity"],
        "max_aspect_err": DETECTOR_CFG["blue_cube"]["max_aspect_ratio_err"],
        "min_area":     DETECTOR_CFG["blue_cube"]["min_area"],
        "max_area":     DETECTOR_CFG["blue_cube"]["max_area"],
        "shape_filter": True,
        "show_mask":    False,
    }

    detector = make_detector(state)
    tracker  = ObjectTracker(TRACKER_CFG)

    snap_count = 0
    fps        = 0.0
    t_last     = time.time()
    frame_cnt  = 0

    left_win  = "ZED — Left Eye (blue cube)"
    right_win = "ZED — Right Eye (raw)"

    cv2.namedWindow(left_win,  cv2.WINDOW_NORMAL)
    cv2.namedWindow(right_win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(left_win,  eye_w, eye_h + 56)
    cv2.resizeWindow(right_win, eye_w, eye_h)
    cv2.moveWindow(left_win,  0,          0)
    cv2.moveWindow(right_win, eye_w + 10, 0)

    _cb_set = False

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        left  = frame[:, :eye_w, :]
        right = frame[:, eye_w:, :]
        _click_frame = left.copy()

        # ── detect ────────────────────────────────────────────────────────────
        det          = detector.detect_blue_cube(left) if state["shape_filter"] else None
        mask_px, cands = get_raw_candidates(detector, left)
        n_pass = sum(
            1 for c in cands
            if state["min_area"] <= c["area"] <= state["max_area"]
            and c["sol"] >= state["min_solidity"]
            and abs(c["ar"] - 1.0) <= state["max_aspect_err"]
        )

        # If shape filter off, create a Detection from the largest candidate
        if not state["shape_filter"] and cands:
            best = max(cands, key=lambda c: c["area"])
            from sawyer_vision_tracker.detector import Detection
            import numpy as _np
            x, y, bw, bh = best["bbox"]
            cx = x + bw // 2
            cy = y + bh // 2
            det = Detection(
                centroid=(cx, cy), bbox=best["bbox"],
                contour=best["cnt"], area=best["area"],
                color_name="blue_blob", color_bgr=(0, 220, 255),
                mask=_np.zeros((eye_h, eye_w), dtype=_np.uint8),
                solidity=best["sol"], aspect_ratio=best["ar"],
            )

        tracked_list = tracker.update([det] if det is not None else [])
        tracked_obj  = tracked_list[0] if tracked_list else None

        # ── FPS ───────────────────────────────────────────────────────────────
        frame_cnt += 1
        now = time.time()
        if now - t_last >= 1.0:
            fps = frame_cnt / (now - t_last)
            frame_cnt = 0
            t_last = now

        # ── build left canvas ─────────────────────────────────────────────────
        canvas = left.copy()

        if state["show_mask"]:
            blue_mask = detector.get_blue_mask(left)
            tint = np.zeros_like(canvas)
            tint[:, :, 1] = blue_mask
            canvas = cv2.addWeighted(canvas, 0.55, tint, 0.45, 0)

        # always draw raw candidates (green = pass, orange = fail shape)
        draw_candidates(canvas, cands, state)

        lbl(canvas, f"LEFT EYE  [{chosen_label}]", 8, 26, fg=GREEN, scale=0.54)

        if det is not None:
            draw_cube(canvas, det, tracked_obj)

        canvas = draw_hud(canvas, det, mask_px, len(cands), n_pass, state, fps)

        # ── right canvas ──────────────────────────────────────────────────────
        right_canvas = draw_right(right, det is not None)

        cv2.imshow(left_win,  canvas)
        cv2.imshow(right_win, right_canvas)

        key = cv2.waitKey(1) & 0xFF

        if not _cb_set:
            try:
                cv2.setMouseCallback(left_win, on_mouse)
                _cb_set = True
            except cv2.error:
                pass

        # ── keyboard ──────────────────────────────────────────────────────────
        if key in (ord('q'), 27):
            break

        elif key == ord('s'):
            snap_count += 1
            fl = f"snap_{snap_count:03d}_left.png"
            fr = f"snap_{snap_count:03d}_right.png"
            cv2.imwrite(fl, canvas)
            cv2.imwrite(fr, right_canvas)
            print(f"[Saved] {fl}  {fr}")

        elif key == ord('h'):
            state["show_mask"] = not state["show_mask"]
            print(f"[Mask] {'ON' if state['show_mask'] else 'off'}")

        elif key == ord('f'):
            state["shape_filter"] = not state["shape_filter"]
            tracker = ObjectTracker(TRACKER_CFG)   # reset IDs
            print(f"[ShapeFilter] {'ON' if state['shape_filter'] else 'OFF'}")

        elif key == ord('r') and on_mouse._suggested is not None:
            lo, hi = on_mouse._suggested
            state["hsv_lower"] = lo
            state["hsv_upper"] = hi
            detector = make_detector(state)
            tracker  = ObjectTracker(TRACKER_CFG)
            print(f"[Range applied] lower={lo}  upper={hi}")

        elif key == ord('['):
            state["min_solidity"] = max(0.40, round(state["min_solidity"] - 0.05, 2))
            detector = make_detector(state)
            print(f"[Solidity] min → {state['min_solidity']:.2f}")

        elif key == ord(']'):
            state["min_solidity"] = min(0.99, round(state["min_solidity"] + 0.05, 2))
            detector = make_detector(state)
            print(f"[Solidity] min → {state['min_solidity']:.2f}")

        elif key == ord(';'):
            state["max_aspect_err"] = max(0.05, round(state["max_aspect_err"] - 0.05, 2))
            detector = make_detector(state)
            print(f"[AspectErr] max → {state['max_aspect_err']:.2f}")

        elif key == ord("'"):
            state["max_aspect_err"] = min(1.00, round(state["max_aspect_err"] + 0.05, 2))
            detector = make_detector(state)
            print(f"[AspectErr] max → {state['max_aspect_err']:.2f}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
