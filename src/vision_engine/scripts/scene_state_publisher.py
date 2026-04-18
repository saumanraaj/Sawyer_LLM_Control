#!/usr/bin/env python3
"""
scene_state_publisher.py — OptiTrack Scene State Publisher + CS-100 Tracker

Reads 6DOF rigid body poses from OptiTrack via NatNet and publishes them
as structured pose messages for Sauman's LLM reasoning system.

Modes
─────
    (default)   : Prints pose messages to console
    --ros       : Publishes to /llm/scene_state as ROS String messages
    --track     : Opens OpenCV window tracking CS-100 center + 2D path on desk
    --demo      : Simulated data, no OptiTrack hardware needed

Usage
─────
    # Live OptiTrack, print to console:
    python scene_state_publisher.py --objects config/objects.yaml --ip 192.168.0.101

    # Track CS-100 with 2D path visualization:
    python scene_state_publisher.py --track --demo
    python scene_state_publisher.py --track --ip 192.168.0.101

    # Demo mode (no hardware):
    python scene_state_publisher.py --demo
"""

import sys
import time
import math
import argparse
import signal
import numpy as np
from collections import deque
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import yaml

# ── Resolve imports ──────────────────────────────────────────────────────────
_script_dir = Path(__file__).resolve().parent
_engine_dir = _script_dir.parent
if str(_engine_dir) not in sys.path:
    sys.path.insert(0, str(_engine_dir))

from cv.optitrack_client import OptiTrackClient, RigidBodyState


# ═════════════════════════════════════════════════════════════════════════════
# Object Registry
# ═════════════════════════════════════════════════════════════════════════════

def load_object_registry(yaml_path: str) -> Dict[str, str]:
    """Load objects.yaml → returns {optitrack_body_name: display_name}."""
    if not Path(yaml_path).exists():
        print(f"[SceneState] WARNING: {yaml_path} not found, using raw body names")
        return {}

    with open(yaml_path, "r") as f:
        raw = yaml.safe_load(f)

    if not raw:
        return {}

    entries = raw.get("objects", raw) if isinstance(raw, dict) else {}
    if not isinstance(entries, dict):
        return {}

    body_to_name = {}
    for display_name, props in entries.items():
        if isinstance(props, dict) and "optitrack_body" in props:
            body_to_name[props["optitrack_body"]] = display_name

    print(f"[SceneState] Loaded {len(body_to_name)} objects from {yaml_path}")
    for body, name in body_to_name.items():
        print(f"  {body} → {name}")

    return body_to_name


# ═════════════════════════════════════════════════════════════════════════════
# Format rigid body pose message
# ═════════════════════════════════════════════════════════════════════════════

def format_body_message(display_name: str, body: RigidBodyState) -> str:
    px, py, pz = body.position
    qx, qy, qz, qw = body.quaternion
    lines = [
        f"RigidBody: {display_name}",
        f"position:",
        f"  x: {px:.4f}",
        f"  y: {py:.4f}",
        f"  z: {pz:.4f}",
        f"orientation (quaternion):",
        f"  x: {qx:.4f}",
        f"  y: {qy:.4f}",
        f"  z: {qz:.4f}",
        f"  w: {qw:.4f}",
        f"timestamp: {body.timestamp:.2f}",
        f"frame: optitrack_world",
    ]
    return "\n".join(lines)


def format_scene_message(
    bodies: Dict[str, RigidBodyState],
    body_to_name: Dict[str, str],
) -> str:
    blocks = []
    for body_name, body in sorted(bodies.items()):
        if not body.tracking_valid:
            continue
        display_name = body_to_name.get(body_name, body_name)
        blocks.append(format_body_message(display_name, body))

    if not blocks:
        return "---\nNo objects tracked\n---"
    return "\n---\n".join(blocks)


# ═════════════════════════════════════════════════════════════════════════════
# Demo Data Generator
# ═════════════════════════════════════════════════════════════════════════════

class DemoOptiTrack:
    """Simulates OptiTrack data — CS-100 draws a figure-8 on the desk."""

    def __init__(self):
        self._t0 = time.time()

    def get_rigid_bodies(self) -> Dict[str, RigidBodyState]:
        t = time.time() - self._t0
        now = time.time()

        # CS-100: figure-8 (lemniscate) pattern on the desk
        scale = 0.15
        cx, cy = 0.45, 0.0  # center of desk
        denom = 1 + math.sin(t * 0.4) ** 2
        x = cx + scale * math.cos(t * 0.4) / denom
        y = cy + scale * math.sin(t * 0.4) * math.cos(t * 0.4) / denom
        z = 0.73  # table height

        # Slight rotation as it moves
        yaw = t * 0.2
        qz_val = math.sin(yaw / 2)
        qw_val = math.cos(yaw / 2)

        bodies = {
            "Rigid_3_Balls": RigidBodyState(
                name="Rigid_3_Balls", id=1,
                position=np.array([x, y, z]),
                quaternion=np.array([0.0, 0.0, qz_val, qw_val]),
                timestamp=now, tracking_valid=True,
            ),
        }

        # Also add some other objects for completeness
        bodies["rigid_body_1"] = RigidBodyState(
            name="rigid_body_1", id=2,
            position=np.array([0.30, -0.15, 0.735]),
            quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
            timestamp=now, tracking_valid=True,
        )
        bodies["rigid_body_2"] = RigidBodyState(
            name="rigid_body_2", id=3,
            position=np.array([0.60, 0.10, 0.735]),
            quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
            timestamp=now, tracking_valid=True,
        )

        return bodies

    def start(self):
        pass

    def stop(self):
        pass


# ═════════════════════════════════════════════════════════════════════════════
# CS-100 2D Path Tracker (OpenCV visualization)
# ═════════════════════════════════════════════════════════════════════════════

def run_tracker(args):
    """
    Track the CS-100 rigid body center and visualize its 2D path on the desk.

    Opens an OpenCV window showing a top-down view of the table with:
    - The CS-100 current position (large dot)
    - The full motion trail (fading line)
    - Grid overlay for scale reference
    - Live position readout
    """
    import cv2

    BODY_NAME = args.body  # default: "Rigid_3_Balls"

    # ── Connect ──────────────────────────────────────────────────────────
    if args.demo:
        print("[Tracker] Demo mode — simulated CS-100 figure-8 motion")
        client = DemoOptiTrack()
    else:
        print(f"[Tracker] Connecting to OptiTrack at {args.ip}...")
        client = OptiTrackClient(server_ip=args.ip)
        client.start()

    # ── Display settings ─────────────────────────────────────────────────
    WIN_W, WIN_H = 900, 700
    MARGIN = 60

    # Desk bounds in meters (auto-adjusts, but start with reasonable defaults)
    # X = forward/back, Y = left/right in OptiTrack Z-up frame
    desk_x_range = [0.1, 0.8]   # meters
    desk_y_range = [-0.4, 0.4]  # meters

    # Colors (BGR)
    BG_COLOR       = (30, 30, 30)
    GRID_COLOR     = (55, 55, 55)
    GRID_TEXT_COLOR = (100, 100, 100)
    DESK_COLOR     = (50, 45, 40)
    DESK_BORDER    = (80, 80, 80)
    TRAIL_COLOR    = (0, 200, 255)     # orange trail
    DOT_COLOR      = (0, 255, 120)     # green current position
    LOST_COLOR     = (0, 0, 200)       # red when tracking lost
    TEXT_COLOR     = (220, 220, 220)
    DIM_TEXT       = (140, 140, 140)

    # Trail buffer
    MAX_TRAIL = args.trail_length
    trail: deque = deque(maxlen=MAX_TRAIL)

    # Auto-range tracking
    x_min_seen, x_max_seen = 999.0, -999.0
    y_min_seen, y_max_seen = 999.0, -999.0

    def world_to_px(wx: float, wy: float) -> Tuple[int, int]:
        """Convert world XY (meters) → pixel coordinates on the canvas."""
        plot_w = WIN_W - 2 * MARGIN
        plot_h = WIN_H - 2 * MARGIN - 40  # leave space for info bar

        x_span = desk_x_range[1] - desk_x_range[0]
        y_span = desk_y_range[1] - desk_y_range[0]

        if x_span < 0.01:
            x_span = 0.01
        if y_span < 0.01:
            y_span = 0.01

        # X (forward) maps to vertical axis (top = far, bottom = near)
        # Y (left/right) maps to horizontal axis
        px = int(MARGIN + (wy - desk_y_range[0]) / y_span * plot_w)
        py = int(MARGIN + (1.0 - (wx - desk_x_range[0]) / x_span) * plot_h)
        return px, py

    frame_count = 0
    last_valid_pos = None
    tracking_lost = False
    start_time = time.time()

    print(f"[Tracker] Tracking body '{BODY_NAME}'. Press 'q' to quit, 'c' to clear trail.\n")

    while True:
        t_frame = time.time()
        bodies = client.get_rigid_bodies()

        body = bodies.get(BODY_NAME)
        current_pos = None

        if body is not None and body.tracking_valid:
            pos = body.position
            # Validate
            if np.all(np.isfinite(pos)) and np.linalg.norm(pos) < 50.0:
                current_pos = (float(pos[0]), float(pos[1]), float(pos[2]))
                trail.append((current_pos[0], current_pos[1], t_frame))
                last_valid_pos = current_pos
                tracking_lost = False

                # Auto-expand range with padding
                pad = 0.05
                x_min_seen = min(x_min_seen, current_pos[0])
                x_max_seen = max(x_max_seen, current_pos[0])
                y_min_seen = min(y_min_seen, current_pos[1])
                y_max_seen = max(y_max_seen, current_pos[1])

                desk_x_range[0] = min(desk_x_range[0], x_min_seen - pad)
                desk_x_range[1] = max(desk_x_range[1], x_max_seen + pad)
                desk_y_range[0] = min(desk_y_range[0], y_min_seen - pad)
                desk_y_range[1] = max(desk_y_range[1], y_max_seen + pad)
        else:
            tracking_lost = True

        # ── Draw ─────────────────────────────────────────────────────────
        canvas = np.full((WIN_H, WIN_W, 3), BG_COLOR, dtype=np.uint8)

        # Desk area
        tl = world_to_px(desk_x_range[1], desk_y_range[0])
        br = world_to_px(desk_x_range[0], desk_y_range[1])
        cv2.rectangle(canvas, tl, br, DESK_COLOR, -1)
        cv2.rectangle(canvas, tl, br, DESK_BORDER, 1)

        # Grid lines every 10cm
        x_span = desk_x_range[1] - desk_x_range[0]
        y_span = desk_y_range[1] - desk_y_range[0]

        grid_step = 0.10  # 10cm
        gx = math.floor(desk_x_range[0] / grid_step) * grid_step
        while gx <= desk_x_range[1]:
            p1 = world_to_px(gx, desk_y_range[0])
            p2 = world_to_px(gx, desk_y_range[1])
            cv2.line(canvas, p1, p2, GRID_COLOR, 1)
            lbl_pos = (p2[0] + 4, p2[1])
            cv2.putText(canvas, f"{gx:.1f}m", lbl_pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, GRID_TEXT_COLOR, 1)
            gx += grid_step

        gy = math.floor(desk_y_range[0] / grid_step) * grid_step
        while gy <= desk_y_range[1]:
            p1 = world_to_px(desk_x_range[0], gy)
            p2 = world_to_px(desk_x_range[1], gy)
            cv2.line(canvas, p1, p2, GRID_COLOR, 1)
            lbl_pos = (p1[0], p1[1] + 12)
            cv2.putText(canvas, f"{gy:.1f}m", lbl_pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, GRID_TEXT_COLOR, 1)
            gy += grid_step

        # ── Trail (fading) ───────────────────────────────────────────────
        trail_list = list(trail)
        n = len(trail_list)
        for i in range(1, n):
            # Fade: older = dimmer
            alpha = i / n
            r = int(TRAIL_COLOR[0] * alpha)
            g = int(TRAIL_COLOR[1] * alpha)
            b = int(TRAIL_COLOR[2] * alpha)
            thickness = max(1, int(2 * alpha))

            p1 = world_to_px(trail_list[i - 1][0], trail_list[i - 1][1])
            p2 = world_to_px(trail_list[i][0], trail_list[i][1])

            # Skip huge jumps (likely tracking glitch)
            dx = abs(trail_list[i][0] - trail_list[i - 1][0])
            dy = abs(trail_list[i][1] - trail_list[i - 1][1])
            if dx < 0.3 and dy < 0.3:
                cv2.line(canvas, p1, p2, (b, g, r), thickness, cv2.LINE_AA)

        # ── Current position dot ─────────────────────────────────────────
        if current_pos is not None:
            px, py_cv = world_to_px(current_pos[0], current_pos[1])
            cv2.circle(canvas, (px, py_cv), 10, DOT_COLOR, -1, cv2.LINE_AA)
            cv2.circle(canvas, (px, py_cv), 12, (255, 255, 255), 1, cv2.LINE_AA)

            # Crosshair
            cv2.line(canvas, (px - 18, py_cv), (px - 13, py_cv), DIM_TEXT, 1)
            cv2.line(canvas, (px + 13, py_cv), (px + 18, py_cv), DIM_TEXT, 1)
            cv2.line(canvas, (px, py_cv - 18), (px, py_cv - 13), DIM_TEXT, 1)
            cv2.line(canvas, (px, py_cv + 13), (px, py_cv + 18), DIM_TEXT, 1)
        elif last_valid_pos is not None:
            # Show last known position in red
            px, py_cv = world_to_px(last_valid_pos[0], last_valid_pos[1])
            cv2.circle(canvas, (px, py_cv), 10, LOST_COLOR, 2, cv2.LINE_AA)
            cv2.putText(canvas, "LOST", (px + 15, py_cv + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, LOST_COLOR, 1)

        # ── Info bar (bottom) ────────────────────────────────────────────
        info_y = WIN_H - 30
        cv2.line(canvas, (0, info_y - 10), (WIN_W, info_y - 10), DESK_BORDER, 1)

        if current_pos is not None:
            pos_text = (f"CS-100 Center:  X={current_pos[0]:.4f}  "
                        f"Y={current_pos[1]:.4f}  Z={current_pos[2]:.4f} m")
            cv2.putText(canvas, pos_text, (10, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_COLOR, 1)
        elif tracking_lost:
            cv2.putText(canvas, "CS-100: TRACKING LOST", (10, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, LOST_COLOR, 1)
        else:
            cv2.putText(canvas, f"Waiting for body '{BODY_NAME}'...", (10, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, DIM_TEXT, 1)

        # Trail length + elapsed time
        elapsed = t_frame - start_time
        status = (f"Trail: {len(trail_list)} pts | "
                  f"Elapsed: {elapsed:.1f}s | "
                  f"Frame: {frame_count} | "
                  f"'c' clear  'q' quit")
        cv2.putText(canvas, status, (WIN_W - 430, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, DIM_TEXT, 1)

        # ── Title bar ────────────────────────────────────────────────────
        cv2.putText(canvas, "CS-100 2D Desk Tracker", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1)
        cv2.putText(canvas, "Top-down view (X=forward, Y=left/right)",
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.35, DIM_TEXT, 1)

        # Axis labels
        cv2.putText(canvas, "Y -->", (WIN_W // 2 - 15, MARGIN - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, DIM_TEXT, 1)
        # Vertical X label
        mid_y = (MARGIN + WIN_H - 70) // 2
        cv2.putText(canvas, "X", (8, mid_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, DIM_TEXT, 1)
        cv2.putText(canvas, "|", (12, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, DIM_TEXT, 1)
        cv2.putText(canvas, "v", (10, mid_y + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, DIM_TEXT, 1)

        # ── Show ─────────────────────────────────────────────────────────
        cv2.imshow("CS-100 Desk Tracker", canvas)
        key = cv2.waitKey(max(1, int(1000 / args.rate))) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("c"):
            trail.clear()
            print("[Tracker] Trail cleared")

        frame_count += 1

    # Cleanup
    cv2.destroyAllWindows()
    if hasattr(client, "stop"):
        client.stop()
    print(f"[Tracker] Done. {frame_count} frames, {len(trail)} trail points.")


# ═════════════════════════════════════════════════════════════════════════════
# Console / ROS Mode
# ═════════════════════════════════════════════════════════════════════════════

def run_publisher(args):
    """Run in console-print or ROS mode."""
    body_to_name = load_object_registry(args.objects)

    if args.demo:
        print("[SceneState] Demo mode — simulated data")
        client = DemoOptiTrack()
    else:
        print(f"[SceneState] Connecting to OptiTrack at {args.ip}...")
        client = OptiTrackClient(server_ip=args.ip)
        client.start()

    ros_pub = None
    if args.ros:
        try:
            import rospy
            from std_msgs.msg import String
            rospy.init_node("scene_state_publisher", anonymous=False)
            ros_pub = rospy.Publisher(args.topic, String, queue_size=1)
            print(f"[SceneState] ROS publisher on {args.topic}")
        except ImportError:
            print("[SceneState] ERROR: rospy not found.")
            sys.exit(1)

    running = [True]
    def on_signal(sig, frame):
        running[0] = False
        print("\n[SceneState] Stopping...")
    signal.signal(signal.SIGINT, on_signal)

    print(f"[SceneState] Publishing at {args.rate} Hz. Ctrl+C to stop.\n")
    interval = 1.0 / args.rate
    count = 0

    while running[0]:
        t0 = time.time()
        if args.ros:
            import rospy
            if rospy.is_shutdown():
                break

        bodies = client.get_rigid_bodies()
        msg_text = format_scene_message(bodies, body_to_name)

        if ros_pub is not None:
            from std_msgs.msg import String
            ros_pub.publish(String(data=msg_text))
        else:
            print(f"\033[2J\033[H", end="")
            print(f"═══ Scene State (frame {count}) ═══")
            print(msg_text)
            print(f"\n[{time.strftime('%H:%M:%S')}] Rate: {args.rate} Hz | "
                  f"Bodies: {sum(1 for b in bodies.values() if b.tracking_valid)}")

        count += 1
        elapsed = time.time() - t0
        if interval - elapsed > 0:
            time.sleep(interval - elapsed)

    if hasattr(client, "stop"):
        client.stop()
    print(f"[SceneState] Done. {count} frames published.")


# ═════════════════════════════════════════════════════════════════════════════
# Entry Point
# ═════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="OptiTrack Scene State Publisher + CS-100 Tracker",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python scene_state_publisher.py --demo                     # console, simulated
  python scene_state_publisher.py --track --demo             # visual tracker, simulated
  python scene_state_publisher.py --track --ip 192.168.0.101 # visual tracker, live
  python scene_state_publisher.py --ros                      # ROS publisher, live
        """,
    )
    parser.add_argument("--objects", default="config/objects.yaml",
                        help="Path to objects.yaml (default: config/objects.yaml)")
    parser.add_argument("--ip", default="192.168.0.101",
                        help="OptiTrack server IP (default: 192.168.0.101)")
    parser.add_argument("--rate", type=int, default=30,
                        help="Update rate in Hz (default: 30)")
    parser.add_argument("--ros", action="store_true",
                        help="Publish as ROS String messages")
    parser.add_argument("--topic", default="/llm/scene_state",
                        help="ROS topic name (default: /llm/scene_state)")
    parser.add_argument("--demo", action="store_true",
                        help="Simulated data, no OptiTrack needed")
    parser.add_argument("--track", action="store_true",
                        help="Open CS-100 2D path tracker visualization")
    parser.add_argument("--body", default="Rigid_3_Balls",
                        help="Rigid body name to track (default: Rigid_3_Balls)")
    parser.add_argument("--trail-length", type=int, default=2000,
                        help="Max trail points to keep (default: 2000)")

    args = parser.parse_args()

    if args.track:
        run_tracker(args)
    else:
        run_publisher(args)


if __name__ == "__main__":
    main()
