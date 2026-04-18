#!/usr/bin/env python3
"""
run_3d_trace.py — Real-Time 3D Trace Renderer for OptiTrack Rigid Bodies

Writers: Nguyen Nguyen (Alan), Sauman Raaj

Plots the live 3D trajectory of one or more tracked rigid bodies
using matplotlib's 3D axes. Each body gets a distinct trail color.
The trace shows each object's current position as a highlighted sphere.

Coordinate System
-----------------
Uses raw Y-up values from Motive by default (convert_to_zup=False)
so you see exactly what OptiTrack reports:
    X = right,  Y = up (height),  Z = toward cameras

Use --zup to convert to robotics Z-up convention.

Usage
-----
    # Live, track rigid_body_12:
    python run_3d_trace.py --body rigid_body_12 --ip 192.168.0.101

    # Track multiple bodies:
    python run_3d_trace.py --body rigid_body_12 rigid_body_13

    # Track all discovered bodies:
    python run_3d_trace.py --all-bodies --ip 192.168.0.101

    # Demo mode (simulated data, no hardware):
    python run_3d_trace.py --demo

    # With Z-up conversion:
    python run_3d_trace.py --body rigid_body_12 --ip 192.168.0.101 --zup

    # Longer trail, slower update:
    python run_3d_trace.py --demo --trail 5000 --rate 15
"""

import sys
import time
import math
import argparse
import numpy as np
from collections import deque
from pathlib import Path

# Resolve imports from parent directory
_script_dir = Path(__file__).resolve().parent
_engine_dir = _script_dir.parent
if str(_engine_dir) not in sys.path:
    sys.path.insert(0, str(_engine_dir))

from cv.optitrack_client import OptiTrackClient, RigidBodyState

# Color palette: (trail_color, dot_color) per body
BODY_COLORS = [
    ("#ff6b6b", "#ffd43b"),  # red trail, yellow dot
    ("#4dabf7", "#69db7c"),  # blue trail, green dot
    ("#da77f2", "#ffa94d"),  # purple trail, orange dot
    ("#38d9a9", "#ff8787"),  # teal trail, pink dot
    ("#ffd43b", "#4dabf7"),  # yellow trail, blue dot
    ("#69db7c", "#da77f2"),  # green trail, purple dot
]


# --- Demo Data Generator ---

class DemoOptiTrack3D:
    """Simulates rigid bodies tracing 3D paths for demo/testing."""

    def __init__(self, body_names=None):
        self._t0 = time.time()
        self._body_names = body_names or ["rigid_body_12"]

    def get_rigid_bodies(self):
        t = time.time() - self._t0
        now = time.time()
        bodies = {}

        for i, name in enumerate(self._body_names):
            # Each body gets a different phase offset and radius
            phase = i * 2 * math.pi / max(len(self._body_names), 1)
            radius = 0.20 + i * 0.05
            speed = 0.5 - i * 0.1

            x = radius * math.cos(t * speed + phase)
            z = radius * math.sin(t * speed + phase)
            y = 0.8 + 0.05 * math.sin(t * 0.15 + phase)

            # Add some noise to make it realistic
            x += 0.002 * math.sin(t * 7.3 + i)
            z += 0.002 * math.cos(t * 5.1 + i)

            qw = math.cos(t * speed / 2)
            qy = math.sin(t * speed / 2)

            bodies[name] = RigidBodyState(
                name=name, id=12 + i,
                position=np.array([x, y, z]),
                quaternion=np.array([0.0, qy, 0.0, qw]),
                timestamp=now, tracking_valid=True,
            )

        return bodies

    def get_frame_count(self):
        return int((time.time() - self._t0) * 120)

    def start(self):
        pass

    def stop(self):
        pass


# --- Main 3D Trace Renderer ---

def run_3d_trace(args):
    """Real-time 3D trace of rigid bodies using matplotlib."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    BODY_NAMES = args.body  # list of names, or None for auto
    TRACK_ALL = args.all_bodies or (args.demo and args.body is None)
    TRAIL_MAX = args.trail

    if BODY_NAMES is None:
        BODY_NAMES = ["rigid_body_12"]

    # Connect
    if args.demo:
        demo_bodies = ["rigid_body_12", "rigid_body_13"]
        print(f"[3D Trace] Demo mode — simulated paths for {demo_bodies}")
        client = DemoOptiTrack3D(body_names=demo_bodies)
        if TRACK_ALL:
            print(f"[3D Trace] Tracking all discovered bodies")
        else:
            print(f"[3D Trace] Filtering to: {BODY_NAMES}")
    else:
        convert = args.zup
        print(f"[3D Trace] Connecting to OptiTrack at {args.ip} "
              f"({'Z-up' if convert else 'raw Y-up'})...")
        client = OptiTrackClient(server_ip=args.ip, convert_to_zup=convert)
        client.start()

    # Per-body storage: {name: {xs, ys, zs, trail_line, current_dot, ghost_dot}}
    tracked = {}
    color_counter = [0]

    t0 = time.time()
    last_print = [0.0]

    # Set up figure
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection='3d')

    coord_label = "Z-up" if args.zup else "Y-up (raw)"
    if TRACK_ALL:
        title_label = "all bodies"
    elif len(BODY_NAMES) == 1:
        title_label = f"'{BODY_NAMES[0]}'"
    else:
        title_label = ", ".join(BODY_NAMES)
    ax.set_title(f"3D Trace — {title_label}  [{coord_label}]",
                 fontsize=13, fontweight="bold", pad=15)

    if args.zup:
        ax.set_xlabel("X (m)", fontsize=9)
        ax.set_ylabel("Y (m)", fontsize=9)
        ax.set_zlabel("Z (m) ↑ UP", fontsize=9)
    else:
        ax.set_xlabel("X (m) ← right →", fontsize=9)
        ax.set_ylabel("Y (m) ↑ UP", fontsize=9)
        ax.set_zlabel("Z (m) ← fwd →", fontsize=9)

    ax.tick_params(labelsize=7)

    # Status text
    status_text = fig.text(0.02, 0.02, "Waiting for bodies...",
                           fontsize=8, color="#aaaaaa", family="monospace")
    pos_text = fig.text(0.02, 0.05, "",
                        fontsize=9, color="#51cf66", family="monospace")

    # For auto-scaling
    range_pad = 0.05

    def _add_body(name):
        """Create plot artists and data buffers for a newly discovered body."""
        ci = color_counter[0] % len(BODY_COLORS)
        color_counter[0] += 1
        trail_c, dot_c = BODY_COLORS[ci]

        trail_line, = ax.plot([], [], [], color=trail_c, linewidth=1.0,
                              alpha=0.6, label=name)
        current_dot, = ax.plot([], [], [], 'o', color=dot_c, markersize=10,
                               markeredgecolor="white", markeredgewidth=1.5)
        ghost_dot, = ax.plot([], [], [], 'o', color=trail_c, markersize=5,
                             alpha=0.3)

        tracked[name] = {
            "xs": deque(maxlen=TRAIL_MAX),
            "ys": deque(maxlen=TRAIL_MAX),
            "zs": deque(maxlen=TRAIL_MAX),
            "trail_line": trail_line,
            "current_dot": current_dot,
            "ghost_dot": ghost_dot,
        }
        # Update legend when a new body appears
        if len(tracked) > 1:
            ax.legend(loc="upper right", fontsize=8, framealpha=0.5)

        return tracked[name]

    def update(frame_num):
        t_now = time.time() - t0
        bodies = client.get_rigid_bodies()

        # Determine which bodies to process this frame
        if TRACK_ALL:
            names_to_track = list(bodies.keys())
        else:
            names_to_track = BODY_NAMES

        any_data = False
        primary_info = ""

        for bname in names_to_track:
            body = bodies.get(bname)
            if body is None:
                continue

            pos = body.position
            if not (np.all(np.isfinite(pos)) and np.linalg.norm(pos) < 50.0):
                continue

            # Auto-register new body on first sighting
            if bname not in tracked:
                _add_body(bname)

            bd = tracked[bname]
            px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
            bd["xs"].append(px)
            bd["ys"].append(py)
            bd["zs"].append(pz)
            any_data = True

            # Update trail line
            x_arr = list(bd["xs"])
            y_arr = list(bd["ys"])
            z_arr = list(bd["zs"])
            bd["trail_line"].set_data(x_arr, y_arr)
            bd["trail_line"].set_3d_properties(z_arr)

            # Current position (bright dot)
            bd["current_dot"].set_data([px], [py])
            bd["current_dot"].set_3d_properties([pz])

            # Ghost dot at first point
            if len(bd["xs"]) > 1:
                bd["ghost_dot"].set_data([x_arr[0]], [y_arr[0]])
                bd["ghost_dot"].set_3d_properties([z_arr[0]])

            # Position info for the first body (shown in HUD)
            if not primary_info:
                q = body.quaternion
                primary_info = (
                    f"{bname}: pos=({px:+.4f}, {py:+.4f}, {pz:+.4f})  "
                    f"quat=({q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}, {q[3]:+.3f})"
                )

        # Auto-scale axes across all tracked bodies
        if any_data and tracked:
            all_x, all_y, all_z = [], [], []
            for bd in tracked.values():
                all_x.extend(bd["xs"])
                all_y.extend(bd["ys"])
                all_z.extend(bd["zs"])

            if len(all_x) > 2:
                x_min, x_max = min(all_x), max(all_x)
                y_min, y_max = min(all_y), max(all_y)
                z_min, z_max = min(all_z), max(all_z)

                # Ensure minimum range so axes don't collapse
                for arr_min, arr_max in [(x_min, x_max), (y_min, y_max), (z_min, z_max)]:
                    if arr_max - arr_min < 0.02:
                        mid = (arr_max + arr_min) / 2
                        arr_min = mid - 0.01
                        arr_max = mid + 0.01

                x_span = x_max - x_min
                y_span = y_max - y_min
                z_span = z_max - z_min

                ax.set_xlim(x_min - range_pad * x_span, x_max + range_pad * x_span)
                ax.set_ylim(y_min - range_pad * y_span, y_max + range_pad * y_span)
                ax.set_zlim(z_min - range_pad * z_span, z_max + range_pad * z_span)

        # Status text
        if any_data:
            total_pts = sum(len(bd["xs"]) for bd in tracked.values())
            n = len(tracked)
            pos_text.set_text(primary_info)
            status_text.set_text(
                f"TRACKING {n} bod{'y' if n == 1 else 'ies'}  |  "
                f"{total_pts} pts  |  {t_now:.1f}s"
            )
            status_text.set_color("#51cf66")

            # Terminal print every 1s
            if t_now - last_print[0] >= 1.0:
                last_print[0] = t_now
                for bname, bd in tracked.items():
                    if bd["xs"]:
                        px = bd["xs"][-1]
                        py = bd["ys"][-1]
                        pz = bd["zs"][-1]
                        print(f"[{t_now:6.1f}s] {bname}: "
                              f"pos=({px:+.5f}, {py:+.5f}, {pz:+.5f})  "
                              f"|  {len(bd['xs'])} trail pts")
        else:
            names = list(bodies.keys()) if bodies else ["(none)"]
            if TRACK_ALL:
                status_text.set_text(
                    f"NO BODIES FOUND  |  "
                    f"Available: {', '.join(names)}  |  {t_now:.1f}s"
                )
            else:
                missing = [n for n in BODY_NAMES if n not in bodies]
                status_text.set_text(
                    f"MISSING: {', '.join(missing)}  |  "
                    f"Available: {', '.join(names)}  |  {t_now:.1f}s"
                )
            status_text.set_color("#ffd43b")

            if t_now - last_print[0] >= 2.0:
                last_print[0] = t_now
                print(f"[{t_now:6.1f}s] Waiting... "
                      f"Available: {', '.join(names)}")

        # Return all artists
        artists = [status_text, pos_text]
        for bd in tracked.values():
            artists.extend([bd["trail_line"], bd["current_dot"], bd["ghost_dot"]])
        return artists

    # Run animation
    interval_ms = max(16, int(1000 / args.rate))
    anim = FuncAnimation(fig, update, interval=interval_ms,
                         blit=False, cache_frame_data=False)

    mode_label = "all discovered" if TRACK_ALL else ", ".join(BODY_NAMES)
    print(f"[3D Trace] Tracking: {mode_label} at {args.rate} Hz")
    print(f"[3D Trace] Trail buffer: {TRAIL_MAX} points per body")
    print(f"[3D Trace] Close matplotlib window to stop.\n")

    try:
        plt.show()
    except KeyboardInterrupt:
        pass

    if hasattr(client, "stop"):
        client.stop()
    total = sum(len(bd["xs"]) for bd in tracked.values())
    print(f"[3D Trace] Done. {len(tracked)} bodies, {total} total trail points.")


# --- Entry Point ---

def main():
    parser = argparse.ArgumentParser(
        description="Real-time 3D trace of OptiTrack rigid bodies",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_3d_trace.py --demo                                   # simulated (2 bodies)
  python run_3d_trace.py --body rigid_body_12 --ip 192.168.0.101  # single body
  python run_3d_trace.py --body rigid_body_12 rigid_body_13       # two specific bodies
  python run_3d_trace.py --all-bodies --ip 192.168.0.101          # all discovered
  python run_3d_trace.py --body rigid_body_12 --ip 192.168.0.101 --zup  # Z-up
  python run_3d_trace.py --demo --trail 5000 --rate 15            # longer trail
        """,
    )
    parser.add_argument("--body", nargs='+', default=None,
                        help="Rigid body name(s) to track (default: rigid_body_12)")
    parser.add_argument("--all-bodies", action="store_true",
                        help="Track all discovered rigid bodies")
    parser.add_argument("--ip", default="192.168.0.101",
                        help="OptiTrack server IP (default: 192.168.0.101)")
    parser.add_argument("--rate", type=int, default=30,
                        help="Update rate in Hz (default: 30)")
    parser.add_argument("--trail", type=int, default=3000,
                        help="Max trail points per body (default: 3000)")
    parser.add_argument("--demo", action="store_true",
                        help="Simulated data, no OptiTrack needed")
    parser.add_argument("--zup", action="store_true",
                        help="Convert to Z-up (robotics convention). "
                             "Default: raw Y-up from Motive.")

    args = parser.parse_args()
    run_3d_trace(args)


if __name__ == "__main__":
    main()
