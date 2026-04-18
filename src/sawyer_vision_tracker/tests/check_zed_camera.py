#!/usr/bin/env python3
"""
ZED camera connectivity checker for sawyer_vision_tracker.

Subscribes to the camera image topic, waits for frames, and reports:
  • whether images are arriving
  • image resolution and encoding
  • measured frame rate
  • a live preview window (optional)

Usage:
    # requires roscore (and ZED node) to be running
    python3 check_zed_camera.py [--topic /camera/color/image_raw] [--timeout 10] [--show]
"""

import sys
import time
import argparse
import threading


def main():
    parser = argparse.ArgumentParser(description="ZED camera connectivity check")
    parser.add_argument("--topic",   default="/camera/color/image_raw",
                        help="ROS image topic to subscribe to")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Seconds to wait for the first frame (default: 10)")
    parser.add_argument("--show",    action="store_true",
                        help="Display a live preview window (requires display)")
    args = parser.parse_args()

    # ── ROS imports (fail fast if ROS not installed) ──────────────────────────
    try:
        import rospy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError as e:
        sys.exit(f"[ERROR] Could not import ROS packages: {e}\n"
                 "Make sure you have sourced your ROS workspace:\n"
                 "  source /opt/ros/<distro>/setup.bash\n"
                 "  source <workspace>/devel/setup.bash")

    import cv2

    bridge = CvBridge()

    # ── shared state ──────────────────────────────────────────────────────────
    state = {
        "received":    False,
        "count":       0,
        "first_time":  None,
        "last_time":   None,
        "width":       None,
        "height":      None,
        "encoding":    None,
        "lock":        threading.Lock(),
    }

    def image_callback(msg: Image):
        with state["lock"]:
            now = time.time()
            if not state["received"]:
                state["received"]   = True
                state["first_time"] = now
                state["width"]      = msg.width
                state["height"]     = msg.height
                state["encoding"]   = msg.encoding
                print(f"\n[OK] First frame received!")
                print(f"     Resolution : {msg.width} x {msg.height}")
                print(f"     Encoding   : {msg.encoding}")
                print(f"     Step       : {msg.step} bytes/row")
                print(f"     Frame ID   : {msg.header.frame_id!r}")
                print(f"     Stamp      : {msg.header.stamp.to_sec():.3f} s\n")

            state["count"]    += 1
            state["last_time"] = now

        if args.show:
            try:
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                cv2.putText(frame, f"Frame #{state['count']}",
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2)
                cv2.imshow("ZED Camera Feed", frame)
                cv2.waitKey(1)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"cv_bridge error: {e}")

    # ── init ROS node ─────────────────────────────────────────────────────────
    print(f"[INFO] Initialising ROS node …")
    try:
        rospy.init_node("check_zed_camera", anonymous=True)
    except rospy.ROSInitException as e:
        sys.exit(f"[ERROR] Could not init ROS node: {e}\n"
                 "Is roscore running?  →  roscore &")

    print(f"[INFO] Subscribing to: {args.topic}")
    rospy.Subscriber(args.topic, Image, image_callback, queue_size=1)

    # ── check what topics are actually published ───────────────────────────────
    try:
        import rostopic
        published = [t for t, _ in rospy.get_published_topics()]
        if args.topic not in published:
            print(f"\n[WARN] Topic '{args.topic}' is NOT currently published.")
            cam_topics = [t for t in published
                          if "image" in t or "camera" in t or "zed" in t]
            if cam_topics:
                print("       Camera-related topics found on the master:")
                for t in sorted(cam_topics):
                    print(f"         {t}")
            else:
                print("       No camera topics found at all — is the ZED node running?")
            print()
    except Exception:
        pass  # rostopic may not be importable in all envs

    # ── wait for first frame ──────────────────────────────────────────────────
    deadline = time.time() + args.timeout
    print(f"[INFO] Waiting up to {args.timeout:.0f} s for the first frame …")

    while not rospy.is_shutdown() and time.time() < deadline:
        with state["lock"]:
            if state["received"]:
                break
        time.sleep(0.1)

    if not state["received"]:
        print(f"\n[FAIL] No frames received within {args.timeout:.0f} s on '{args.topic}'")
        print("       Checklist:")
        print("         1. Is the ZED node running?  →  roslaunch zed_wrapper zed.launch")
        print("         2. Is the ZED USB cable plugged in?")
        print("         3. Is the correct topic published?  →  rostopic list | grep image")
        print("         4. Try a different encoding topic, e.g. /zed/zed_node/rgb/image_rect_color")
        sys.exit(1)

    # ── measure frame rate for a few seconds ──────────────────────────────────
    measure_secs = 5.0
    print(f"[INFO] Measuring frame rate for {measure_secs:.0f} s …")
    count_start = state["count"]
    t_start = time.time()

    while not rospy.is_shutdown() and (time.time() - t_start) < measure_secs:
        time.sleep(0.1)

    elapsed = time.time() - t_start
    frames  = state["count"] - count_start
    fps     = frames / elapsed if elapsed > 0 else 0.0

    print(f"\n{'='*50}")
    print(f"  ZED Camera Check — RESULTS")
    print(f"{'='*50}")
    print(f"  Topic      : {args.topic}")
    print(f"  Resolution : {state['width']} x {state['height']}")
    print(f"  Encoding   : {state['encoding']}")
    print(f"  Total frames received : {state['count']}")
    print(f"  Measured FPS          : {fps:.1f}")
    print(f"{'='*50}")

    if fps < 5:
        print(f"\n[WARN] FPS is very low ({fps:.1f}). Check ZED node settings or USB bandwidth.")
    elif fps < 15:
        print(f"\n[INFO] FPS is moderate ({fps:.1f}). Typical ZED default is 30 fps.")
    else:
        print(f"\n[OK] Camera is streaming at a healthy {fps:.1f} fps.")

    if args.show:
        print("\n[INFO] Close the preview window (press Q) to exit.")
        while not rospy.is_shutdown():
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                break
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
