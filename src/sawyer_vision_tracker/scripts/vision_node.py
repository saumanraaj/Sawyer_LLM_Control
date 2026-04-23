#!/usr/bin/env python3
"""
ROS node: detects a single blue cube and publishes its pose in the robot base frame.

Subscribed topics
  /camera/color/image_raw  (sensor_msgs/Image)

Published topics
  /vision_tracker/target_pose   (geometry_msgs/PoseStamped)  — cube in base frame
  /vision_tracker/target_valid  (std_msgs/Bool)              — True when cube is seen
"""

import sys
import os

# Allow running from the workspace without a full catkin build
_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
_PKG_SRC = os.path.join(os.path.dirname(_SCRIPTS), "src")
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from sawyer_vision_tracker.detector import Detector
from sawyer_vision_tracker.tracker  import ObjectTracker
from sawyer_vision_tracker.coordinate_converter import CoordinateConverter

FONT = cv2.FONT_HERSHEY_SIMPLEX


class BlueCubeTrackerNode:

    def __init__(self):
        rospy.init_node("vision_tracker_node")

        # ── load params from parameter server (vision_tracker.yaml) ──────────
        detection_cfg = rospy.get_param("detection", {})
        tracking_cfg  = rospy.get_param("tracking",  {})
        camera_cfg    = rospy.get_param("camera",    {})

        self.detector = Detector(detection_cfg)
        self.tracker  = ObjectTracker(tracking_cfg)
        self.bridge   = CvBridge()

        # ── coordinate converter ─────────────────────────────────────────────
        intr = camera_cfg.get("intrinsics", {})
        self.converter = CoordinateConverter(
            fx=intr.get("fx", 554.38),
            fy=intr.get("fy", 554.38),
            cx=intr.get("cx", 336.5),
            cy=intr.get("cy", 188.5),
            camera_frame=camera_cfg.get("camera_frame", "front_cam_link"),
            base_frame=camera_cfg.get("base_frame",   "base"),
        )
        self.projection_mode = camera_cfg.get("projection_mode", "table_plane")
        self.fixed_z = camera_cfg.get("fixed_z", 0.70)
        self.table_z_base = camera_cfg.get("table_z_base", 0.0)
        pose_offset = camera_cfg.get("pose_offset", [0.0, 0.0, 0.0])
        if not isinstance(pose_offset, (list, tuple)) or len(pose_offset) != 3:
            rospy.logwarn(
                "[BlueCubeTracker] camera.pose_offset must be a 3-element list; using zeros."
            )
            pose_offset = [0.0, 0.0, 0.0]
        self.pose_offset = (
            float(pose_offset[0]),
            float(pose_offset[1]),
            float(pose_offset[2]),
        )

        # ── publishers ───────────────────────────────────────────────────────
        self.pose_pub  = rospy.Publisher(
            "/vision_tracker/target_pose",  PoseStamped, queue_size=1
        )
        self.valid_pub = rospy.Publisher(
            "/vision_tracker/target_valid", Bool,        queue_size=1
        )

        # ── camera subscriber ────────────────────────────────────────────────
        image_topic = camera_cfg.get("image_topic", "/camera/color/image_raw")
        rospy.Subscriber(
            image_topic, Image, self._image_cb, queue_size=1, buff_size=2 ** 24
        )
        rospy.loginfo(f"[BlueCubeTracker] Subscribed to {image_topic}")
        rospy.loginfo("[BlueCubeTracker] Ready — waiting for blue cube …")

    # ── main callback ─────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr_throttle(5.0, f"cv_bridge error: {e}")
            return

        # ── detect single blue cube ───────────────────────────────────────────
        det = self.detector.detect_blue_cube(frame)

        # Feed tracker (handles brief disappearances + EMA smoothing).
        # For control/publishing, only treat detections from the CURRENT frame
        # as valid targets; do not reuse stale tracked positions.
        tracked_list = self.tracker.update([det] if det is not None else [])
        cube = None
        if det is not None and tracked_list:
            # IMPORTANT: don't pick tracked_list[0] blindly — if the detector jumps
            # farther than max_distance, tracker can keep an older object first in
            # dict order and register a new one, making control appear "stuck".
            # Choose the tracked object that best matches current-frame detection.
            def _dist2(obj):
                dx = float(obj.raw_centroid[0]) - float(det.centroid[0])
                dy = float(obj.raw_centroid[1]) - float(det.centroid[1])
                return dx * dx + dy * dy

            visible = [obj for obj in tracked_list if obj.disappeared == 0]
            pool = visible if visible else tracked_list
            cube = min(pool, key=lambda obj: (_dist2(obj), obj.disappeared))

        # ── localise and publish ──────────────────────────────────────────────
        target_valid = False
        base_coords  = None

        if det is not None and cube is not None:
            u, v = int(cube.centroid[0]), int(cube.centroid[1])
            base_coords = self.converter.pixel_to_base(
                u,
                v,
                fixed_z=self.fixed_z,
                projection_mode=self.projection_mode,
                table_z_base=self.table_z_base,
            )

            if base_coords is not None:
                base_coords = (
                    base_coords[0] + self.pose_offset[0],
                    base_coords[1] + self.pose_offset[1],
                    base_coords[2] + self.pose_offset[2],
                )
                target_valid = True
                pose          = PoseStamped()
                pose.header.stamp    = rospy.Time.now()
                pose.header.frame_id = "base"
                pose.pose.position.x = base_coords[0]
                pose.pose.position.y = base_coords[1]
                pose.pose.position.z = base_coords[2]
                pose.pose.orientation.w = 1.0   # gripper pointing down
                self.pose_pub.publish(pose)

        self.valid_pub.publish(Bool(data=target_valid))

        dbg = self.detector.get_last_blue_debug()
        if dbg:
            cxy = dbg.get("chosen_centroid")
            cxy_txt = f"{cxy}" if cxy is not None else "None"
            rospy.loginfo_throttle(
                1.0,
                (
                    "[BlueCubeTracker][dbg] "
                    f"mask_px={dbg.get('mask_px', 0)} "
                    f"contours={dbg.get('contours_total', 0)} "
                    f"roi={dbg.get('roi_candidates', 0)} "
                    f"chosen={dbg.get('chosen', False)} "
                    f"score={dbg.get('chosen_score', float('-inf')):.2f} "
                    f"centroid={cxy_txt} "
                    f"area={dbg.get('chosen_area', 0.0):.1f} "
                    f"track_id={cube.id if cube is not None else 'None'} "
                    f"track_dis={cube.disappeared if cube is not None else -1} "
                    f"valid={target_valid}"
                ),
            )

        # ── visualise ────────────────────────────────────────────────────────
        self._draw(frame, det, cube, base_coords)

    # ── drawing ───────────────────────────────────────────────────────────────

    def _draw(self, frame, det, cube, base_coords):
        if det is not None and cube is not None:
            col  = tuple(det.color_bgr)
            x, y, w, h = det.bbox
            cx, cy = int(cube.centroid[0]), int(cube.centroid[1])

            cv2.rectangle(frame, (x, y), (x + w, y + h), col, 2)
            cv2.line(frame, (cx - 12, cy), (cx + 12, cy), col, 2)
            cv2.line(frame, (cx, cy - 12), (cx, cy + 12), col, 2)
            cv2.circle(frame, (cx, cy), 4, col, -1)

            # trajectory
            pts = list(cube.trajectory)
            for i in range(1, len(pts)):
                alpha = i / max(len(pts), 1)
                p1 = (int(pts[i - 1][0]), int(pts[i - 1][1]))
                p2 = (int(pts[i][0]),     int(pts[i][1]))
                faded = tuple(int(c * alpha) for c in col)
                cv2.line(frame, p1, p2, faded, max(1, int(alpha * 2)))

            # shape metrics
            info = [
                f"solid={det.solidity:.2f}  ar={det.aspect_ratio:.2f}",
                f"area={int(det.area)} px2",
            ]
            for k, txt in enumerate(info):
                cv2.putText(frame, txt, (x, y - 10 - k * 16),
                            FONT, 0.45, (0, 0, 0), 2)
                cv2.putText(frame, txt, (x, y - 10 - k * 16),
                            FONT, 0.45, col, 1)

        # base-frame coordinates / status
        fh = frame.shape[0]
        if base_coords is not None:
            bx, by, bz = base_coords
            txt = f"Base: ({bx:.3f}, {by:.3f}, {bz:.3f}) m"
        elif det is None or cube is None:
            txt = "Base: no detection"
        else:
            txt = "Base: no TF"
        cv2.putText(frame, txt, (10, fh - 12), FONT, 0.50, (0, 0, 0),    2)
        cv2.putText(frame, txt, (10, fh - 12), FONT, 0.50, (80, 220, 80), 1)

        # detection status
        status     = "CUBE DETECTED" if det is not None else "searching …"
        status_col = (80, 220, 80) if det is not None else (60, 60, 220)
        cv2.putText(frame, status, (10, 24), FONT, 0.60, (0, 0, 0),    2)
        cv2.putText(frame, status, (10, 24), FONT, 0.60, status_col,   1)

        cv2.imshow("Blue Cube Tracker", frame)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = BlueCubeTrackerNode()
    node.run()
