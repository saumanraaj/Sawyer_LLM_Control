#!/usr/bin/env python3
"""
Minimal ZED camera publisher (no SDK required).

The ZED exposes a side-by-side stereo image over UVC.  This node reads it,
splits out the left eye, and publishes on /camera/color/image_raw so that
the vision_tracker pipeline can consume it without the proprietary SDK.

Usage:
    rosrun sawyer_vision_tracker zed_raw_publisher.py
    # or
    python3 zed_raw_publisher.py
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

DEVICE   = 0                          # /dev/video0
TOPIC    = "/camera/color/image_raw"
INFO_T   = "/camera/color/camera_info"
FRAME_ID = "front_cam_link"
TARGET_FPS = 30


def build_camera_info(w, h):
    """Fill a CameraInfo msg using the intrinsics from vision_tracker.yaml."""
    msg = CameraInfo()
    msg.width  = w
    msg.height = h
    fx = fy = 554.3827128226441
    cx = w / 2.0 + 0.5
    cy = h / 2.0 + 0.5
    msg.distortion_model = "plumb_bob"
    msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.K = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]
    msg.R = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.P = [fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
    return msg


def main():
    rospy.init_node("zed_raw_publisher")

    bridge   = CvBridge()
    img_pub  = rospy.Publisher(TOPIC,  Image,      queue_size=1)
    info_pub = rospy.Publisher(INFO_T, CameraInfo, queue_size=1)

    cap = cv2.VideoCapture(DEVICE)
    if not cap.isOpened():
        rospy.logfatal(f"Cannot open /dev/video{DEVICE}. Is the ZED plugged in?")
        return

    # Read one frame to learn the side-by-side resolution
    ok, probe = cap.read()
    if not ok:
        rospy.logfatal("Could not read from ZED camera.")
        cap.release()
        return

    full_h, full_w = probe.shape[:2]
    left_w  = full_w // 2          # ZED side-by-side: left eye is left half
    left_h  = full_h

    rospy.loginfo(
        f"ZED opened: full frame {full_w}x{full_h}, "
        f"publishing left eye {left_w}x{left_h} → {TOPIC}"
    )

    cam_info = build_camera_info(left_w, left_h)
    rate = rospy.Rate(TARGET_FPS)

    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok:
            rospy.logwarn_throttle(5.0, "ZED read failed, retrying …")
            continue

        # Crop left eye
        left = frame[:, :left_w, :]

        now = rospy.Time.now()

        img_msg = bridge.cv2_to_imgmsg(left, encoding="bgr8")
        img_msg.header.stamp    = now
        img_msg.header.frame_id = FRAME_ID

        cam_info.header.stamp    = now
        cam_info.header.frame_id = FRAME_ID

        img_pub.publish(img_msg)
        info_pub.publish(cam_info)

        rate.sleep()

    cap.release()
    rospy.loginfo("ZED publisher shut down.")


if __name__ == "__main__":
    main()
