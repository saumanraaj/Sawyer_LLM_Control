#!/usr/bin/env python3
"""Pixel-to-robot-base-frame coordinate converter using TF2."""

import rospy
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs


class CoordinateConverter:
    """
    Convert pixel (u, v) to robot base frame coordinates.

    Assumes a flat table at a fixed Z in the camera frame,
    projects the pixel into 3D in the camera frame, then
    uses TF2 to transform to the robot base frame.
    """

    def __init__(self, fx, fy, cx, cy, camera_frame, base_frame):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.camera_frame = camera_frame
        self.base_frame = base_frame

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def pixel_to_base(self, u, v, fixed_z=0.7):
        """
        Convert pixel (u, v) to (x, y, z) in the robot base frame.

        Args:
            u: pixel column
            v: pixel row
            fixed_z: assumed depth in camera frame (table height)

        Returns:
            (x, y, z) in base frame, or None if TF lookup fails.
        """
        # project pixel to 3D point in camera frame
        x_cam = (u - self.cx) * fixed_z / self.fx
        y_cam = (v - self.cy) * fixed_z / self.fy
        z_cam = fixed_z

        pt_cam = PointStamped()
        pt_cam.header.stamp = rospy.Time.now()
        pt_cam.header.frame_id = self.camera_frame
        pt_cam.point.x = x_cam
        pt_cam.point.y = y_cam
        pt_cam.point.z = z_cam

        try:
            pt_base = self.tf_buffer.transform(
                pt_cam, self.base_frame, rospy.Duration(1.0)
            )
            return (
                pt_base.point.x,
                pt_base.point.y,
                pt_base.point.z,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ) as e:
            rospy.logwarn_throttle(
                5.0, f"[CoordinateConverter] TF transform failed: {e}"
            )
            return None
