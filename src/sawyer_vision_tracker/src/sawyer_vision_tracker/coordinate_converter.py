#!/usr/bin/env python3
"""Pixel-to-robot-base-frame coordinate converter using TF2."""

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix


class CoordinateConverter:
    """
    Convert pixel (u, v) to robot base frame coordinates.

    Supports two projection modes:
      1) fixed depth in camera frame (legacy)
      2) ray-plane intersection in base frame (preferred)
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

    def _lookup_base_from_camera(self):
        return self.tf_buffer.lookup_transform(
            self.base_frame, self.camera_frame, rospy.Time(0), rospy.Duration(1.0)
        )

    def pixel_to_base_fixed_depth(self, u, v, fixed_z=0.7):
        """
        Legacy conversion: assumes known depth along camera Z axis.

        Args:
            u: pixel column
            v: pixel row
            fixed_z: assumed depth in camera frame (metres)

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

    def pixel_to_base_on_table(self, u, v, table_z_base=0.0):
        """
        Preferred conversion: intersect camera pixel ray with table plane z=const in base.
        """
        try:
            tfm = self._lookup_base_from_camera()
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ) as e:
            rospy.logwarn_throttle(
                5.0, f"[CoordinateConverter] TF transform failed: {e}"
            )
            return None

        tx = tfm.transform.translation.x
        ty = tfm.transform.translation.y
        tz = tfm.transform.translation.z
        origin_base = np.array([tx, ty, tz], dtype=np.float64)

        qx = tfm.transform.rotation.x
        qy = tfm.transform.rotation.y
        qz = tfm.transform.rotation.z
        qw = tfm.transform.rotation.w
        rot_base_from_cam = quaternion_matrix([qx, qy, qz, qw])[:3, :3]

        # Camera-frame ray. Some setups publish camera frames with opposite
        # optical-axis sign conventions, so evaluate both +/-Z directions.
        xn = (float(u) - self.cx) / self.fx
        yn = (float(v) - self.cy) / self.fy
        ray_candidates_cam = (
            np.array([xn, yn, 1.0], dtype=np.float64),
            np.array([xn, yn, -1.0], dtype=np.float64),
        )

        best_point = None
        best_t = None
        saw_parallel = False
        for ray_cam in ray_candidates_cam:
            ray_base = rot_base_from_cam.dot(ray_cam)
            dz = ray_base[2]
            if abs(dz) < 1e-6:
                saw_parallel = True
                continue

            t = (float(table_z_base) - origin_base[2]) / dz
            if t <= 0:
                continue

            p_base = origin_base + t * ray_base
            if best_t is None or t < best_t:
                best_t = t
                best_point = p_base

        if best_point is not None:
            return (
                float(best_point[0]),
                float(best_point[1]),
                float(best_point[2]),
            )

        if saw_parallel:
            rospy.logwarn_throttle(
                5.0, "[CoordinateConverter] Pixel ray parallel to table plane."
            )
        else:
            rospy.logwarn_throttle(
                5.0,
                "[CoordinateConverter] Pixel ray does not intersect table plane in front of camera.",
            )
        return None

    def pixel_to_base(self, u, v, fixed_z=0.7, projection_mode="table_plane", table_z_base=0.0):
        """
        Convert pixel (u, v) to (x, y, z) in base frame.
        """
        if projection_mode == "fixed_depth":
            return self.pixel_to_base_fixed_depth(u, v, fixed_z=fixed_z)
        return self.pixel_to_base_on_table(u, v, table_z_base=table_z_base)
