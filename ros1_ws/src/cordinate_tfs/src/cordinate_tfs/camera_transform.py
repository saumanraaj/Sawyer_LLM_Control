#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class CoordinateConverter:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def convert_to_base_cordinates(self, u, v, fixed_z=0.7):
        """
        Convert pixel (u, v) assuming a flat table at z = 0.7m into base frame coordinates.
        """
        # Camera intrinsics (adjust if needed)
        fx = 554.3827128226441
        fy = 554.3827128226441
        cx = 320.5
        cy = 240.5

        # Project to 3D camera frame
        x = (u - cx) * fixed_z / fx
        y = (v - cy) * fixed_z / fy
        z = fixed_z

        pt_cam = PointStamped()
        pt_cam.header.stamp = rospy.Time.now()
        pt_cam.header.frame_id = "front_cam_link"
        pt_cam.point.x = x
        pt_cam.point.y = y
        pt_cam.point.z = z

        try:
            pt_base = self.tf_buffer.transform(pt_cam, 'base', rospy.Duration(1.0))
            return pt_base.point.x, pt_base.point.y, pt_base.point.z
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            rospy.logerr(f"[CoordinateConverter] TF transform failed: {e}")
            return None