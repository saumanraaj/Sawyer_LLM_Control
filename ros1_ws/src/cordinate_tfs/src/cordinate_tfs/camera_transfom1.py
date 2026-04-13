#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs

class CoordinateConverter:
    def __init__(self):
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def convert_to_base_cordinates(self, u, v, z):
        # Validate depth
        if z <= 0.0 or np.isnan(z):
            rospy.logwarn(f"[CoordinateConverter] Invalid depth at pixel ({u},{v}): {z}. Cannot convert.")
            return None

        # Camera Intrinsics (hardcoded for now)
        K = [554.3827128226441, 0.0, 320.5, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 1.0]
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # Back-project pixel to 3D camera frame
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        pt_cam = PointStamped()
        pt_cam.header.stamp = rospy.Time.now()
        pt_cam.header.frame_id = "front_cam_link"  # Match your camera frame
        pt_cam.point.x = x
        pt_cam.point.y = y
        pt_cam.point.z = z

        try:
            pt_base = self.tf_buffer.transform(pt_cam, 'base', rospy.Duration(1.0))
            return pt_base.point.x, pt_base.point.y, pt_base.point.z
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logerr(f"[CoordinateConverter] TF transform failed: {e}")
            return None

if __name__ == '__main__':
    try:
        CoordinateConverter()
    except rospy.ROSInterruptException:
        pass