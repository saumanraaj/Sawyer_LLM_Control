#!/usr/bin/env python

import rospy
from sawyer_action import SawyerActions

if __name__ == "__main__":
    rospy.init_node("manual_pickup_node", anonymous=True)

    robot = SawyerActions()

    rospy.sleep(1.0)

    # 1. Move above the object (adjust x, y, z as per your box location)
    target_x = 0.79  # or adjust manually
    target_y = 0.065
    hover_z = 0.82  # hover height above the table
    pickup_z = 0.78   # lower height near object

    print("[INFO] Moving to hover position...")
    robot.move_to(x=target_x, y=target_y, z=hover_z)

    rospy.sleep(2.0)

    print("[INFO] Moving down to grasp...")
    robot.move_to(x=target_x, y=target_y, z=pickup_z)

    rospy.sleep(1.0)

    print("[INFO] Closing gripper (if available)...")
    robot.close_gripper()

    rospy.sleep(1.0)

    print("[INFO] Lifting object...")
    robot.lift(delta_z=0.2)

    print("[INFO] Pickup sequence complete.")