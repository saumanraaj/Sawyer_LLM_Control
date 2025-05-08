#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import moveit_commander
from sensor_msgs.msg import JointState
from intera_interface import Gripper

class sawyer_actions():

    def __init__(self):
        rospy.loginfo("Initializing Sawyer Actions...")
        self.latest_joint_state_time = None
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.group.set_planning_time(10.0)

        try:
            self.gripper = Gripper('right_gripper')
            if not self.gripper.is_ready():
                raise Exception("Gripper not ready")
            self.has_gripper = True
            rospy.loginfo("Gripper detected and ready.")
        except Exception as e:
            self.has_gripper = False
            rospy.logwarn(f"No gripper detected: {e}")

        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        rospy.loginfo("Sawyer Actions Initialized.")

    def joint_state_callback(self, msg):
        self.latest_joint_state_time = msg.header.stamp

    def wait_for_fresh_joint_state(self, threshold_sec=2.0, timeout_sec=10.0):
        rospy.loginfo("Waiting for fresh /joint_states...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self.latest_joint_state_time is not None:
                delta = (now - self.latest_joint_state_time).to_sec()
                if delta < threshold_sec:
                    rospy.loginfo("Fresh joint state received.")
                    return True

            if (now - start_time).to_sec() > timeout_sec:
                rospy.logwarn("Timed out waiting for fresh /joint_states after {:.1f} seconds.".format(timeout_sec))
                return False

            rate.sleep()

    def move_to(self, x, y, z=0.6):
        if not self.wait_for_fresh_joint_state():
            rospy.logwarn("Skipping move due to stale joint states.")
            return False

        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose)

        rospy.loginfo(f"Planning move_to(x={x:.3f}, y={y:.3f}, z={z:.3f})")

        plan = self.group.plan()

        if isinstance(plan, tuple):
            plan = plan[1]

        if hasattr(plan, "joint_trajectory") and len(plan.joint_trajectory.points) > 0:
            rospy.loginfo("Executing planned motion.")
            self.group.go(wait=True)
            return True
        else:
            rospy.logerr("Planning failed for move_to command.")
            return False

    def close_gripper(self):
        if not self.has_gripper:
            rospy.logwarn("Skipping close_gripper: No gripper attached.")
            return
        rospy.loginfo("Closing gripper...")
        try:
            if not self.gripper.is_calibrated():
                rospy.loginfo("Calibrating gripper...")
                self.gripper.calibrate()
                rospy.sleep(1.0)
            self.gripper.close()
            rospy.sleep(2.0)
        except Exception as e:
            rospy.logerr(f"Gripper error: {e}")

    def open_gripper(self):
        if not self.has_gripper:
            rospy.logwarn("Skipping open_gripper: No gripper attached.")
            return
        rospy.loginfo("Opening gripper...")
        try:
            self.gripper.open()
            rospy.sleep(2.0)
        except Exception as e:
            rospy.logerr(f"Gripper error: {e}")

    def lift(self, delta_z=0.1):
        if not self.wait_for_fresh_joint_state():
            rospy.logwarn("Skipping lift due to stale joint states.")
            return False

        current_pose = self.group.get_current_pose().pose
        current_pose.position.z += delta_z

        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(current_pose)

        rospy.loginfo(f"Lifting end-effector by {delta_z:.3f} meters.")

        plan = self.group.plan()

        if isinstance(plan, tuple):
            plan = plan[1]

        if hasattr(plan, "joint_trajectory") and len(plan.joint_trajectory.points) > 0:
            rospy.loginfo("Executing lift motion.")
            self.group.go(wait=True)
            return True
        else:
            rospy.logerr("Planning failed for lift command.")
            return False
