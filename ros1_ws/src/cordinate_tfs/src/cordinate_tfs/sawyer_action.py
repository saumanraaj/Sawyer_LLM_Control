import geometry_msgs.msg
import moveit_commander
from sensor_msgs.msg import JointState
import rospy
from moveit_msgs.msg import MoveItErrorCodes
import numpy as np
import tf.transformations as tf

class SawyerActions():
    def __init__(self):
        self.latest_joint_state_time = None
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        print("[INFO] Initialized MoveGroupCommander for 'right_arm'")

        self.gripper = None
        try:
            from intera_interface import Gripper
            self.gripper = Gripper('right_gripper')
            if not self.gripper.is_calibrated():
                self.gripper.calibrate()
                rospy.sleep(1.0)
            print("[INFO] Gripper initialized successfully.")
        except Exception as e:
            print(f"[WARN] No physical gripper detected. Proceeding without gripper control.")

        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        self.latest_joint_state_time = msg.header.stamp

    def wait_for_fresh_joint_state(self, timeout_sec=10.0):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout_sec:
            if self.latest_joint_state_time and (rospy.Time.now() - self.latest_joint_state_time).to_sec() < 1.0:
                return True
            rospy.sleep(0.1)
        print("[WARN] Timed out waiting for fresh /joint_states.")
        return False

    def move_to(self, x, y, z):
        if not self.wait_for_fresh_joint_state():
            print("[ERROR] No fresh joint state. Aborting move.")
            return

        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = np.clip(x, 0.4, 1.0)
        pose_target.position.y = np.clip(y, -0.6, 0.6)
        pose_target.position.z = max(z, 0.1)

        quat = tf.quaternion_from_euler(0, np.pi, 0)
        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]

        self.group.set_pose_target(pose_target)
        success, motion_plan, planning_time, error_code = self.group.plan()

        if success and motion_plan and len(motion_plan.joint_trajectory.points) > 0:
            self.group.execute(motion_plan, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            print(f"[INFO] Moved to x={x:.3f}, y={y:.3f}, z={z:.3f}")
        else:
            print("[ERROR] Motion planning failed!")

    def close_gripper(self):
        if self.gripper:
            try:
                self.gripper.close()
                rospy.sleep(2.0)
                print("[INFO] Gripper closed.")
            except Exception as e:
                print(f"[WARN] Failed to close gripper: {e}")
        else:
            print("[INFO] Skipping gripper close (no gripper present).")

    def open_gripper(self):
        if self.gripper:
            try:
                self.gripper.open()
                rospy.sleep(2.0)
                print("[INFO] Gripper opened.")
            except Exception as e:
                print(f"[WARN] Failed to open gripper: {e}")
        else:
            print("[INFO] Skipping gripper open (no gripper present).")

    def lift(self, delta_z):
        if not self.wait_for_fresh_joint_state():
            print("[ERROR] No fresh joint state. Aborting lift.")
            return

        current_pose = self.group.get_current_pose().pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = current_pose.position.x
        target_pose.position.y = current_pose.position.y
        target_pose.position.z = current_pose.position.z + delta_z
        target_pose.orientation = current_pose.orientation

        self.group.set_pose_target(target_pose)
        success, motion_plan, planning_time, error_code = self.group.plan()

        if success and motion_plan and len(motion_plan.joint_trajectory.points) > 0:
            self.group.execute(motion_plan, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            print(f"[INFO] Lifted by {delta_z:.2f} meters.")
        else:
            print("[ERROR] Lift planning failed!")