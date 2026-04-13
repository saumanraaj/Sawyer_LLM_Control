import rospy
import numpy as np
import geometry_msgs.msg
import moveit_commander
from intera_interface import Gripper, Limb
from sensor_msgs.msg import JointState
from camera_transform import CoordinateConverter
from moveit_msgs.msg import MoveItErrorCodes
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class sawyer_actions():

    def __init__(self):
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        rospy.loginfo("Initialized MoveGroupCommander for 'right_arm'")

        try:
            self.gripper = Gripper('right_gripper')
            rospy.loginfo("Gripper initialized successfully.")
        except Exception as e:
            rospy.logwarn(f"Gripper not available: {e}")
            self.gripper = None

        self.camera_fns = CoordinateConverter()

        self.latest_joint_state_time = None
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        self.latest_joint_state_time = msg.header.stamp

    def wait_for_fresh_joint_state(self, threshold_sec=2.0, timeout_sec=10.0):
        rospy.loginfo("Waiting for fresh /joint_states...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.latest_joint_state_time and (now - self.latest_joint_state_time).to_sec() < threshold_sec:
                rospy.loginfo("Fresh joint state received.")
                return True
            if (now - start_time).to_sec() > timeout_sec:
                rospy.logwarn("Timed out waiting for fresh /joint_states.")
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

        # Set gripper facing straight down
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

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
        if self.gripper is None:
            rospy.logwarn("No gripper available. Skipping close_gripper.")
            return
        rospy.loginfo("Closing electric gripper...")
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
        if self.gripper is None:
            rospy.logwarn("No gripper available. Skipping open_gripper.")
            return
        rospy.loginfo("Opening electric gripper...")
        try:
            self.gripper.open()
            rospy.sleep(2.0)
        except Exception as e:
            rospy.logerr(f"Gripper error: {e}")
    def print_current_pose(self):
        """
        Print the current end-effector pose (x, y, z) in the base frame.
        """
        current_pose = self.group.get_current_pose().pose
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z
        rospy.loginfo(f"Current end-effector pose: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    def lift(self, delta_z):
        if not self.wait_for_fresh_joint_state():
            rospy.logwarn("Skipping lift due to stale joint states.")
            return

        self.group.set_start_state_to_current_state()
        current_pose = self.group.get_current_pose().pose
        current_pose.position.z += delta_z
        self.group.set_pose_target(current_pose)

        rospy.loginfo(f"Planning lift by delta_z={delta_z:.3f} meters")
        result = self.group.plan()
        plan = result[1] if isinstance(result, tuple) else result

        if hasattr(plan, "joint_trajectory") and len(plan.joint_trajectory.points) > 0:
            rospy.loginfo("Executing lift motion.")
            self.group.execute(plan, wait=True)
        else:
            rospy.logerr("Planning failed for lift command.")

    def execute_sequence(self, data, depth_image):
        actions = data['actions']
        for action in actions:
            rospy.loginfo(f"Executing action: {action}")

            if action.startswith("move_to"):
                x_pix, y_pix = data["position"]

                # Skip depth reading entirely
                fixed_z = 0.7  # Safe table height assumption

                point = self.camera_fns.convert_to_base_cordinates(x_pix, y_pix, fixed_z)
                if point is None:
                    rospy.logwarn("Skipping move_to due to invalid coordinate conversion.")
                    continue

                x, y, z = point

                # Safety check for z (should not be below ground)
                if z < 0.1:
                    rospy.logwarn(f"Transformed z={z:.3f} too low. Resetting to safe height 0.7m.")
                    z = 0.7

                # Clamp workspace (to safe limits)
                max_reach = 0.8
                x = np.clip(x, -max_reach, max_reach)
                y = np.clip(y, -max_reach, max_reach)

                success = self.move_to(x, y, z)
                if not success:
                    rospy.logerr("Aborting remaining actions due to failed move_to().")
                    return

            elif action.startswith("close_gripper"):
                self.close_gripper()

            elif action.startswith("open_gripper"):
                self.open_gripper()

            elif action.startswith("lift"):
                self.lift(0.2)