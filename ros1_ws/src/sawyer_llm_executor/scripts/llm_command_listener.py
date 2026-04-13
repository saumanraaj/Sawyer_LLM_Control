#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import moveit_commander
import json
import os
from sensor_msgs.msg import JointState
from intera_interface import Gripper

latest_joint_state_time = None

# ----------- Joint State Freshness Tracker -----------

def joint_state_callback(msg):
    global latest_joint_state_time
    latest_joint_state_time = msg.header.stamp

def wait_for_fresh_joint_state(threshold_sec=0.5, timeout_sec=5.0):
    rospy.loginfo("Waiting for fresh /joint_states...")
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        if latest_joint_state_time is not None:
            delta = (now - latest_joint_state_time).to_sec()
            if delta < threshold_sec:
                rospy.loginfo("Fresh joint state received.")
                return True

        if (now - start_time).to_sec() > timeout_sec:
            rospy.logwarn("Timed out waiting for fresh /joint_states.")
            return False

        rate.sleep()

# ----------- Action Functions -----------

def move_to(x, y, z):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    group.set_pose_target(pose)
    group.set_start_state_to_current_state()

    result = group.plan()
    plan = result[1] if isinstance(result, tuple) else result

    if hasattr(plan, "joint_trajectory") and len(plan.joint_trajectory.points) > 0:
        rospy.loginfo(f"Planned joint values: {plan.joint_trajectory.points[-1].positions}")
        group.execute(plan, wait=True)
        return True
    else:
        rospy.logerr("Planning failed for move_to command")
        return False

def close_gripper():
    rospy.loginfo("Closing electric gripper...")
    try:
        gripper = Gripper('right_gripper')
        if not gripper.is_calibrated():
            rospy.loginfo("Calibrating gripper...")
            gripper.calibrate()
            rospy.sleep(1.0)
        gripper.close()
        rospy.sleep(2.0)
    except Exception as e:
        rospy.logerr(f"Gripper error: {e}")

def open_gripper():
    rospy.loginfo("Opening electric gripper...")
    try:
        gripper = Gripper('right_gripper')
        gripper.open()
        rospy.sleep(2.0)
    except Exception as e:
        rospy.logerr(f"Gripper error: {e}")

def lift(delta_z):
    if not wait_for_fresh_joint_state():
        rospy.logwarn("Skipping lift due to stale joint states.")
        return

    group.set_start_state_to_current_state()
    current_pose = group.get_current_pose().pose
    current_pose.position.z += delta_z
    group.set_pose_target(current_pose)

    result = group.plan()
    plan = result[1] if isinstance(result, tuple) else result

    if hasattr(plan, "joint_trajectory") and len(plan.joint_trajectory.points) > 0:
        rospy.loginfo(f"Lifting by {delta_z} meters")
        group.execute(plan, wait=True)
    else:
        rospy.logerr("Planning failed for lift command")

# ----------- Helper Function -----------

def extract_args(action_str):
    try:
        args_str = action_str[action_str.index("(")+1 : action_str.index(")")]
        args = [float(x.strip()) for x in args_str.split(",")]
        return args if len(args) > 1 else args[0]
    except:
        rospy.logerr(f"Failed to parse arguments from: {action_str}")
        return []

# ----------- Main Executor -----------

def parse_and_execute(json_path):
    if not os.path.exists(json_path):
        rospy.logerr(f"Command file not found at {json_path}")
        return

    with open(json_path, 'r') as f:
        data = json.load(f)

    actions = data.get('actions', [])
    for action in actions:
        rospy.loginfo(f"Executing action: {action}")
        if action.startswith("move_to"):
            x, y, z = extract_args(action)
            success = move_to(x, y, z)
            if not success:
                rospy.logwarn("Aborting remaining actions due to failed move_to()")
                return
        elif action.startswith("close_gripper"):
            close_gripper()
        elif action.startswith("open_gripper"):
            open_gripper()
        elif action.startswith("lift"):
            dz = extract_args(action)
            lift(dz)

# ----------- Node Init -----------

if __name__ == "__main__":
    moveit_commander.roscpp_initialize([])
    rospy.init_node("llm_executor_node", anonymous=True)

    group = moveit_commander.MoveGroupCommander("right_arm")
    rospy.loginfo("Initialized MoveGroupCommander for 'right_arm'")

    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    json_cmd_path = "/tmp/command.json"
    parse_and_execute(json_cmd_path)
