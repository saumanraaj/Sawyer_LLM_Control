'''from sawyer_action import sawyer_actions
import rospy

rospy.init_node("sawyer_actions_node", anonymous=True)
s = sawyer_actions()
s.move_to(0.5,0.2,0.0) '''
import rospy
import moveit_commander
from sawyer_action import sawyer_actions

rospy.init_node("sawyer_test_node", anonymous=True)
s = sawyer_actions()

# 🔥 1. Synchronize robot to MoveIt expected pose
rospy.loginfo("Moving robot to known safe starting pose (gripper_down)")
s.group.set_named_target('gripper_down')  # from SRDF
s.group.go(wait=True)

# (IMPORTANT) Wait a little to stabilize
rospy.sleep(2.0)


rospy.loginfo("Now moving to test point...")
s.move_to(0.6, 0.0, 0.6)  # Example safe target
