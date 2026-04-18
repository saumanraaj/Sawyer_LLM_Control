#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def callback(msg):
    # Use current time so MoveIt's "recent timestamp" check passes when clocks differ (robot vs workstation)
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('joint_state_relay')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.Subscriber('/robot/joint_states', JointState, callback)
    rospy.loginfo("Relaying /robot/joint_states to /joint_states...")
    rospy.spin()
