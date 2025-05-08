
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def publisher():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('fake_joint_states_publisher', anonymous=True)
    rate = rospy.Rate(50) # 50hz

    joint_names = [
        'right_j0', 'right_j1', 'right_j2', 
        'right_j3', 'right_j4', 'right_j5', 'right_j6'
    ]

    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = joint_names
        msg.position = [0.0 for _ in joint_names] # Fake static pose
        msg.velocity = []
        msg.effort = []
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
