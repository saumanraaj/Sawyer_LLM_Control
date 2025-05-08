#!/usr/bin/env python

# llm_command_listener.py

import rospy
from std_msgs.msg import String
from gpt import gpt_api
from sawyer_action import sawyer_actions

class gpt_controller():
    def __init__(self):
        rospy.init_node("llm_executor_node", anonymous=True)
        self.sawyer = sawyer_actions()
        self.gpt = gpt_api()

        rospy.Subscriber("/llm/user_input", String, self.callback)

        print("Listening for LLM user commands...")
        rospy.spin()

    def callback(self, msg):
        user_command = msg.data
        print(f"Received user command: {user_command}")

        data = self.gpt.get_vlm_output(None, user_command)
        self.parse_and_execute(data)

    def parse_and_execute(self, data):
        actions = data.get('actions', [])
        position = data.get('position', [0.6, 0.0])  # Default safe position

        x, y = position
        z = 0.6  # Default z height

        for action in actions:
            if action.startswith("move_to"):
                success = self.sawyer.move_to(x, y, z)
                if not success:
                    rospy.logwarn("Move failed. Aborting remaining actions.")
                    return
            elif action.startswith("open_gripper"):
                self.sawyer.open_gripper()
            elif action.startswith("close_gripper"):
                self.sawyer.close_gripper()
            elif action.startswith("lift"):
                dz = self.extract_args(action)
                self.sawyer.lift(dz)

    def extract_args(self, action_str):
        try:
            args_str = action_str[action_str.index("(")+1 : action_str.index(")")]
            args = [float(x.strip()) for x in args_str.split(",")]
            return args[0] if args else 0.1
        except:
            rospy.logwarn(f"Failed to extract args from action: {action_str}")
            return 0.1

if __name__ == "__main__":
    gpt_controller()

