#!/usr/bin/env python3

# llm_command_listener.py

import rospy
import re
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

        data = self.parse_direct_relative_command(user_command)
        if data is None:
            data = self.gpt.get_vlm_output(None, user_command)
        self.parse_and_execute(data)

    def parse_direct_relative_command(self, user_command):
        """
        Deterministic parser for common relative move phrases so we don't
        accidentally fall back to absolute move_to defaults from LLM output.
        """
        cmd = user_command.lower().strip()
        if not cmd.startswith("move"):
            return None

        # Supports: "move forward by 10 cm", "move right 0.1 m", etc.
        pattern = r"(forward|backward|back|left|right|up|down)\s*(?:by\s*)?(-?\d*\.?\d+)\s*(cm|centimeter|centimeters|m|meter|meters)\b"
        matches = re.findall(pattern, cmd)
        if not matches:
            return None

        actions = []
        for direction, value_str, unit in matches:
            magnitude = float(value_str)
            if unit.startswith("cm") or unit.startswith("centimeter"):
                magnitude /= 100.0

            dx, dy, dz = 0.0, 0.0, 0.0
            if direction == "forward":
                dx = magnitude
            elif direction in ("backward", "back"):
                dx = -magnitude
            elif direction == "left":
                dy = magnitude
            elif direction == "right":
                dy = -magnitude
            elif direction == "up":
                dz = magnitude
            elif direction == "down":
                dz = -magnitude

            actions.append(f"move_relative({dx}, {dy}, {dz})")

        rospy.loginfo(f"Using direct relative parser actions: {actions}")
        return {"actions": actions, "position": [0.6, 0.0], "delta": [0, 0, 0]}

    def parse_and_execute(self, data):
        actions = data.get('actions', [])
        position = data.get('position', [0.6, 0.0])
        delta = data.get('delta', [0, 0, 0])

        x, y = position[0], position[1] if len(position) >= 2 else 0.0
        z = 0.6
        default_dx = delta[0] if len(delta) >= 1 else 0.0
        default_dy = delta[1] if len(delta) >= 2 else 0.0
        default_dz = delta[2] if len(delta) >= 3 else 0.0

        for action in actions:
            if action.startswith("move_relative"):
                dx, dy, dz = self.extract_move_relative_args(action, default_dx, default_dy, default_dz)
                success = self.sawyer.move_relative(dx, dy, dz)
                if not success:
                    rospy.logwarn("move_relative failed. Aborting remaining actions.")
                    return
            elif action == "move_to":
                success = self.sawyer.move_to(x, y, z)
                if not success:
                    rospy.logwarn("move_to failed. Aborting remaining actions.")
                    return
            elif action.startswith("open_gripper"):
                self.sawyer.open_gripper()
            elif action.startswith("close_gripper"):
                self.sawyer.close_gripper()
            elif action.startswith("lift"):
                dz = self.extract_args(action)
                self.sawyer.lift(dz)

    def extract_move_relative_args(self, action_str, default_dx=0.0, default_dy=0.0, default_dz=0.0):
        """Extract (dx, dy, dz) from move_relative(dx, dy, dz). Falls back to defaults if no params."""
        if "(" not in action_str or ")" not in action_str:
            return (default_dx, default_dy, default_dz)
        try:
            args_str = action_str[action_str.index("(")+1 : action_str.index(")")]
            args = [float(x.strip()) for x in args_str.split(",")]
            dx = args[0] if len(args) >= 1 else default_dx
            dy = args[1] if len(args) >= 2 else default_dy
            dz = args[2] if len(args) >= 3 else default_dz
            return (dx, dy, dz)
        except (ValueError, IndexError):
            rospy.logwarn(f"Failed to parse move_relative args from: {action_str}")
            return (default_dx, default_dy, default_dz)

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

