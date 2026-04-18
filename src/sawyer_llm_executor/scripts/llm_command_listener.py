#!/usr/bin/env python3

# llm_command_listener.py

import re

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from gpt import gpt_api
from sawyer_action import sawyer_actions

# Deterministic phrases -> move_to cached vision pose (no OpenAI call)
_VISION_GOTO_RES = [
    re.compile(p, re.IGNORECASE)
    for p in (
        r"\bgo\s+to\s+the\s+cube\b",
        r"\bmove\s+to\s+the\s+cube\b",
        r"\bgo\s+to\s+the\s+blue\s+cube\b",
        r"\bmove\s+to\s+the\s+blue\s+cube\b",
        r"\bmove\s+to\s+the\s+detected\s+object\b",
        r"\bgo\s+to\s+the\s+detected\s+object\b",
        r"\bgo\s+to\s+what\s+the\s+camera\s+sees\b",
    )
]


class gpt_controller():
    def __init__(self):
        rospy.init_node("llm_executor_node", anonymous=True)
        self.sawyer = sawyer_actions()
        self.gpt = gpt_api()
        self._vision_stale_sec = rospy.get_param("~vision_stale_sec", 2.0)
        self._vision_valid = False
        self._last_pose_xyz = None  # (float, float, float)
        self._last_pose_wall_time = None  # rospy.Time

        rospy.Subscriber("/llm/user_input", String, self.callback)
        rospy.Subscriber(
            "/vision_tracker/target_pose", PoseStamped, self._vision_pose_cb
        )
        rospy.Subscriber("/vision_tracker/target_valid", Bool, self._vision_valid_cb)

        print("Listening for LLM user commands...")
        rospy.spin()

    def _vision_pose_cb(self, msg):
        p = msg.pose.position
        self._last_pose_xyz = (float(p.x), float(p.y), float(p.z))
        self._last_pose_wall_time = rospy.Time.now()

    def _vision_valid_cb(self, msg):
        self._vision_valid = bool(msg.data)

    def vision_fresh(self):
        if (
            not self._vision_valid
            or self._last_pose_xyz is None
            or self._last_pose_wall_time is None
        ):
            return False
        age = (rospy.Time.now() - self._last_pose_wall_time).to_sec()
        return age < self._vision_stale_sec

    def build_vision_context(self):
        if self.vision_fresh():
            x, y, z = self._last_pose_xyz
            return (
                "Vision: tracked object in robot base frame at "
                f"x={x:.4f} m, y={y:.4f} m, z={z:.4f} m."
            )
        return (
            "Vision: no reliable tracked object right now "
            "(nothing visible or data is stale)."
        )

    def try_parse_vision_goto(self, user_command):
        """
        Deterministic 'go to cube' style commands using cached vision pose.
        Returns a data dict, or None if no phrase matched.
        """
        for pat in _VISION_GOTO_RES:
            if pat.search(user_command):
                if not self.vision_fresh():
                    rospy.logwarn(
                        "[vision] Goto phrase matched but no fresh vision pose "
                        "(target not visible, TF failed, or stream stale)."
                    )
                    return {"actions": [], "position": [0.6, 0.0], "delta": [0, 0, 0]}
                x, y, z = self._last_pose_xyz
                rospy.loginfo(
                    f"[vision] Deterministic move_to from vision: ({x:.4f}, {y:.4f}, {z:.4f})"
                )
                return {
                    "actions": ["move_to"],
                    "position": [x, y, z],
                    "delta": [0, 0, 0],
                }
        return None

    def callback(self, msg):
        user_command = msg.data
        print(f"Received user command: {user_command}")

        data = self.try_parse_vision_goto(user_command)
        if data is not None:
            self.parse_and_execute(data)
            return

        data = self.parse_direct_relative_command(user_command)
        if data is None:
            vision_context = self.build_vision_context()
            data = self.gpt.get_vlm_output(None, user_command, vision_context)
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
        actions = data.get("actions", [])
        position = data.get("position", [0.6, 0.0])
        delta = data.get("delta", [0, 0, 0])

        x = position[0] if len(position) >= 1 else 0.6
        y = position[1] if len(position) >= 2 else 0.0
        z = position[2] if len(position) >= 3 else 0.6
        default_dx = delta[0] if len(delta) >= 1 else 0.0
        default_dy = delta[1] if len(delta) >= 2 else 0.0
        default_dz = delta[2] if len(delta) >= 3 else 0.0

        for action in actions:
            if action.startswith("move_relative"):
                dx, dy, dz = self.extract_move_relative_args(
                    action, default_dx, default_dy, default_dz
                )
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
            args_str = action_str[action_str.index("(") + 1 : action_str.index(")")]
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
            args_str = action_str[action_str.index("(") + 1 : action_str.index(")")]
            args = [float(x.strip()) for x in args_str.split(",")]
            return args[0] if args else 0.1
        except Exception:
            rospy.logwarn(f"Failed to extract args from action: {action_str}")
            return 0.1


if __name__ == "__main__":
    gpt_controller()
