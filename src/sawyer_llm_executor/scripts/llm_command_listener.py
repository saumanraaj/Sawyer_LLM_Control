#!/usr/bin/env python3

import json
import rospy
import re
from std_msgs.msg import String
from gpt import gpt_api
from sawyer_action import sawyer_actions
from execution_adapter import ExecutionAdapter
from interaction_logger import InteractionLogger
from interaction_manager import InteractionManager
from interaction_policy import InteractionPolicy
from intent_parser import IntentParser
from preview_generator import PreviewGenerator
from uncertainty_engine import UncertaintyEngine
from hci_helpers import format_intent_summary, issue_category

class gpt_controller():
    def __init__(self):
        rospy.init_node("llm_executor_node", anonymous=True)
        self.sawyer = sawyer_actions()
        self.gpt = gpt_api()
        self.enable_interaction_layer = rospy.get_param(
            "~enable_interaction_layer",
            rospy.get_param(
                "/llm_executor_node/enable_interaction_layer",
                rospy.get_param("/enable_interaction_layer", False),
            ),
        )
        self.max_interaction_turns = rospy.get_param("~max_interaction_turns", 3)
        rospy.loginfo(f"Interaction layer enabled: {self.enable_interaction_layer}")

        self.intent_parser = IntentParser(self.gpt)
        self.uncertainty_engine = UncertaintyEngine()
        self.interaction_policy = InteractionPolicy()
        self.preview_generator = PreviewGenerator()
        self.interaction_manager = InteractionManager(
            prompt_topic=rospy.get_param("~system_prompt_topic", "/llm/system_prompt"),
            prompt_json_topic=rospy.get_param("~system_prompt_json_topic", "/llm/system_prompt_json"),
            feedback_topic=rospy.get_param("~feedback_topic", "/llm/user_response"),
            timeout_sec=float(rospy.get_param("~interaction_timeout_sec", 90.0)),
        )
        self.execution_adapter = ExecutionAdapter()
        log_path = rospy.get_param("~interaction_log_path", "/home/sauman25/ros_ws/logs/interaction_events.jsonl")
        self.interaction_logger = InteractionLogger(log_path)
        self.state_pub = rospy.Publisher(
            rospy.get_param("~interaction_state_topic", "/llm/interaction_state"),
            String,
            queue_size=10,
        )

        rospy.Subscriber("/llm/user_input", String, self.callback)

        print("Listening for LLM user commands...")
        rospy.spin()

    def callback(self, msg):
        user_command = msg.data
        lowered = user_command.strip().lower()
        if lowered in ("confirm", "cancel", "proceed", "ok", "yes", "go", "stop", "no") or lowered.startswith(
            "edit:"
        ):
            rospy.loginfo("Ignoring command-like response on /llm/user_input. Use /llm/user_response for interaction replies.")
            return

        print(f"Received user command: {user_command}")

        if self.enable_interaction_layer:
            self._process_with_interaction_layer(user_command)
            return

        # Legacy path unchanged when interaction layer is disabled.
        data = self.parse_direct_relative_command(user_command)
        if data is None:
            data = self.gpt.get_vlm_output(None, user_command)
        self.parse_and_execute(data)

    def _process_with_interaction_layer(self, initial_command):
        command = initial_command
        previous_command = None
        self._publish_state("parsing", None, initial_command, 0)
        for turn in range(self.max_interaction_turns):
            intent = self.intent_parser.parse(command)
            context = self._build_robot_context()
            uncertainty_report = self.uncertainty_engine.evaluate(intent, context)
            policy = self.interaction_policy.choose_action(uncertainty_report)
            preview_text = self.preview_generator.build_preview(intent, context)
            request_id = intent.get("request_id", "unknown")
            state_mode = self._policy_to_state(policy["action"])
            self._publish_state(
                state_mode,
                request_id,
                command,
                turn + 1,
                extra={
                    "intent_summary": format_intent_summary(intent),
                    "issue_category": issue_category(uncertainty_report),
                    "policy_action": policy["action"],
                },
            )
            self.interaction_logger.log_event(
                "policy_decision",
                {
                    "turn": turn + 1,
                    "intent": intent,
                    "uncertainty": uncertainty_report,
                    "policy": policy,
                },
            )

            if policy["action"] == "EXECUTE":
                legacy_data = self.execution_adapter.intent_to_legacy_command(intent)
                self.interaction_logger.log_event("execution_handoff", {"legacy_data": legacy_data})
                self._publish_state(
                    "executing",
                    request_id,
                    command,
                    turn + 1,
                    extra={
                        "intent_summary": format_intent_summary(intent),
                        "issue_category": "none",
                        "policy_action": "EXECUTE",
                    },
                )
                self.parse_and_execute(legacy_data)
                self._publish_state(
                    "completed",
                    request_id,
                    command,
                    turn + 1,
                    extra={"intent_summary": format_intent_summary(intent)},
                )
                self._publish_idle()
                return

            interaction_result = self.interaction_manager.handle(
                policy,
                uncertainty_report,
                preview_text,
                request_id,
                intent=intent,
                raw_command=command,
            )
            self.interaction_logger.log_event(
                "interaction_result",
                {"turn": turn + 1, "result": interaction_result},
            )

            if interaction_result["outcome"] == "cancel":
                rospy.loginfo("Command cancelled by user.")
                self._publish_state("cancelled", request_id, command, turn + 1)
                self._publish_idle()
                return

            if interaction_result["outcome"] == "edit":
                edited = interaction_result.get("edited_command", "").strip()
                if not edited:
                    rospy.logwarn("Empty edit response received; cancelling command.")
                    self._publish_state("cancelled", request_id, command, turn + 1)
                    self._publish_idle()
                    return
                previous_command = command
                command = edited
                self._publish_state(
                    "editing",
                    request_id,
                    command,
                    turn + 1,
                    extra={
                        "previous_command": previous_command,
                        "repair_note": f"Revised: {previous_command!r} → {command!r}",
                    },
                )
                continue

            legacy_data = self.execution_adapter.intent_to_legacy_command(intent)
            self.interaction_logger.log_event("execution_handoff", {"legacy_data": legacy_data})
            self._publish_state("executing", request_id, command, turn + 1)
            self.parse_and_execute(legacy_data)
            self._publish_state("completed", request_id, command, turn + 1)
            self._publish_idle()
            return

        rospy.logwarn("Max interaction turns exceeded; aborting request.")
        self._publish_state("timeout", None, command, self.max_interaction_turns)
        self._publish_idle()

    def _build_robot_context(self):
        context = {"workspace_limits": self.sawyer.workspace_limits}
        try:
            pose = self.sawyer.group.get_current_pose().pose
            context["current_pose"] = (pose.position.x, pose.position.y, pose.position.z)
        except Exception as e:
            rospy.logwarn(f"Unable to fetch current pose for preview/context: {e}")
        return context

    def _policy_to_state(self, policy_action):
        if policy_action == "CLARIFY":
            return "awaiting_clarification"
        if policy_action == "PREVIEW_CONFIRM":
            return "awaiting_confirmation"
        if policy_action == "WARN":
            return "awaiting_warning_ack"
        if policy_action == "EXECUTE":
            return "ready_to_execute"
        return "unknown"

    def _publish_state(self, mode, request_id, current_command, turn, extra=None):
        payload = {
            "mode": mode,
            "request_id": request_id,
            "command": current_command,
            "turn": turn,
            "timestamp": rospy.Time.now().to_sec(),
        }
        if extra:
            payload.update(extra)
        self.state_pub.publish(String(data=json.dumps(payload)))

    def _publish_idle(self):
        self._publish_state(
            "idle",
            None,
            None,
            0,
            extra={
                "intent_summary": None,
                "issue_category": "none",
                "policy_action": None,
            },
        )

    def parse_direct_relative_command(self, user_command):
        """
        Deterministic parser for common relative move phrases so we don't
        accidentally fall back to absolute move_to defaults from LLM output.
        """
        cmd = user_command.lower().strip()
        if not cmd.startswith("move"):
            return None

        # Supports: "move forward by 10 cm", "move right 0.1 m", etc.
        pattern = r"(forward|backward|back|left|right|up|down)\s*(?:by\s*)?(-?\d*\.?\d+)\s*(cm|centimeter|centimeters|mm|m|meter|meters)\b|(forward|backward|back|left|right|up|down)\s*(?:by\s*)?(-?\d*\.?\d+)(cm|mm|m)\b"
        raw_matches = re.findall(pattern, cmd)
        if not raw_matches:
            return None

        matches = []
        for tup in raw_matches:
            if tup[0]:
                matches.append((tup[0], tup[1], tup[2]))
            else:
                matches.append((tup[3], tup[4], tup[5]))

        actions = []
        for direction, value_str, unit in matches:
            magnitude = float(value_str)
            if unit == "mm":
                magnitude /= 1000.0
            elif unit.startswith("cm") or unit.startswith("centimeter"):
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

