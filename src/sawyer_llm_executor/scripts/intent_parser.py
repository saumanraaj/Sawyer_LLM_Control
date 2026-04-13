#!/usr/bin/env python3

import re

from command_schema import normalize_intent


class IntentParser:
    def __init__(self, gpt_client):
        self.gpt = gpt_client

    def parse(self, user_command):
        deterministic_ops = self._parse_direct_relative_command(user_command)
        if deterministic_ops is not None:
            return normalize_intent(
                raw_text=user_command,
                operations=deterministic_ops,
                source="deterministic",
                parse_confidence=0.95,
            )

        llm_output = self.gpt.get_vlm_output(None, user_command)
        normalized_ops, confidence = self._normalize_llm_output(llm_output)
        return normalize_intent(
            raw_text=user_command,
            operations=normalized_ops,
            source="llm",
            parse_confidence=confidence,
        )

    def _parse_direct_relative_command(self, user_command):
        cmd = (user_command or "").lower().strip()
        if not cmd.startswith("move"):
            return None

        pattern = r"(forward|backward|back|left|right|up|down)\s*(?:by\s*)?(-?\d*\.?\d+)\s*(cm|centimeter|centimeters|mm|m|meter|meters)\b|(forward|backward|back|left|right|up|down)\s*(?:by\s*)?(-?\d*\.?\d+)(cm|mm|m)\b"
        raw_matches = re.findall(pattern, cmd)
        if not raw_matches:
            return None

        normalized = []
        for tup in raw_matches:
            if tup[0]:
                normalized.append((tup[0], tup[1], tup[2]))
            else:
                normalized.append((tup[3], tup[4], tup[5]))

        operations = []
        for direction, value_str, unit in normalized:
            magnitude = float(value_str)
            if unit in ("cm", "centimeter", "centimeters") or (
                unit == "mm"
            ):
                if unit == "mm":
                    magnitude /= 1000.0
                else:
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

            operations.append(
                {
                    "op": "move_relative",
                    "dx": dx,
                    "dy": dy,
                    "dz": dz,
                    "magnitude_source": "explicit",
                    "frame": "base",
                    "frame_source": "explicit",
                }
            )

        return operations

    def _normalize_llm_output(self, llm_output):
        if not isinstance(llm_output, dict):
            return [], 0.0

        if "operations" in llm_output and isinstance(llm_output["operations"], list):
            confidence = llm_output.get("parser_meta", {}).get("parse_confidence", 0.75)
            return llm_output["operations"], float(confidence)

        actions = llm_output.get("actions", [])
        position = llm_output.get("position", [0.6, 0.0])
        operations = []

        for action in actions:
            if action.startswith("move_relative"):
                args = self._extract_args(action, expected=3)
                operations.append(
                    {
                        "op": "move_relative",
                        "dx": args[0],
                        "dy": args[1],
                        "dz": args[2],
                        "magnitude_source": "explicit",
                        "frame": "base",
                        "frame_source": "implicit",
                    }
                )
            elif action == "move_to":
                x = position[0] if len(position) > 0 else None
                y = position[1] if len(position) > 1 else None
                operations.append(
                    {
                        "op": "move_to",
                        "target_pose": {"x": x, "y": y, "z": 0.6},
                        "frame": "base",
                        "frame_source": "implicit",
                    }
                )
            elif action.startswith("lift"):
                dz = self._extract_args(action, expected=1)[0]
                operations.append({"op": "lift", "dz": dz})
            elif action.startswith("open_gripper"):
                operations.append({"op": "open_gripper"})
            elif action.startswith("close_gripper"):
                operations.append({"op": "close_gripper"})

        parse_conf = llm_output.get("parse_confidence", 0.75)
        return operations, float(parse_conf)

    def _extract_args(self, action_str, expected):
        defaults = [0.0] * expected
        if "(" not in action_str or ")" not in action_str:
            return defaults
        try:
            args_str = action_str[action_str.index("(") + 1 : action_str.index(")")]
            args = [float(x.strip()) for x in args_str.split(",") if x.strip() != ""]
            for i in range(min(expected, len(args))):
                defaults[i] = args[i]
            return defaults
        except (ValueError, IndexError):
            return defaults
