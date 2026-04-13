#!/usr/bin/env python3

import math
import re


class UncertaintyEngine:
    def __init__(self, config=None):
        self.config = config or {}
        self.downward_warn_m = self.config.get("downward_warn_m", 0.08)
        self.displacement_warn_m = self.config.get("displacement_warn_m", 0.25)
        self.displacement_preview_m = self.config.get("displacement_preview_m", 0.15)
        self.low_conf_threshold = self.config.get("low_conf_threshold", 0.65)

    def evaluate(self, intent, robot_context=None):
        robot_context = robot_context or {}
        issues = []
        ops = intent.get("operations", [])
        parser_meta = intent.get("parser_meta", {})
        raw_text = (intent.get("raw_text") or "").lower().strip()
        has_explicit_distance = self._has_explicit_distance(raw_text)
        has_directional_motion = self._has_directional_motion(raw_text)

        if parser_meta.get("parse_confidence", 1.0) < self.low_conf_threshold:
            issues.append(self._issue("LOW_PARSE_CONFIDENCE", "medium", "Parser confidence is low."))

        total_dx, total_dy, total_dz = 0.0, 0.0, 0.0
        for op in ops:
            op_name = op.get("op")
            if op_name == "move_relative":
                dx = op.get("dx")
                dy = op.get("dy")
                dz = op.get("dz")
                if dx is None and dy is None and dz is None:
                    issues.append(self._issue("MISSING_MAGNITUDE", "high", "Relative move missing offsets."))
                if has_directional_motion and not has_explicit_distance:
                    issues.append(
                        self._issue(
                            "MISSING_MAGNITUDE",
                            "high",
                            "Directional move has no explicit distance (e.g., '10 cm').",
                        )
                    )
                if op.get("magnitude_source") == "vague":
                    issues.append(self._issue("VAGUE_MAGNITUDE", "high", "Magnitude term is vague."))
                if not op.get("frame"):
                    issues.append(self._issue("AMBIGUOUS_FRAME", "high", "Reference frame is not explicit."))
                dx = dx or 0.0
                dy = dy or 0.0
                dz = dz or 0.0
                total_dx += dx
                total_dy += dy
                total_dz += dz
                if dz < 0.0 and abs(dz) > self.downward_warn_m:
                    issues.append(self._issue("DOWNWARD_RISK", "high", f"Downward move {abs(dz):.3f}m exceeds threshold."))
            elif op_name == "move_to":
                pose = op.get("target_pose", {})
                if pose.get("z") is None:
                    issues.append(self._issue("UNSPECIFIED_Z_FOR_MOVE_TO", "medium", "Absolute move missing z value."))
                if not op.get("frame"):
                    issues.append(self._issue("AMBIGUOUS_FRAME", "high", "Reference frame is not explicit."))

        disp = math.sqrt(total_dx * total_dx + total_dy * total_dy + total_dz * total_dz)
        if disp > self.displacement_warn_m:
            issues.append(self._issue("OVERSIZED_DISPLACEMENT", "high", f"Displacement {disp:.3f}m exceeds warning threshold."))
        elif disp >= self.displacement_preview_m:
            issues.append(self._issue("LARGE_BUT_ALLOWED_DISPLACEMENT", "medium", f"Displacement {disp:.3f}m should be previewed."))

        workspace_issue = self._check_workspace_risk(intent, robot_context)
        if workspace_issue:
            issues.append(workspace_issue)

        if len(ops) > 1:
            issues.append(self._issue("MULTI_STEP_PREVIEW", "medium", "Multi-step sequence should be previewed."))

        return {"issues": issues, "metrics": {"total_displacement_m": disp}}

    def _check_workspace_risk(self, intent, robot_context):
        current_pose = robot_context.get("current_pose")
        workspace_limits = robot_context.get("workspace_limits")
        if not current_pose or not workspace_limits:
            return None

        x, y, z = current_pose
        for op in intent.get("operations", []):
            if op.get("op") == "move_relative":
                x += op.get("dx") or 0.0
                y += op.get("dy") or 0.0
                z += op.get("dz") or 0.0
            elif op.get("op") == "move_to":
                pose = op.get("target_pose", {})
                x = pose.get("x") if pose.get("x") is not None else x
                y = pose.get("y") if pose.get("y") is not None else y
                z = pose.get("z") if pose.get("z") is not None else z

            if not self._in_workspace(x, y, z, workspace_limits):
                msg = f"Target ({x:.3f}, {y:.3f}, {z:.3f}) outside workspace limits."
                return self._issue("WORKSPACE_LIMIT_RISK", "high", msg)
        return None

    def _in_workspace(self, x, y, z, limits):
        return (
            limits["x"][0] <= x <= limits["x"][1]
            and limits["y"][0] <= y <= limits["y"][1]
            and limits["z"][0] <= z <= limits["z"][1]
        )

    def _issue(self, issue_type, severity, message):
        return {"type": issue_type, "severity": severity, "message": message}

    def _has_explicit_distance(self, raw_text):
        # Numeric magnitude + unit (allows "10 cm" or "10cm").
        return re.search(
            r"(-?\d*\.?\d+)\s*(cm|centimeter|centimeters|mm|m|meter|meters)\b|(-?\d*\.?\d+)(cm|mm|m)\b",
            raw_text,
        ) is not None

    def _has_directional_motion(self, raw_text):
        return re.search(r"\b(forward|backward|back|left|right|up|down)\b", raw_text) is not None
