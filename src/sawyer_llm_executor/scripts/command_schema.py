#!/usr/bin/env python3

import copy
import uuid


DEFAULT_SCHEMA = {
    "request_id": "",
    "raw_text": "",
    "intent_type": "motion",
    "operations": [],
    "defaults": {
        "frame": "base",
        "z_default_m": 0.6,
    },
    "parser_meta": {
        "source": "unknown",
        "parse_confidence": 0.5,
        "missing_fields": [],
    },
}


def _as_float_or_none(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _normalize_op(raw_op):
    if not isinstance(raw_op, dict):
        return None

    op_type = raw_op.get("op")
    if op_type not in ("move_relative", "move_to", "open_gripper", "close_gripper", "lift"):
        return None

    if op_type == "move_relative":
        return {
            "op": "move_relative",
            "dx": _as_float_or_none(raw_op.get("dx")),
            "dy": _as_float_or_none(raw_op.get("dy")),
            "dz": _as_float_or_none(raw_op.get("dz")),
            "magnitude_source": raw_op.get("magnitude_source", "explicit"),
            "frame": raw_op.get("frame"),
            "frame_source": raw_op.get("frame_source", "implicit"),
        }

    if op_type == "move_to":
        target_pose = raw_op.get("target_pose", {})
        return {
            "op": "move_to",
            "target_pose": {
                "x": _as_float_or_none(target_pose.get("x")),
                "y": _as_float_or_none(target_pose.get("y")),
                "z": _as_float_or_none(target_pose.get("z")),
            },
            "frame": raw_op.get("frame"),
            "frame_source": raw_op.get("frame_source", "implicit"),
        }

    if op_type == "lift":
        return {"op": "lift", "dz": _as_float_or_none(raw_op.get("dz"))}

    return {"op": op_type}


def normalize_intent(raw_text, operations, source="unknown", parse_confidence=0.5, defaults=None):
    normalized = copy.deepcopy(DEFAULT_SCHEMA)
    normalized["request_id"] = str(uuid.uuid4())
    normalized["raw_text"] = raw_text or ""
    normalized["parser_meta"]["source"] = source
    normalized["parser_meta"]["parse_confidence"] = float(parse_confidence)

    if isinstance(defaults, dict):
        normalized["defaults"].update(defaults)

    normalized_ops = []
    for op in operations or []:
        normalized_op = _normalize_op(op)
        if normalized_op:
            normalized_ops.append(normalized_op)
    normalized["operations"] = normalized_ops

    missing = []
    for idx, op in enumerate(normalized_ops):
        if op["op"] == "move_relative":
            if op["dx"] is None and op["dy"] is None and op["dz"] is None:
                missing.append(f"operations[{idx}].delta")
            if op.get("frame") in (None, ""):
                missing.append(f"operations[{idx}].frame")
        elif op["op"] == "move_to":
            pose = op["target_pose"]
            if pose["x"] is None:
                missing.append(f"operations[{idx}].target_pose.x")
            if pose["y"] is None:
                missing.append(f"operations[{idx}].target_pose.y")
            if pose["z"] is None:
                missing.append(f"operations[{idx}].target_pose.z")
            if op.get("frame") in (None, ""):
                missing.append(f"operations[{idx}].frame")

    normalized["parser_meta"]["missing_fields"] = missing
    return normalized
