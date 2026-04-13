#!/usr/bin/env python3

import re


FRAME_NOTE = (
    "Frame: motion is interpreted in the robot base frame "
    "(forward +x, left +y, right −y, up +z)."
)


def issue_category(uncertainty_report):
    issues = uncertainty_report.get("issues", [])
    types = {i.get("type") for i in issues}
    risk = {
        "WORKSPACE_LIMIT_RISK",
        "DOWNWARD_RISK",
        "OVERSIZED_DISPLACEMENT",
    }
    ambiguity = {
        "MISSING_MAGNITUDE",
        "VAGUE_MAGNITUDE",
        "AMBIGUOUS_FRAME",
        "LOW_PARSE_CONFIDENCE",
        "UNSPECIFIED_Z_FOR_MOVE_TO",
    }
    has_r = bool(types & risk)
    has_a = bool(types & ambiguity)
    if has_r and has_a:
        return "mixed"
    if has_r:
        return "risk"
    if has_a:
        return "ambiguity"
    return "none"


def format_intent_summary(intent):
    if not intent or not isinstance(intent, dict):
        return "(no structured intent)"
    parts = []
    for op in intent.get("operations") or []:
        name = op.get("op")
        if name == "move_relative":
            dx = op.get("dx")
            dy = op.get("dy")
            dz = op.get("dz")
            parts.append(
                f"move_relative Δx={dx}, Δy={dy}, Δz={dz} m "
                f"(source={op.get('magnitude_source', '?')}, frame={op.get('frame', '?')})"
            )
        elif name == "move_to":
            p = op.get("target_pose") or {}
            parts.append(f"move_to x={p.get('x')}, y={p.get('y')}, z={p.get('z')}")
        else:
            parts.append(str(name))
    if not parts:
        return "(empty operations)"
    return "; ".join(parts)


def suggested_clarification_commands(raw_command):
    text = (raw_command or "").lower()
    direction = None
    for d in ("forward", "backward", "back", "left", "right", "up", "down"):
        if re.search(rf"\b{re.escape(d)}\b", text):
            direction = "back" if d == "back" else d
            break
    if direction == "back":
        direction = "backward"
    outs = []
    if direction:
        outs.extend(
            [
                f"move {direction} by 5 cm",
                f"move {direction} by 10 cm",
                f"move {direction} by 2 cm",
            ]
        )
    outs.append("move forward by 10 cm")
    outs.append("move up by 5 cm")
    seen = set()
    uniq = []
    for o in outs:
        if o not in seen:
            seen.add(o)
            uniq.append(o)
    return uniq[:5]
