#!/usr/bin/env python3

import re


FRAME_NOTE = (
    "Forward, back, left, right, up, and down are always from the robot’s point "
    "of view (looking out from its base)."
)


def _fmt_distance_m(m):
    """Return a short distance string (cm preferred for human scale)."""
    cm = abs(m) * 100.0
    if cm >= 100.0:
        return f"{abs(m):.2f} m"
    if abs(cm - round(cm)) < 0.05:
        return f"{round(cm):.0f} cm"
    return f"{cm:.1f} cm"


def describe_relative_move(dx, dy, dz):
    """
    Plain-language description of a base-frame relative move.
    Matches Sawyer conventions: forward +x, left +y, right −y, up +z.
    """
    tol = 1e-6
    parts = []
    if abs(dx) > tol:
        parts.append(f"{_fmt_distance_m(dx)} {'forward' if dx > 0 else 'backward'}")
    if abs(dy) > tol:
        parts.append(f"{_fmt_distance_m(dy)} to the {'left' if dy > 0 else 'right'}")
    if abs(dz) > tol:
        parts.append(f"{_fmt_distance_m(dz)} {'up' if dz > 0 else 'down'}")
    if not parts:
        return "No movement."
    if len(parts) == 1:
        return f"Move about {parts[0]}."
    return "Move about " + ", ".join(parts[:-1]) + ", and " + parts[-1] + "."


def describe_move_to(op):
    pose = op.get("target_pose") or {}
    px, py, pz = pose.get("x"), pose.get("y"), pose.get("z")
    bits = []
    if px is not None:
        bits.append(f"{px:.2f} m in front or back")
    if py is not None:
        bits.append(f"{py:.2f} m side to side")
    if pz is not None:
        bits.append(f"{pz:.2f} m height")
    if not bits:
        return "Move to a target position."
    if len(bits) == 1:
        return "Move the arm tip to about " + bits[0] + "."
    return (
        "Move the arm tip to about "
        + ", ".join(bits[:-1])
        + ", and "
        + bits[-1]
        + "."
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
        return "I didn’t get a clear motion plan from that message."
    parts = []
    for op in intent.get("operations") or []:
        name = op.get("op")
        if name == "move_relative":
            dx = float(op.get("dx") or 0.0)
            dy = float(op.get("dy") or 0.0)
            dz = float(op.get("dz") or 0.0)
            parts.append(describe_relative_move(dx, dy, dz).rstrip("."))
        elif name == "move_to":
            parts.append(describe_move_to(op).rstrip("."))
        elif name in ("open_gripper", "close_gripper"):
            parts.append("Open the gripper" if name == "open_gripper" else "Close the gripper")
        elif name == "lift":
            parts.append("Adjust height")
        else:
            parts.append(str(name).replace("_", " "))
    if not parts:
        return "No specific motion steps were listed."
    return "; ".join(parts)


def parse_motion_direction(raw_command):
    """Return a normalized direction word from user text, or None."""
    text = (raw_command or "").lower()
    for d in ("forward", "backward", "back", "left", "right", "up", "down"):
        if re.search(rf"\b{re.escape(d)}\b", text):
            return "backward" if d == "back" else d
    return None


_WORKSPACE_WARN_OPPOSITE = {
    "forward": "backward",
    "backward": "forward",
    "left": "right",
    "right": "left",
    "up": "down",
    "down": "up",
}


def suggested_replies_workspace_warn(raw_command):
    """
    When the planned motion is outside the workspace, do not suggest chips that
    repeat the same direction (or larger moves along it). Prefer small moves the
    other way plus orthogonal nudges.
    """
    direction = parse_motion_direction(raw_command)
    outs = []
    if direction and direction in _WORKSPACE_WARN_OPPOSITE:
        opp = _WORKSPACE_WARN_OPPOSITE[direction]
        outs.extend(
            [
                f"move {opp} by 2 cm",
                f"move {opp} by 5 cm",
            ]
        )
    outs.append("move up by 3 cm")
    # Orthogonal nudge: avoid repeating the same axis that hit the limit.
    if direction in ("left", "right"):
        outs.append("move forward by 5 cm")
    elif direction in ("forward", "backward"):
        outs.append("move left by 3 cm")
    elif direction in ("up", "down"):
        outs.append("move forward by 5 cm")
    else:
        outs.append("move forward by 5 cm")
    seen = set()
    uniq = []
    for o in outs:
        if o not in seen:
            seen.add(o)
            uniq.append(o)
    return uniq[:5]


def suggested_clarification_commands(raw_command):
    direction = parse_motion_direction(raw_command)
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
