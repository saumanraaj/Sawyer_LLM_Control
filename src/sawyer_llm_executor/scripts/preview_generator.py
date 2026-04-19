#!/usr/bin/env python3

from hci_helpers import describe_move_to, describe_relative_move


class PreviewGenerator:
    def build_preview(self, intent, robot_context=None):
        robot_context = robot_context or {}
        current_pose = robot_context.get("current_pose")
        workspace_limits = robot_context.get("workspace_limits")
        ops = intent.get("operations", [])
        blocks = []
        raw = (intent.get("raw_text") or "").strip()
        if raw:
            blocks.append(f'You asked: “{raw}”')

        x = y = z = None
        if current_pose:
            x, y, z = current_pose
            blocks.append(
                "Arm position now (approx.): "
                f"{x:.2f} m toward the front, {y:.2f} m to the side, {z:.2f} m up."
            )

        step_lines = []
        for idx, op in enumerate(ops):
            op_name = op.get("op")
            if op_name == "move_relative":
                dx = float(op.get("dx") or 0.0)
                dy = float(op.get("dy") or 0.0)
                dz = float(op.get("dz") or 0.0)
                human = describe_relative_move(dx, dy, dz)
                step_lines.append(f"Step {idx + 1}: {human}")
                if x is not None:
                    x += dx
                    y += dy
                    z += dz
            elif op_name == "move_to":
                step_lines.append(f"Step {idx + 1}: {describe_move_to(op)}")
                pose = op.get("target_pose", {})
                px = pose.get("x")
                py = pose.get("y")
                pz = pose.get("z")
                if x is not None:
                    x = x if px is None else float(px)
                    y = y if py is None else float(py)
                    z = z if pz is None else float(pz)
            elif op_name == "open_gripper":
                step_lines.append(f"Step {idx + 1}: Open the gripper.")
            elif op_name == "close_gripper":
                step_lines.append(f"Step {idx + 1}: Close the gripper.")
            else:
                step_lines.append(f"Step {idx + 1}: {op_name.replace('_', ' ')}")

        if step_lines:
            blocks.append("\n".join(step_lines))

        if x is not None and workspace_limits:
            in_bounds = (
                workspace_limits["x"][0] <= x <= workspace_limits["x"][1]
                and workspace_limits["y"][0] <= y <= workspace_limits["y"][1]
                and workspace_limits["z"][0] <= z <= workspace_limits["z"][1]
            )
            if in_bounds:
                blocks.append(
                    "Safety check: After this motion the arm should still stay "
                    "within the allowed work area."
                )
            else:
                blocks.append(
                    "Safety check: That target looks outside the usual safe zone — "
                    "consider a shorter move or a different direction."
                )

        return "\n\n".join(blocks)
