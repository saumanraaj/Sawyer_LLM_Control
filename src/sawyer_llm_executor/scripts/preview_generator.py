#!/usr/bin/env python3


class PreviewGenerator:
    def build_preview(self, intent, robot_context=None):
        robot_context = robot_context or {}
        current_pose = robot_context.get("current_pose")
        workspace_limits = robot_context.get("workspace_limits")
        ops = intent.get("operations", [])
        lines = []
        lines.append(f"Request: {intent.get('raw_text', '')}")

        x = y = z = None
        if current_pose:
            x, y, z = current_pose
            lines.append(f"Current pose: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        for idx, op in enumerate(ops):
            if op.get("op") == "move_relative":
                dx = op.get("dx") or 0.0
                dy = op.get("dy") or 0.0
                dz = op.get("dz") or 0.0
                lines.append(f"Step {idx+1}: move_relative(dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f})")
                if x is not None:
                    x += dx
                    y += dy
                    z += dz
            elif op.get("op") == "move_to":
                pose = op.get("target_pose", {})
                px = pose.get("x")
                py = pose.get("y")
                pz = pose.get("z")
                lines.append(f"Step {idx+1}: move_to(x={px}, y={py}, z={pz})")
                if x is not None:
                    x = x if px is None else px
                    y = y if py is None else py
                    z = z if pz is None else pz
            else:
                lines.append(f"Step {idx+1}: {op.get('op')}")

        if x is not None:
            lines.append(f"Predicted target: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            if workspace_limits:
                in_bounds = (
                    workspace_limits["x"][0] <= x <= workspace_limits["x"][1]
                    and workspace_limits["y"][0] <= y <= workspace_limits["y"][1]
                    and workspace_limits["z"][0] <= z <= workspace_limits["z"][1]
                )
                lines.append(f"Workspace check: {'PASS' if in_bounds else 'FAIL'}")

        return "\n".join(lines)
