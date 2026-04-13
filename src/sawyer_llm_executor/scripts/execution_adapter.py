#!/usr/bin/env python3


class ExecutionAdapter:
    def intent_to_legacy_command(self, intent):
        actions = []
        position = [0.6, 0.0]
        delta = [0.0, 0.0, 0.0]

        for op in intent.get("operations", []):
            op_name = op.get("op")
            if op_name == "move_relative":
                dx = op.get("dx") or 0.0
                dy = op.get("dy") or 0.0
                dz = op.get("dz") or 0.0
                actions.append(f"move_relative({dx}, {dy}, {dz})")
            elif op_name == "move_to":
                pose = op.get("target_pose", {})
                x = pose.get("x", 0.6)
                y = pose.get("y", 0.0)
                z = pose.get("z", 0.6)
                actions.append("move_to")
                position = [x, y]
                delta = [0.0, 0.0, z - 0.6]
            elif op_name == "open_gripper":
                actions.append("open_gripper")
            elif op_name == "close_gripper":
                actions.append("close_gripper")
            elif op_name == "lift":
                dz = op.get("dz", 0.1)
                actions.append(f"lift({dz})")

        return {"actions": actions, "position": position, "delta": delta}
