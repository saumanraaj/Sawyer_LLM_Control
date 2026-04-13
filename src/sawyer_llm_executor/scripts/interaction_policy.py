#!/usr/bin/env python3


class InteractionPolicy:
    def choose_action(self, uncertainty_report):
        issues = uncertainty_report.get("issues", [])
        issue_types = {issue["type"] for issue in issues}
        high_severity = [issue for issue in issues if issue.get("severity") == "high"]

        warning_types = {
            "WORKSPACE_LIMIT_RISK",
            "DOWNWARD_RISK",
            "OVERSIZED_DISPLACEMENT",
        }
        clarify_types = {
            "MISSING_MAGNITUDE",
            "VAGUE_MAGNITUDE",
            "AMBIGUOUS_FRAME",
            "LOW_PARSE_CONFIDENCE",
            "UNSPECIFIED_Z_FOR_MOVE_TO",
        }
        preview_types = {
            "LARGE_BUT_ALLOWED_DISPLACEMENT",
            "MULTI_STEP_PREVIEW",
        }

        if issue_types.intersection(warning_types):
            return {"action": "WARN", "reasons": self._filter(issues, warning_types)}

        if issue_types.intersection(clarify_types):
            return {"action": "CLARIFY", "reasons": self._filter(issues, clarify_types)}

        if issue_types.intersection(preview_types) or len(high_severity) > 0:
            return {"action": "PREVIEW_CONFIRM", "reasons": self._filter(issues, preview_types)}

        return {"action": "EXECUTE", "reasons": []}

    def _filter(self, issues, issue_types):
        return [issue for issue in issues if issue["type"] in issue_types]
