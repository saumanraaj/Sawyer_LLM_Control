#!/usr/bin/env python3

import json

import rospy
from std_msgs.msg import String

from hci_helpers import (
    FRAME_NOTE,
    format_intent_summary,
    issue_category,
    suggested_clarification_commands,
    suggested_replies_workspace_warn,
)


class InteractionManager:
    def __init__(
        self,
        prompt_topic="/llm/system_prompt",
        prompt_json_topic="/llm/system_prompt_json",
        feedback_topic="/llm/user_response",
        timeout_sec=90.0,
    ):
        self.prompt_pub = rospy.Publisher(prompt_topic, String, queue_size=10)
        self.prompt_json_pub = rospy.Publisher(prompt_json_topic, String, queue_size=10)
        self.feedback_topic = feedback_topic
        self.timeout_sec = timeout_sec

    def handle(
        self,
        policy_decision,
        uncertainty_report,
        preview_text,
        request_id,
        intent=None,
        raw_command=None,
    ):
        intent = intent or {}
        raw_command = raw_command or intent.get("raw_text") or ""
        action = policy_decision.get("action")
        reasons = policy_decision.get("reasons", [])
        icat = issue_category(uncertainty_report)
        intent_summary = format_intent_summary(intent)
        reason_types = {r.get("type") for r in reasons}
        suggested = suggested_clarification_commands(raw_command)

        if action == "WARN":
            if "WORKSPACE_LIMIT_RISK" in reason_types:
                suggested = suggested_replies_workspace_warn(raw_command)
            # Unsafe as interpreted — do not offer proceed (workspace / oversize / downward).
            msg = "Safety heads-up — please read this before continuing.\n"
            msg += "\n".join([f"• {r['message']}" for r in reasons])
            msg += f"\n\nIn short, I’m planning to: {intent_summary}"
            msg += f"\n\nHow directions work:\n{FRAME_NOTE}"
            msg += (
                "\n\nThis motion isn’t safe to run as interpreted. "
                "Revise the command (e.g. a shorter move), or cancel.\n\n"
                "Your choice:\n"
                "Type a revised command, use Revise command, or say cancel."
            )
            reply = self._prompt_and_wait(
                msg,
                request_id=request_id,
                prompt_type="warning",
                issue_category=icat,
                allowed_actions=["cancel", "edit", "free_text"],
                intent_summary=intent_summary,
                suggested_replies=suggested,
                original_command=raw_command,
            )
            return self._reply_to_result(reply, allow_free_text=True, allow_execute_confirm=False)

        if action == "CLARIFY":
            issue = reasons[0] if reasons else {"message": "I need a bit more detail."}
            msg = "I need a bit more detail to move safely.\n"
            msg += f"\nWhy: {issue['message']}"
            msg += f"\n\nSo far I’m picturing: {intent_summary}"
            msg += f"\n{FRAME_NOTE}"
            msg += "\n\nTry something like “move forward 10 cm” or tap a suggestion below."
            reply = self._prompt_and_wait(
                msg,
                request_id=request_id,
                prompt_type="clarification",
                issue_category=icat,
                allowed_actions=["free_text", "cancel"],
                intent_summary=intent_summary,
                suggested_replies=suggested,
                original_command=raw_command,
            )
            return self._reply_to_result(reply, allow_free_text=True)

        if action == "PREVIEW_CONFIRM":
            # preview_text = request, pose, steps, safety — blocks separated by blank lines.
            msg = preview_text
            msg += f"\n\nHow directions work:\n{FRAME_NOTE}"
            msg += (
                "\n\nYour choice:\n"
                "Say proceed to run it, cancel to stop, or type a changed command."
            )
            reply = self._prompt_and_wait(
                msg,
                request_id=request_id,
                prompt_type="preview",
                issue_category=icat,
                allowed_actions=["proceed", "cancel", "edit", "free_text"],
                intent_summary=intent_summary,
                suggested_replies=suggested,
                original_command=raw_command,
            )
            return self._reply_to_result(reply, allow_free_text=True)

        return {"outcome": "execute", "edited_command": None}

    def _reply_to_result(self, reply, allow_free_text=False, allow_execute_confirm=True):
        lowered = reply.lower().strip()
        if lowered in ("confirm", "proceed", "ok", "yes", "go"):
            if not allow_execute_confirm:
                rospy.logwarn(
                    "[HRI] Proceed/confirm ignored — not allowed for this safety prompt."
                )
                return {"outcome": "cancel", "edited_command": None}
            return {"outcome": "execute", "edited_command": None}
        if lowered in ("cancel", "stop", "no"):
            return {"outcome": "cancel", "edited_command": None}
        if lowered.startswith("edit:"):
            return {"outcome": "edit", "edited_command": reply[5:].strip()}
        if allow_free_text and reply:
            return {"outcome": "edit", "edited_command": reply}
        return {"outcome": "cancel", "edited_command": None}

    def _prompt_and_wait(
        self,
        prompt_text,
        request_id,
        prompt_type,
        issue_category,
        allowed_actions,
        intent_summary,
        suggested_replies,
        original_command,
    ):
        self.prompt_pub.publish(String(data=prompt_text))
        envelope = {
            "request_id": request_id,
            "prompt_type": prompt_type,
            "issue_category": issue_category,
            "allowed_actions": allowed_actions,
            "message": prompt_text,
            "intent_summary": intent_summary,
            "suggested_replies": suggested_replies,
            "original_command": original_command,
            "frame_note": FRAME_NOTE,
        }
        self.prompt_json_pub.publish(String(data=json.dumps(envelope)))
        rospy.loginfo(f"[HRI] {prompt_text}")
        try:
            msg = rospy.wait_for_message(self.feedback_topic, String, timeout=self.timeout_sec)
            return msg.data.strip()
        except rospy.ROSException:
            rospy.logwarn("No user response received before timeout; cancelling request.")
            return "cancel"
