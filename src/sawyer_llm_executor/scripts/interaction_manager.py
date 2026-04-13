#!/usr/bin/env python3

import json

import rospy
from std_msgs.msg import String

from hci_helpers import FRAME_NOTE, format_intent_summary, issue_category, suggested_clarification_commands


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
        suggested = suggested_clarification_commands(raw_command)

        if action == "WARN":
            msg = "Safety notice — please read before continuing.\n"
            msg += "\n".join([f"• {r['message']}" for r in reasons])
            msg += f"\n\nWhat I understood: {intent_summary}"
            msg += f"\n{FRAME_NOTE}"
            msg += "\n\nReply with proceed to run as interpreted, cancel to stop, or send a revised command (plain text or start with edit:)."
            reply = self._prompt_and_wait(
                msg,
                request_id=request_id,
                prompt_type="warning",
                issue_category=icat,
                allowed_actions=["proceed", "cancel", "edit", "free_text"],
                intent_summary=intent_summary,
                suggested_replies=suggested,
                original_command=raw_command,
            )
            return self._reply_to_result(reply, allow_free_text=True)

        if action == "CLARIFY":
            issue = reasons[0] if reasons else {"message": "I need a bit more detail."}
            msg = "I’m not sure how far or exactly what you want yet.\n"
            msg += f"\nWhy: {issue['message']}"
            msg += f"\n\nWhat I understood so far: {intent_summary}"
            msg += f"\n{FRAME_NOTE}"
            msg += "\n\nPlease send a clearer command (distance in cm or m helps), or tap a suggestion in the UI."
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
            msg = "Here is the planned motion.\n\n"
            msg += preview_text
            msg += f"\n\nWhat I understood: {intent_summary}"
            msg += f"\n{FRAME_NOTE}"
            msg += "\n\nReply proceed to execute, cancel to stop, or send a revised command."
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

    def _reply_to_result(self, reply, allow_free_text=False):
        lowered = reply.lower().strip()
        if lowered in ("confirm", "proceed", "ok", "yes", "go"):
            return {"outcome": "execute", "edited_command": None}
        if lowered in ("cancel", "stop", "no"):
            return {"outcome": "cancel", "edited_command": None}
        if lowered.startswith("edit:"):
            return {"outcome": "edit", "edited_command": reply[5:].strip()}
        if allow_free_text and reply:
            return {"outcome": "edit", "edited_command": reply}
        return {"outcome": "cancel", "edited_command": None}
