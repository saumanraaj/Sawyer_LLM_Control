#!/usr/bin/env python3

import json
import threading

import rospy
from std_msgs.msg import String

from hri_session_store import HRISessionStore


class HRIRosBridge:
    def __init__(
        self,
        store=None,
        cmd_topic="/llm/user_input",
        response_topic="/llm/user_response",
        prompt_topic="/llm/system_prompt_json",
        state_topic="/llm/interaction_state",
    ):
        self.store = store or HRISessionStore()
        self.cmd_pub = None
        self.response_pub = None
        self._init_lock = threading.Lock()
        self._started = False

        self.cmd_topic = cmd_topic
        self.response_topic = response_topic
        self.prompt_topic = prompt_topic
        self.state_topic = state_topic

    def start(self):
        with self._init_lock:
            if self._started:
                return

            if not rospy.core.is_initialized():
                rospy.init_node("hri_web_bridge_node", anonymous=True, disable_signals=True)

            self.cmd_pub = rospy.Publisher(self.cmd_topic, String, queue_size=10)
            self.response_pub = rospy.Publisher(self.response_topic, String, queue_size=10)
            rospy.Subscriber(self.prompt_topic, String, self._on_prompt)
            rospy.Subscriber(self.state_topic, String, self._on_state)
            self._started = True

    def submit_command(self, text):
        self.start()
        self.cmd_pub.publish(String(data=text))
        self.store.add_ui_event("ui_command", {"text": text})

    def submit_response(self, response):
        self.start()
        self.response_pub.publish(String(data=response))
        self.store.add_ui_event("ui_response", {"response": response})

    def get_state(self):
        self.start()
        return self.store.get_state()

    def get_events(self, since_id=0):
        self.start()
        return self.store.get_events(since_id=since_id)

    def _on_prompt(self, msg):
        payload = self._safe_json(msg.data, fallback_type="prompt")
        self.store.set_latest_prompt(payload)

    def _on_state(self, msg):
        payload = self._safe_json(msg.data, fallback_type="state")
        mode = payload.get("mode", "unknown")
        request_id = payload.get("request_id")
        command = payload.get("command")
        turn = payload.get("turn", 0)
        self.store.set_state(mode=mode, request_id=request_id, command=command, turn=turn)

    def _safe_json(self, value, fallback_type):
        try:
            return json.loads(value)
        except Exception:
            return {"type": fallback_type, "raw": value}
