#!/usr/bin/env python3

import threading
import time


class HRISessionStore:
    def __init__(self, max_events=300):
        self._lock = threading.Lock()
        self._max_events = max_events
        self._next_event_id = 1
        self._state = {
            "mode": "idle",
            "request_id": None,
            "latest_prompt": None,
            "command": None,
            "turn": 0,
            "last_updated": time.time(),
        }
        self._events = []

    def get_state(self):
        with self._lock:
            return dict(self._state)

    def set_state(self, mode, request_id=None, command=None, turn=0):
        with self._lock:
            self._state.update(
                {
                    "mode": mode,
                    "request_id": request_id,
                    "command": command,
                    "turn": turn,
                    "last_updated": time.time(),
                }
            )
            self._append_event_locked("state", dict(self._state))

    def set_latest_prompt(self, prompt_payload):
        with self._lock:
            self._state["latest_prompt"] = prompt_payload
            self._state["last_updated"] = time.time()
            self._append_event_locked("prompt", prompt_payload)

    def add_ui_event(self, event_type, payload):
        with self._lock:
            self._append_event_locked(event_type, payload)

    def get_events(self, since_id=0):
        with self._lock:
            return [e for e in self._events if e["event_id"] > since_id]

    def _append_event_locked(self, event_type, payload):
        event = {
            "event_id": self._next_event_id,
            "event_type": event_type,
            "timestamp": time.time(),
            "payload": payload,
        }
        self._next_event_id += 1
        self._events.append(event)
        if len(self._events) > self._max_events:
            self._events = self._events[-self._max_events :]
