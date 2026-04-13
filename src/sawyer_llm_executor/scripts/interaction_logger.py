#!/usr/bin/env python3

import json
import os
from datetime import datetime


class InteractionLogger:
    def __init__(self, log_path):
        self.log_path = log_path
        log_dir = os.path.dirname(log_path)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)

    def log_event(self, event_type, payload):
        record = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "event_type": event_type,
            "payload": payload,
        }
        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record) + "\n")
