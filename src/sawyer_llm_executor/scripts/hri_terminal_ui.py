#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


class HRITerminalUI:
    def __init__(self):
        rospy.init_node("hri_terminal_ui", anonymous=True)
        self.cmd_pub = rospy.Publisher("/llm/user_input", String, queue_size=10)
        self.reply_pub = rospy.Publisher("/llm/user_response", String, queue_size=10)
        rospy.Subscriber("/llm/system_prompt", String, self._on_prompt)

    def _on_prompt(self, msg):
        print("\n[Robot Prompt]")
        print(msg.data)
        print("")

    def run(self):
        print("HRI Terminal UI started.")
        print("Type plain text to send a new command.")
        print("Use '/r <text>' to send a response on /llm/user_response.")
        print("Use '/confirm', '/cancel', '/edit <full command>' shortcuts.")
        print("Type '/quit' to exit.")
        while not rospy.is_shutdown():
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("")
                return
            if not line:
                continue
            if line == "/quit":
                return
            if line == "/confirm":
                self.reply_pub.publish(String(data="confirm"))
                continue
            if line == "/cancel":
                self.reply_pub.publish(String(data="cancel"))
                continue
            if line.startswith("/edit "):
                self.reply_pub.publish(String(data=f"edit: {line[6:].strip()}"))
                continue
            if line.startswith("/r "):
                self.reply_pub.publish(String(data=line[3:].strip()))
                continue
            self.cmd_pub.publish(String(data=line))


if __name__ == "__main__":
    HRITerminalUI().run()
