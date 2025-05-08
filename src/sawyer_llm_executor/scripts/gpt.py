# gpt.py

import openai
import os
import json

class gpt_api:
    def __init__(self):
        self.client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    def get_vlm_output(self, rgb_image, user_command):
        prompt = f"""
You are controlling a Sawyer robot arm.

Given a user command like "move to the blue cube", you must output a JSON object with **TWO fields**:

- "actions": a list of actions from this list only: [move_to, open_gripper, close_gripper, lift(delta_z)]
- "position": an (x, y) coordinate in meters for move_to. Assume z=0.6 meters by default.

---
**Here are some examples:**

User command: "move forward by 10 centimeters"

Output:
{{
  "actions": ["move_to"],
  "position": [0.7, 0.0]
}}

---

User command: "move right by 20 centimeters"

Output:
{{
  "actions": ["move_to"],
  "position": [0.6, -0.2]
}}

---

User command: "move to position (0.5, 0.1)"

Output:
{{
  "actions": ["move_to"],
  "position": [0.5, 0.1]
}}

---

Now, follow the same style.  
Only output valid JSON — no extra words.

User command: "{user_command}"
"""

        try:
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=200
            )

            message_content = response.choices[0].message.content
            output = json.loads(message_content)
            return output

        except Exception as e:
            print(f"GPT API error: {e}")
            return {
                "actions": [],
                "position": [0.6, 0.0]
            }

