
import openai
import os
import json

class gpt_api:
    def __init__(self):
        self.client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    def get_vlm_output(self, rgb_image, user_command):
        prompt = f"""
You are controlling a Sawyer robot arm.

Output a JSON object with:
- "actions": list of actions. Use move_relative(dx, dy, dz) for relative moves — each move carries its own offset in meters.
- "position": [x, y] for move_to — absolute target. Use when user specifies explicit position. Assume z=0.6.

For relative moves: use move_relative(dx, dy, dz). Forward=+x, Right=-y, Left=+y, Up=+z.
  - 10 cm forward = move_relative(0.1, 0, 0)
  - 20 cm right = move_relative(0, -0.2, 0)
  - 5 cm left = move_relative(0, 0.05, 0)
  - Multiple moves: each move_relative has its own (dx, dy, dz)

---
**Examples:**

User command: "move forward by 10 centimeters"
Output: {{ "actions": ["move_relative(0.1, 0, 0)"], "position": [0.6, 0.0] }}

User command: "move right by 20 centimeters"
Output: {{ "actions": ["move_relative(0, -0.2, 0)"], "position": [0.6, 0.0] }}

User command: "move forward by 10 cm then right by 5 cm"
Output: {{ "actions": ["move_relative(0.1, 0, 0)", "move_relative(0, -0.05, 0)"], "position": [0.6, 0.0] }}

User command: "move left 5 cm, then forward 15 cm, then up 3 cm"
Output: {{ "actions": ["move_relative(0, 0.05, 0)", "move_relative(0.15, 0, 0)", "move_relative(0, 0, 0.03)"], "position": [0.6, 0.0] }}

User command: "move to position (0.5, 0.1)"
Output: {{ "actions": ["move_to"], "position": [0.5, 0.1] }}

User command: "move forward 5 cm and open gripper"
Output: {{ "actions": ["move_relative(0.05, 0, 0)", "open_gripper"], "position": [0.6, 0.0] }}

User command: "close gripper then lift by 0.1"
Output: {{ "actions": ["close_gripper", "lift(0.1)"], "position": [0.6, 0.0] }}

---
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
            
            # Check if response is empty
            if not message_content or message_content.strip() == "":
                print(f"GPT API error: Empty response from API")
                return {
                    "actions": [],
                    "position": [0.6, 0.0],
                    "delta": [0, 0, 0]
                }
            
            # Try to extract JSON if there's extra text
            message_content = message_content.strip()
            
            # Find JSON object in response (in case there's extra text)
            start_idx = message_content.find('{')
            end_idx = message_content.rfind('}') + 1
            
            if start_idx != -1 and end_idx > start_idx:
                json_str = message_content[start_idx:end_idx]
            else:
                json_str = message_content
            
            output = json.loads(json_str)
            return output

        except json.JSONDecodeError as e:
            print(f"GPT API JSON parsing error: {e}")
            print(f"Response was: {message_content if 'message_content' in locals() else 'N/A'}")
            return {
                "actions": [],
                "position": [0.6, 0.0],
                "delta": [0, 0, 0]
            }
        except Exception as e:
            print(f"GPT API error: {e}")
            return {
                "actions": [],
                "position": [0.6, 0.0],
                "delta": [0, 0, 0]
            }

