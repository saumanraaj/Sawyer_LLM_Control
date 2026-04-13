#!/usr/bin/env python3
import os
import json
from openai import OpenAI

# ----------- SETUP OPENAI CLIENT -----------

# Make sure OPENAI_API_KEY is exported in your shell:
# export OPENAI_API_KEY="sk-xxxx..."
client = OpenAI()

# ----------- PROMPT TEMPLATE -----------

PROMPT_TEMPLATE = """
You are controlling a 7-DOF Sawyer robot arm in simulation. The user will give you high-level movement commands.
Convert the instruction into structured robot actions that can be interpreted and executed by a ROS node.

Respond ONLY with a JSON block in this format:

{{
  "actions": [
    "move_to(x, y, z)",
    "close_gripper()",
    "lift(delta_z)",
    "open_gripper()"
  ]
}}

Examples:
Instruction: "Move to (0.5, 0.2, 0.15)"
→ {{
  "actions": ["move_to(0.5, 0.2, 0.15)"]
}}

Instruction: "Move 10 cm to the right"
→ {{
  "actions": ["move_to(0.6, 0.0, 0.15)"]
}}

Now, respond to the following instruction:
Instruction: "{}"
"""

# ----------- QUERY FUNCTION -----------

def query_gpt(instruction):
    prompt = PROMPT_TEMPLATE.format(instruction)

    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You're a robotics command planner."},
            {"role": "user", "content": prompt}
        ],
        temperature=0.0
    )

    # Extract JSON from response
    text = response.choices[0].message.content
    try:
        start = text.index("{")
        end = text.rindex("}") + 1
        action_json = text[start:end]
        return json.loads(action_json)
    except Exception as e:
        print("Failed to extract JSON from GPT response.")
        print("Raw output:", text)
        raise e

# ----------- ENTRY POINT -----------

if __name__ == "__main__":
    print("🔧 GPT → Sawyer Control")
    instruction = input("Enter your command for the Sawyer robot: ")

    try:
        actions = query_gpt(instruction)
        with open("/tmp/command.json", "w") as f:
            json.dump(actions, f, indent=2)
        print("\nCommand saved to /tmp/command.json:")
        print(json.dumps(actions, indent=2))
    except Exception as e:
        print(f"Failed to generate valid action plan.\n{e}")
