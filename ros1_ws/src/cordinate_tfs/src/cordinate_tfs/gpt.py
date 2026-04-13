import openai
import base64
import os
import cv2


class gpt_api():
    def __init__(self):

        openai.api_key = os.getenv("OPENAI_API_KEY")

    def generate_prompt(self,instruction):
        return ( """You are a robot command generator.

                Given a user instruction, you must output a JSON object with:
                - "position": [x, y] image pixel coordinates (two floats)
                - "lift_height": how much to lift after picking (a float, in meters)
                - "actions": list of function names (with empty parentheses)

                Rules:
                - Always output valid JSON.
                - "position" must have exactly two floats.
                - "lift_height" must be a float.
                - "actions" must be a list of strings like "move_to()", "close_gripper()", "open_gripper()", "lift()".
                - Use only lowercase function names.
                - If no movement or position is needed, set position to [0.0, 0.0] and lift_height to 0.0.
                - Always complete all fields even if not used.

                ---

                Examples:

                Input: "Pick up the red cube"
                Output:
                {
                "position": [120, 240],
                "lift_height": 0.2,
                "actions": ["move_to()", "close_gripper()", "lift()"]
                }

                Input: "Grab the blue ball"
                Output:
                {
                "position": [180, 300],
                "lift_height": 0.25,
                "actions": ["move_to()", "close_gripper()", "lift()"]
                }

                Input: "Lift the object slightly"
                Output:
                {
                "position": [100, 200],
                "lift_height": 0.1,
                "actions": ["move_to()", "close_gripper()", "lift()"]
                }

                Input: "Move to the green box"
                Output:
                {
                "position": [220, 150],
                "lift_height": 0.0,
                "actions": ["move_to()"]
                }

                Input: "Open the gripper"
                Output:
                {
                "position": [0.0, 0.0],
                "lift_height": 0.0,
                "actions": ["open_gripper()"]
                }

                Input: "Close the gripper"
                Output:
                {
                "position": [0.0, 0.0],
                "lift_height": 0.0,
                "actions": ["close_gripper()"]
                }

                Input: "Lift the arm higher"
                Output:
                {
                "position": [0.0, 0.0],
                "lift_height": 0.3,
                "actions": ["lift()"]
                }

                Input: "Do nothing"
                Output:
                {
                "position": [0.0, 0.0],
                "lift_height": 0.0,
                "actions": []
                }

                ---

                If you cannot understand the instruction, output:
                {
                "position": [0.0, 0.0],
                "lift_height": 0.0,
                "actions": []
                }"""
        )

    def cv_image_to_base64(self,cv_image):

        """Encode OpenCV image (BGR) to base64 string (PNG format)."""
        success, buffer = cv2.imencode('.png', cv_image)
        if not success:
            raise RuntimeError("Failed to encode image to .png buffer")
        img_b64 = base64.b64encode(buffer).decode('utf-8')
        return img_b64

    def get_vlm_output(self,cv_img,instruction):
        
        img_b64= self.cv_image_to_base64(cv_img)
        prompt = self.generate_prompt(instruction)

        response = openai.chat.completions.create(
            model="gpt-4-turbo",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/png;base64,{img_b64}",
                                "detail": "auto"
                            }
                        }
                    ]
                }
            ],
            max_tokens=500,
            temperature=0.2,
        )
        print("sending openai api call",instruction)
        print("output",response)
        output = response.choices[0].message.content
        return eval(output)  # Use json.loads() if GPT returns strict JSON