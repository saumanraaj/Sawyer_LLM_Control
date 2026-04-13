#!/usr/bin/env python
import rospy
from sawyer_action import sawyer_actions
from gpt import gpt_api 
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import json

from gpt import gpt_api

class gpt_controller():
    def __init__(self):
        
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.gpt = gpt_api()
        self.image_ready = False
        rospy.init_node("llm_executor_node", anonymous=True)
        # rospy.Subscriber("/llm/user_input", String, self.callback)
        rospy.Subscriber('/camera/color/image_raw',Image,self.rgb_cb)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_cb)
        self.sawyer = sawyer_actions() 
        self.main()
        rospy.spin()
    
    def main(self):
        
        str_input = "move to the blue cube"
        if self.image_ready:
            print("[DEBUG] Performing action with input:", str_input)
            data = self.extract_args(str_input)
            print("[DEBUG] Parsed data after extract_args:", data)
            output = self.gpt.get_vlm_output(self.rgb_image, data)
            print("[DEBUG] Output received from VLM:", output)  # ADD THIS
            self.sawyer.execute_sequence(output, self.depth_image)
    
    def rgb_cb(self, img_msg):
        self.image_ready = True
        self.rgb_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    def depth_cb(self, img_msg):
        # Cache latest depth image (in meters)
        self.depth_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')


    def extract_args(self,action_str):
        print(action_str)
        """
        Safely parse GPT JSON output for robot control.

        Args:
            output_text (str): Raw text output from GPT (expected to be JSON).

        Returns:
            dict: Parsed JSON dictionary with 'position', 'lift_height', and 'actions' keys.
        """
        try:
            print(action_str)
            parsed_data = json.loads(action_str)
            
            # Validate important fields
            if not isinstance(parsed_data.get('position', None), list) or len(parsed_data['position']) != 2:
                rospy.logwarn("Invalid or missing 'position' field.")
                parsed_data['position'] = [0.0, 0.0]
            
            if not isinstance(parsed_data.get('lift_height', None), (int, float)):
                rospy.logwarn("Invalid or missing 'lift_height' field.")
                parsed_data['lift_height'] = 0.0
            
            if not isinstance(parsed_data.get('actions', None), list):
                rospy.logwarn("Invalid or missing 'actions' field.")
                parsed_data['actions'] = []
            
            return parsed_data

        except json.JSONDecodeError as e:
            rospy.logerr(f"JSON parsing error: {e}")
            return {
                "position": [0.0, 0.0],
                "lift_height": 0.0,
                "actions": []
            }

    def chk_bounds(self,point): 
        
        u = point[0]
        v = point[1]
        z = point[2]
        
        if z <= 0.0 or np.isnan(z):
            rospy.logwarn("Invalid depth at pixel (%d,%d): %f", u, v, z)
            return
        
        h, w = self.depth_image.shape
        
        if not (0 <= u < w and 0 <= v < h):
            rospy.logerr("Pixel out of bounds: u=%d, v=%d", u, v)
            return


# ----------- Node Init -----------

if __name__ == "__main__":
    gpt_controller()
