#!/usr/bin/env python3
"""
Test VLM can see and understand drone camera feed
No movement - just vision testing
"""

import torch
from transformers import Qwen2VLForConditionalGeneration, AutoProcessor, BitsAndBytesConfig
from qwen_vl_utils import process_vision_info
from PIL import Image
import time
import sys

# Subscribe to ROS2 camera feed
sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2

class VisionTester(Node):
    def __init__(self):
        super().__init__('vision_tester')
        
        print("Loading Qwen2.5-VL-3B in 4-bit...")
        bnb_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16
        )
        
        self.model = Qwen2VLForConditionalGeneration.from_pretrained(
            "Qwen/Qwen2.5-VL-3B-Instruct",
            quantization_config=bnb_config,
            device_map="auto"
        )
        
        self.processor = AutoProcessor.from_pretrained(
            "Qwen/Qwen2.5-VL-3B-Instruct",
            min_pixels=256*28*28,
            max_pixels=512*28*28
        )
        
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Subscribe to camera
        self.sub = self.create_subscription(
            ROSImage,
            '/drone1/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        print("✅ VLM loaded! Waiting for camera feed...")
        
    def image_callback(self, msg):
        """Store latest frame"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.latest_image = Image.fromarray(cv_image)
        
    def analyze_scene(self, question="What do you see in this image?"):
        """Ask VLM about current camera view"""
        if self.latest_image is None:
            return "No camera feed"
            
        messages = [{
            "role": "user",
            "content": [
                {"type": "image", "image": self.latest_image},
                {"type": "text", "text": question}
            ]
        }]
        
        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        image_inputs, _ = process_vision_info(messages)
        inputs = self.processor(text=[text], images=image_inputs, return_tensors="pt").to("cuda")
        
        start = time.time()
        with torch.inference_mode():
            output_ids = self.model.generate(**inputs, max_new_tokens=128)
        
        response = self.processor.batch_decode(output_ids, skip_special_tokens=True)[0]
        latency = time.time() - start
        
        print(f"\n🤖 VLM Response ({latency:.2f}s):\n{response}\n")
        return response

def main():
    rclpy.init()
    tester = VisionTester()
    
    print("\n=== VLM Vision Test ===\n")
    
    # Wait for camera feed
    while tester.latest_image is None:
        rclpy.spin_once(tester, timeout_sec=0.1)
        time.sleep(0.1)
    
    print("📸 Camera feed received!\n")
    
    # Test questions
    questions = [
        "Describe what you see in detail.",
        "Are there any people in this image?",
        "What objects can you identify?",
        "Is this indoors or outdoors?"
    ]
    
    for q in questions:
        print(f"❓ Question: {q}")
        tester.analyze_scene(q)
        time.sleep(2)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
