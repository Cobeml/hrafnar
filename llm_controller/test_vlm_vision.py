#!/usr/bin/env python3
"""
Test VLM - Qwen2.5-VL
Based on official Qwen documentation
"""

import torch
from transformers import AutoProcessor, AutoModelForVision2Seq, BitsAndBytesConfig
from qwen_vl_utils import process_vision_info
from PIL import Image
import time
import sys

# ROS2 imports
sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge

class VisionTester(Node):
    def __init__(self):
        super().__init__('vision_tester')
        
        # Use official Qwen model loading (with trust_remote_code)
        model_name = "Qwen/Qwen2-VL-2B-Instruct"
        
        print(f"Loading {model_name} in 4-bit...")
        
        # 4-bit quantization
        bnb_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16
        )
        
        # Load with AutoModel (Qwen's official way)
        self.model = AutoModelForVision2Seq.from_pretrained(
            model_name,
            quantization_config=bnb_config,
            device_map="auto",
            trust_remote_code=True  # Required for Qwen
        )
        
        self.processor = AutoProcessor.from_pretrained(
            model_name,
            trust_remote_code=True
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
        
        # Process inputs
        text = self.processor.apply_chat_template(
            messages, 
            tokenize=False, 
            add_generation_prompt=True
        )
        
        image_inputs, video_inputs = process_vision_info(messages)
        
        inputs = self.processor(
            text=[text],
            images=image_inputs,
            videos=video_inputs,
            padding=True,
            return_tensors="pt"
        ).to("cuda")
        
        # Generate
        start = time.time()
        with torch.inference_mode():
            output_ids = self.model.generate(
                **inputs,
                max_new_tokens=128
            )
        
        # Decode
        generated_ids = [
            output_ids[len(input_ids):]
            for input_ids, output_ids in zip(inputs.input_ids, output_ids)
        ]
        response = self.processor.batch_decode(
            generated_ids,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=True
        )[0]
        
        latency = time.time() - start
        
        print(f"\n🤖 VLM Response ({latency:.2f}s):\n{response}\n")
        return response

def main():
    rclpy.init()
    tester = VisionTester()
    
    print("\n=== VLM Vision Test ===\n")
    
    # Wait for camera
    print("Waiting for camera feed...")
    while tester.latest_image is None:
        rclpy.spin_once(tester, timeout_sec=0.1)
        time.sleep(0.1)
    
    print("📸 Camera feed received!\n")
    
    # Test question
    tester.analyze_scene("Describe what you see in this image.")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
