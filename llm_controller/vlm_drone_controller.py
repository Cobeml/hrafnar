#!/usr/bin/env python3
"""
VLM Drone Controller with Tool Calling
Qwen2.5-VL processes camera + commands → Executes drone movements
"""

import sys
sys.path.append('/app')

import torch
from transformers import AutoProcessor, AutoModelForVision2Seq, BitsAndBytesConfig
from qwen_vl_utils import process_vision_info
from PIL import Image
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import json
import re

from drone_controller import DroneController

SYSTEM_PROMPT = """You are an autonomous drone with vision and movement capabilities.

AVAILABLE TOOLS (these are the ONLY actions you can take):
1. move_forward(distance: float) - Move forward in meters
2. move_right(distance: float) - Move right in meters (negative = left)
3. climb(distance: float) - Climb up in meters (works from ground or air)
4. descend(distance: float) - Descend down in meters
5. rotate(degrees: float) - Rotate in degrees (positive = clockwise, negative = counter-clockwise)
6. land() - Land at current position (no parameters)

IMPORTANT RULES:
- To move LEFT: use move_right with NEGATIVE distance (e.g., move_right(-2.0))
- To move BACKWARD: use move_forward with NEGATIVE distance (e.g., move_forward(-2.0))
- To ROTATE LEFT: use rotate with NEGATIVE degrees (e.g., rotate(-90))
- To ROTATE RIGHT: use rotate with POSITIVE degrees (e.g., rotate(90))
- To TAKEOFF: use climb() from any altitude (ground or air)

RESPONSE FORMAT:
OBSERVATION: <what you see in the camera>
ACTION: <exact tool name>
PARAMS: {"distance": <number>} OR {"degrees": <number>}
REASONING: <why you chose this action>

Examples:
User: "Takeoff to 1.5 meters"
OBSERVATION: I am on the ground.
ACTION: climb
PARAMS: {"distance": 1.5}
REASONING: Need to climb from ground to 1.5m altitude.

User: "Turn left 90 degrees"
OBSERVATION: I see a wall ahead.
ACTION: rotate
PARAMS: {"degrees": -90}
REASONING: Rotating left (counter-clockwise) 90 degrees.

User: "Rotate right 45 degrees"
OBSERVATION: I want to face right.
ACTION: rotate
PARAMS: {"degrees": 45}
REASONING: Rotating clockwise 45 degrees.

User: "Move left 2 meters"
OBSERVATION: I see open space to the left.
ACTION: move_right
PARAMS: {"distance": -2.0}
REASONING: Moving left requires negative move_right.

Always check your camera view before moving!
"""


from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class VLMDroneController(Node):
    def __init__(self):
        super().__init__('vlm_drone_controller')
        
        # Load VLM
        print("Loading Qwen2.5-VL-3B...")
        bnb_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16
        )
        
        self.model = AutoModelForVision2Seq.from_pretrained(
            "Qwen/Qwen2.5-VL-3B-Instruct",
            quantization_config=bnb_config,
            device_map="auto",
            trust_remote_code=True
        )
        
        self.processor = AutoProcessor.from_pretrained(
            "Qwen/Qwen2.5-VL-3B-Instruct",
            trust_remote_code=True
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.create_subscription(
            ROSImage,
            '/drone1/camera/color/image_raw',
            self.image_callback,
            qos_profile  # Use BEST_EFFORT
        )
        
        # Initialize drone controller
        print("Connecting to drone...")
        self.drone = DroneController()
        
        # Camera
        self.bridge = CvBridge()
        self.latest_image = None
        self.create_subscription(
            ROSImage,
            '/drone1/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        print("✅ VLM Drone Controller ready!")
        
    def image_callback(self, msg):
        """Store latest camera frame"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.latest_image = Image.fromarray(cv_image)
        
    def process_command(self, user_command):
        """
        Process command with VLM vision + reasoning
        Returns: dict with action_data
        """
        if self.latest_image is None:
            return {
                'observation': "No camera feed",
                'action': None,
                'params': {},
                'reasoning': "Cannot proceed without vision"
            }
            
        # Create prompt with system instructions
        messages = [{
            "role": "user",
            "content": [
                {"type": "image", "image": self.latest_image},
                {"type": "text", "text": f"{SYSTEM_PROMPT}\n\nUser Command: {user_command}"}
            ]
        }]
        
        # Process with VLM
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
        
        # Generate response
        print("🧠 VLM processing...")
        start = time.time()
        with torch.inference_mode():
            output_ids = self.model.generate(
                **inputs,
                max_new_tokens=256
            )
        
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
        print(f"⏱️  VLM response time: {latency:.2f}s")
        print(f"\n🤖 VLM Response:\n{response}\n")
        
        # Parse response - returns a dict
        action_data = self._parse_response(response)
        
        # Make sure it's a dict (not a tuple)
        if not isinstance(action_data, dict):
            return {
                'observation': str(response),
                'action': None,
                'params': {},
                'reasoning': "Failed to parse response"
            }
        
        return action_data  # Return the dict directly
        
    def _parse_response(self, response):
        """Extract action, params, observation, reasoning"""
        result = {
            'observation': '',
            'action': None,
            'params': {},
            'reasoning': ''
        }
        
        lines = response.split('\n')
        for line in lines:
            line = line.strip()
            if line.startswith('OBSERVATION:'):
                result['observation'] = line.split('OBSERVATION:')[1].strip()
            elif line.startswith('ACTION:'):
                result['action'] = line.split('ACTION:')[1].strip().lower()
            elif line.startswith('PARAMS:'):
                try:
                    params_str = line.split('PARAMS:')[1].strip()
                    result['params'] = json.loads(params_str)
                except:
                    # Try to extract distance from text
                    match = re.search(r'(\d+\.?\d*)', params_str)
                    if match:
                        result['params'] = {'distance': float(match.group(1))}
            elif line.startswith('REASONING:'):
                result['reasoning'] = line.split('REASONING:')[1].strip()
        
        return result
        
    def execute_action(self, action_data):
        """Execute the parsed action"""
        action = action_data['action']
        params = action_data['params']
        
        if not action:
            return "No action detected"
            
        # Make sure drone is in OFFBOARD mode
        if not self.drone.offboard_mode:
            self.drone.set_offboard_mode()
            self.drone.arm()
        
        # Execute action
        distance = params.get('distance', 1.0)
        degrees = params.get('degrees', 0)
        
        if action == 'move_forward':
            self.drone.move_forward(distance)
            direction = "forward" if distance > 0 else "backward"
            return f"Moved {direction} {abs(distance)}m"
        elif action == 'move_right':
            self.drone.move_right(distance)
            direction = "right" if distance > 0 else "left"
            return f"Moved {direction} {abs(distance)}m"
        elif action == 'climb':
            self.drone.climb(distance)
            return f"Climbed {distance}m"
        elif action == 'descend':
            self.drone.descend(distance)
            return f"Descended {distance}m"
        elif action == 'rotate':
            self.drone.rotate(degrees)
            direction = "clockwise" if degrees > 0 else "counter-clockwise"
            return f"Rotated {direction} {abs(degrees)}°"
        elif action == 'land':
            self.drone.land()
            return "Landed successfully"
        else:
            return f"Unknown action: {action}. Available: move_forward, move_right, climb, descend, rotate, land"


def main():
    rclpy.init()
    
    controller = VLMDroneController()
    
    print("\n=== VLM Drone Controller Test ===\n")
    
    # Wait for camera
    print("Waiting for camera feed...")
    while controller.latest_image is None:
        rclpy.spin_once(controller, timeout_sec=0.1)
        time.sleep(0.1)
    print("✅ Camera ready!\n")
    
    # Test commands
    test_commands = [
        "Move forward 2 meters",
        "Climb up half a meter",
        "Land now"
    ]
    
    for cmd in test_commands:
        input(f"\nPress Enter to test: '{cmd}'...")
        
        # Process command
        action_data = controller.process_command(cmd)
        
        print(f"\n📋 Parsed Action:")
        print(f"   Observation: {action_data['observation']}")
        print(f"   Action: {action_data['action']}")
        print(f"   Params: {action_data['params']}")
        print(f"   Reasoning: {action_data['reasoning']}")
        
        # Execute
        result = controller.execute_action(action_data)
        print(f"✅ Result: {result}")
        
        time.sleep(2)
    
    print("\n✅ Test complete!")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
