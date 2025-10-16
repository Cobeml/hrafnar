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
from PIL import Image, ImageDraw, ImageFont
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import re
import cv2
import numpy as np

from drone_controller import DroneController
from yolo_tracker import YOLOTracker
import threading


SYSTEM_PROMPT = """You are an autonomous emergency response drone searching for people who may need assistance.

VISUAL GROUNDING - IMPORTANT:
The camera image you see has COLORED BOUNDING BOXES overlaid on detected people.
- Each box is labeled with: "ID:X POSITION CONFIDENCE"
- POSITION shows: LEFT, CENTER, or RIGHT (person's location in frame)
- Box colors are consistent per person ID (same person = same color)
- A center dot marks each person's center point

**CRITICAL: Before making ANY decision, you MUST:**
1. Look at the VISUAL bounding boxes in the image
2. Read the YOLO DETECTIONS text below the image
3. VERIFY they match: Do you see the boxes? Do they align with the text descriptions?
4. Base your spatial reasoning on BOTH the visual boxes AND the YOLO text

If you see a box labeled "ID:5 CENTER 85%" in the image, that person IS in the center of your view.
Trust the visual bounding boxes - they show the ground truth.

You have access to real-time YOLO detections showing:
- Number of people detected
- Their bounding box locations (LEFT/CENTER/RIGHT)
- Unique tracking IDs

Use this data to inform your decisions.

AVAILABLE TOOLS (these are the ONLY actions you can take):
1. move_forward(distance: float) - Move forward in the direction the camera faces (meters)
2. move_right(distance: float) - Move to the camera's right side (meters, negative = left)
3. move_backward(distance: float) - Move backward (opposite of camera direction)
4. move_left(distance: float) - Move to the camera's left side (meters)
5. climb(distance: float) - Climb up in meters (works from ground or air)
6. descend(distance: float) - Descend down in meters
7. rotate(degrees: float) - Rotate in degrees (positive = clockwise, negative = counter-clockwise)
8. land() - Land at current position (no parameters)
9. speak(text: str) - Speak to a person you have approached (ONLY use when engaging with someone)
10. end_conversation() - Mark conversation as complete and continue mission

IMPORTANT RULES - MOVEMENTS ARE CAMERA-RELATIVE:
- move_forward moves in the CAMERA'S DIRECTION (not global north)
- move_right moves to the CAMERA'S RIGHT (not global east)
- After rotating, movements automatically follow the new camera direction
- To move toward something you see: use move_forward (it will move toward what the camera sees)
- To ROTATE LEFT: use rotate with NEGATIVE degrees (e.g., rotate(-90))
- To ROTATE RIGHT: use rotate with POSITIVE degrees (e.g., rotate(90))
- To TAKEOFF: use climb() from any altitude (ground or air)

CRITICAL MOVEMENT DISTANCE RULES:
- NEVER move more than 5 meters in a single action (system enforces max 5m)
- Typical approach movements: 1-3 meters at a time
- If something seems far, break into multiple small movements
- Bounding box coordinates (bbox) are PIXELS, not meters - DO NOT use raw pixel numbers as distances
- If a person is visible in camera, they are within 10 meters - use SMALL movements only (1-3m)
- Moving 200 meters is absurd - that's two football fields! Stay reasonable
- When adjusting position: 0.5-2 meters is appropriate, NOT 50+ meters

SPATIAL REASONING (THINK BEFORE ACTING):
1. **Where is the person?** (left side / center / right side of frame, per YOLO detection)
2. **Which direction gets me CLOSER?**
   - Person in CENTER or "closer to center" → move_forward (approach them!)
   - Person on LEFT side → move_left OR rotate left to center them
   - Person on RIGHT side → move_right OR rotate right to center them
3. **Critical rule:** If person is already centered, move FORWARD, never left/right
4. **Avoid repetition:** Don't repeat the same action multiple times in a row

RESPONSE FORMAT:
VISUAL_CHECK: <What bounding boxes do I see in the image? List each: ID, position, color>
YOLO_CHECK: <What does the YOLO text say? Does it match what I see visually?>
GROUNDING: <Are the visual boxes and text aligned? Any discrepancies?>
THINKING: <step-by-step spatial reasoning following the steps above>
OBSERVATION: <what you see in the camera>
ACTION: <exact tool name>
PARAMS: {"distance": <number>} OR {"degrees": <number>}
REASONING: <why you chose this action based on your thinking>

Examples:

Example 1: Person in CENTER of frame (most important!)
VISUAL_CHECK: I see one green bounding box labeled "ID:1 CENTER 92%" in the middle of the image with a center dot.
YOLO_CHECK: YOLO text confirms: Person ID 1, Position: center of frame, Confidence: 92%
GROUNDING: ✓ Perfect alignment - visual box matches YOLO text exactly. Person is centered.
THINKING: YOLO shows person in center of frame. To approach them, I should move forward in the direction the camera faces. Moving left or right would move AWAY from them.
OBSERVATION: I see a person standing in the center of the frame.
ACTION: move_forward
PARAMS: {"distance": 2.0}
REASONING: Person is centered - moving forward approaches them directly.

Example 2: Person "closer to center"
THINKING: Person is near or at the center. They're already aligned with my camera, so I should move forward to get closer, NOT left or right.
OBSERVATION: The person is now closer to the center of the frame.
ACTION: move_forward
PARAMS: {"distance": 1.5}
REASONING: Person is centered, so forward movement approaches them.

Example 3: Person on LEFT side
VISUAL_CHECK: I see one blue bounding box labeled "ID:3 LEFT 88%" on the left portion of the image.
YOLO_CHECK: YOLO text confirms: Person ID 3, Position: left side of frame, Confidence: 88%
GROUNDING: ✓ Aligned - visual box on left matches YOLO description.
THINKING: Person is on the left side of frame. I can rotate left to center them first, then move forward.
OBSERVATION: I see a person on the left side of the frame.
ACTION: rotate
PARAMS: {"degrees": -30}
REASONING: Rotate left to center the person for better approach angle.

Example 4: Person on RIGHT side
THINKING: Person is on the right. Rotate right to center them.
OBSERVATION: I see a person on the right side of the frame.
ACTION: rotate
PARAMS: {"degrees": 30}
REASONING: Rotate right to center the person before approaching.

Example 5: No person visible
VISUAL_CHECK: No bounding boxes visible in the image.
YOLO_CHECK: YOLO text confirms: No people detected
GROUNDING: ✓ Aligned - no visual boxes and no detections in text.
THINKING: No people detected. Should search by rotating to scan the area.
OBSERVATION: I see no people in the camera view.
ACTION: rotate
PARAMS: {"degrees": 45}
REASONING: Rotate to search for people in other directions.

Always check your camera view before moving! Movements follow where the camera points.
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
            bnb_4bit_compute_dtype=torch.bfloat16  # Use bfloat16 instead of float16 to avoid numerical instability
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

        # Initialize drone controller
        print("Connecting to drone...")
        self.drone = DroneController()

        # Camera
        self.bridge = CvBridge()
        self.latest_image = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            ROSImage,
            '/drone1/camera/color/image_raw',
            self.image_callback,
            qos_profile  # Use BEST_EFFORT
        )

        print("✅ VLM Drone Controller ready!")

        # Start YOLO tracker in same container
        print("Starting YOLO tracker...")
        self.yolo_tracker = YOLOTracker()

    def speak(self, text):
        """Drone speaks (simulated - prints for now)"""
        print(f"\n🔊 DRONE: {text}\n")

    def _annotate_image_with_detections(self, pil_image, detections):
        """
        Draw bounding boxes and labels on image for visual grounding.
        This helps the VLM visually align YOLO detections with what it sees.
        """
        if not detections:
            return pil_image

        # Convert PIL to CV2 for drawing
        img_cv = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

        for det in detections:
            bbox = det.get('bbox', [0, 0, 640, 480])
            track_id = det.get('id', -1)
            confidence = det.get('confidence', 0)

            x1, y1, x2, y2 = bbox

            # Color-code by track ID (cycle through colors)
            colors = [
                (0, 255, 0),    # Green
                (255, 0, 0),    # Blue
                (0, 0, 255),    # Red
                (255, 255, 0),  # Cyan
                (255, 0, 255),  # Magenta
            ]
            color = colors[track_id % len(colors)] if track_id >= 0 else (128, 128, 128)

            # Draw bounding box (thick)
            cv2.rectangle(img_cv, (x1, y1), (x2, y2), color, 3)

            # Determine position label
            center_x = (x1 + x2) / 2
            img_width = img_cv.shape[1]
            if center_x < img_width * 0.33:
                position = "LEFT"
            elif center_x > img_width * 0.67:
                position = "RIGHT"
            else:
                position = "CENTER"

            # Draw label background
            label = f"ID:{track_id} {position} {confidence:.0%}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            font_thickness = 2
            (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)

            cv2.rectangle(img_cv, (x1, y1 - text_h - 10), (x1 + text_w + 5, y1), color, -1)
            cv2.putText(img_cv, label, (x1 + 2, y1 - 5), font, font_scale, (255, 255, 255), font_thickness)

            # Draw center point
            center_x_int = int(center_x)
            center_y_int = int((y1 + y2) / 2)
            cv2.circle(img_cv, (center_x_int, center_y_int), 5, color, -1)

        # Convert back to PIL RGB
        annotated_pil = Image.fromarray(cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB))
        return annotated_pil

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

        # Get YOLO detections
        detections = self.yolo_tracker.get_detections()

        # Annotate image with bounding boxes for visual grounding
        annotated_image = self._annotate_image_with_detections(self.latest_image, detections)

        # Format YOLO detections for the prompt (with semantic descriptions, not raw pixels)
        detections_text = ""
        if detections:
            detections_text = f"\n\nCURRENT YOLO DETECTIONS:\n"
            detections_text += f"Total people detected: {len(detections)}\n"
            for det in detections:
                # Convert bbox to semantic position description
                bbox = det.get('bbox', [0, 0, 640, 480])
                center_x = (bbox[0] + bbox[2]) / 2
                img_width = 640  # Typical camera width

                if center_x < img_width * 0.33:
                    position = "left side of frame"
                elif center_x > img_width * 0.67:
                    position = "right side of frame"
                else:
                    position = "center of frame"

                detections_text += f"- Person ID {det.get('id', 'unknown')}: "
                detections_text += f"Position: {position}, "
                detections_text += f"Confidence: {det.get('confidence', 0):.0%}\n"
        else:
            detections_text = "\n\nCURRENT YOLO DETECTIONS: No people detected\n"

        # Create prompt with system instructions
        # Use annotated image so VLM can see bounding boxes overlaid on the camera view
        messages = [{
            "role": "user",
            "content": [
                {"type": "image", "image": annotated_image},
                {"type": "text", "text": f"{SYSTEM_PROMPT}{detections_text}\nUser Command: {user_command}"}
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
                max_new_tokens=512,  # Increased for better chain-of-thought reasoning
                do_sample=True,  # CRITICAL: Sampling avoids endless repetitions (per Qwen docs)
                temperature=0.7,  # Qwen recommended for non-thinking mode
                top_p=0.8,       # Qwen recommended
                top_k=20,        # Qwen recommended
                repetition_penalty=1.05  # Reduce repetitions
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
        """
        Extract action, params, observation, reasoning from VLM response.
        Handles both plain and markdown formatting (**ACTION:** vs ACTION:)
        """
        result = {
            'observation': '',
            'action': None,
            'params': {},
            'reasoning': ''
        }

        lines = response.split('\n')
        for line in lines:
            # Strip markdown formatting (**, *) and whitespace
            line = line.strip().strip('*').strip()

            if line.upper().startswith('OBSERVATION:'):
                result['observation'] = line.split(':', 1)[1].strip()
            elif line.upper().startswith('ACTION:'):
                action_text = line.split(':', 1)[1].strip().lower()
                # Remove any remaining markdown or whitespace
                result['action'] = action_text.strip('*').strip()
            elif line.upper().startswith('PARAMS:'):
                try:
                    params_str = line.split(':', 1)[1].strip()
                    # Try to parse as JSON
                    result['params'] = json.loads(params_str)
                except:
                    # Fallback: extract distance from text
                    match = re.search(r'(\d+\.?\d*)', params_str)
                    if match:
                        result['params'] = {'distance': float(match.group(1))}
            elif line.upper().startswith('REASONING:'):
                result['reasoning'] = line.split(':', 1)[1].strip()

        # Fallback regex patterns if structured parsing failed
        if not result['action']:
            # Try to find action in natural text (e.g., "move_forward")
            action_pattern = r'\b(move_forward|move_backward|move_right|move_left|climb|descend|rotate|land|speak|end_conversation)\b'
            match = re.search(action_pattern, response.lower())
            if match:
                result['action'] = match.group(1)
                print(f"⚠️  Used regex fallback to extract action: {result['action']}")

        if not result['params'] and result['action'] in ['move_forward', 'move_backward', 'move_right', 'move_left', 'climb', 'descend']:
            # Try to find distance near the action
            distance_pattern = r'(\d+\.?\d*)\s*m(?:eter)?s?'
            match = re.search(distance_pattern, response)
            if match:
                result['params'] = {'distance': float(match.group(1))}
                print(f"⚠️  Used regex fallback to extract distance: {result['params']['distance']}")

        if not result['params'] and result['action'] == 'rotate':
            # Try to find degrees
            degrees_pattern = r'(-?\d+\.?\d*)\s*(?:degree|deg|°)'
            match = re.search(degrees_pattern, response)
            if match:
                result['params'] = {'degrees': float(match.group(1))}
                print(f"⚠️  Used regex fallback to extract degrees: {result['params']['degrees']}")

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
        elif action == 'move_backward':
            self.drone.move_backward(distance)
            return f"Moved backward {distance}m"
        elif action == 'move_right':
            self.drone.move_right(distance)
            direction = "right" if distance > 0 else "left"
            return f"Moved {direction} {abs(distance)}m"
        elif action == 'move_left':
            self.drone.move_left(distance)
            return f"Moved left {distance}m"
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
        elif action == 'speak':
            text = params.get('text', 'Hello')
            self.speak(text)
            return f"Spoke: {text}"
        elif action == 'end_conversation':
            return "conversation_ended"  # Special signal
        else:
            return f"Unknown action: {action}. Available: move_forward, move_backward, move_right, move_left, climb, descend, rotate, land, speak, end_conversation"


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
