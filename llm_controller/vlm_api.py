#!/usr/bin/env python3
"""
VLM Drone API
Receives text commands from Mac client, executes with or without vision
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import threading
import rclpy
from vlm_drone_controller import VLMDroneController
import re

app = FastAPI(title="VLM Drone API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global controller
controller = None

def parse_command_fallback(text_command):
    """
    Fallback command parser when vision isn't available
    Extracts action and parameters from natural language
    """
    text = text_command.lower()
    
    # Extract numbers
    numbers = re.findall(r'\d+\.?\d*', text)
    distance = float(numbers[0]) if numbers else 1.0
    
    # Detect action
    if 'takeoff' in text or 'take off' in text:
        return {
            'observation': '',
            'action': 'climb',
            'params': {'distance': distance if distance > 0.1 else 1.5},
            'reasoning': 'Parsed takeoff command'
        }
    elif 'land' in text:
        return {
            'observation': '',
            'action': 'land',
            'params': {},
            'reasoning': 'Parsed land command'
        }
    elif 'rotate' in text or 'turn' in text:
        degrees = distance if distance > 0 else 90
        if 'left' in text or 'counter' in text:
            degrees = -degrees
        return {
            'observation': '',
            'action': 'rotate',
            'params': {'degrees': degrees},
            'reasoning': f'Parsed rotation command'
        }
    elif 'climb' in text or 'up' in text or 'rise' in text:
        return {
            'observation': '',
            'action': 'climb',
            'params': {'distance': distance if distance > 0.1 else 0.5},
            'reasoning': 'Parsed climb command'
        }
    elif 'descend' in text or 'down' in text or 'lower' in text:
        return {
            'observation': '',
            'action': 'descend',
            'params': {'distance': distance if distance > 0.1 else 0.5},
            'reasoning': 'Parsed descend command'
        }
    elif 'forward' in text or 'ahead' in text:
        return {
            'observation': '',
            'action': 'move_forward',
            'params': {'distance': distance if distance > 0.1 else 2.0},
            'reasoning': 'Parsed forward movement'
        }
    elif 'backward' in text or 'back' in text:
        return {
            'observation': '',
            'action': 'move_forward',
            'params': {'distance': -(distance if distance > 0.1 else 2.0)},
            'reasoning': 'Parsed backward movement'
        }
    elif 'left' in text:
        return {
            'observation': '',
            'action': 'move_right',
            'params': {'distance': -(distance if distance > 0.1 else 2.0)},
            'reasoning': 'Parsed left movement'
        }
    elif 'right' in text:
        return {
            'observation': '',
            'action': 'move_right',
            'params': {'distance': distance if distance > 0.1 else 2.0},
            'reasoning': 'Parsed right movement'
        }
    else:
        return {
            'observation': '',
            'action': None,
            'params': {},
            'reasoning': f'Could not parse command: {text_command}'
        }

@app.on_event("startup")
async def startup():
    global controller
    # Initialize in thread
    def init_ros():
        global controller
        rclpy.init()
        controller = VLMDroneController()
        
        # Spin both VLM and YOLO together
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(controller)
        executor.add_node(controller.yolo_tracker)
        executor.spin()
    
    threading.Thread(target=init_ros, daemon=True).start()
    
    # Wait for controller
    import time
    time.sleep(10)  # Wait for VLM to load

@app.post("/command")
async def process_command(text_command: str):
    """Process text command with VLM (or fallback if no vision)"""
    if not controller:
        return {"error": "Controller not ready"}
    
    try:
        # Try VLM with vision first
        action_data = controller.process_command(text_command)
        
        # Check if vision was available
        if action_data.get('observation') == 'No camera feed':
            print("⚠️  No camera feed, using fallback parser")
            action_data = parse_command_fallback(text_command)
        
        # Execute action
        result = controller.execute_action(action_data)
        
        return {
            "command": text_command,
            "observation": action_data.get('observation', ''),
            "action": action_data.get('action', ''),
            "params": action_data.get('params', {}),
            "reasoning": action_data.get('reasoning', ''),
            "result": result,
            "response": f"{action_data.get('observation', '')} {result}"
        }
    except Exception as e:
        print(f"❌ Error processing command: {e}")
        import traceback
        traceback.print_exc()
        
        # Try fallback parser as last resort
        try:
            action_data = parse_command_fallback(text_command)
            result = controller.execute_action(action_data)
            return {
                "command": text_command,
                "response": f"[FALLBACK MODE] {result}",
                "action": action_data['action'],
                "params": action_data['params']
            }
        except:
            return {"error": str(e), "response": f"Error: {str(e)}"}

@app.get("/status")
async def get_status():
    """Get drone status"""
    if not controller:
        return {"status": "initializing"}
    
    pos = controller.drone.get_position()
    return {
        "position": pos,
        "armed": controller.drone.armed,
        "offboard": controller.drone.offboard_mode,
        "camera_active": controller.latest_image is not None
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8002)
