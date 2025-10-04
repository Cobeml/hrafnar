#!/usr/bin/env python3
"""
VLM Drone API
Receives text commands from Mac client, executes with vision
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import threading
import rclpy
from vlm_drone_controller import VLMDroneController

app = FastAPI(title="VLM Drone API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global controller
controller = None

@app.on_event("startup")
async def startup():
    global controller
    # Initialize in thread
    def init_ros():
        global controller
        rclpy.init()
        controller = VLMDroneController()
        rclpy.spin(controller)
    
    threading.Thread(target=init_ros, daemon=True).start()
    
    # Wait for controller
    import time
    time.sleep(10)  # Wait for VLM to load

@app.post("/command")
async def process_command(text_command: str):
    """Process text command with VLM"""
    if not controller:
        return {"error": "Controller not ready"}
    
    try:
        # Process with VLM - returns a dict
        action_data = controller.process_command(text_command)
        
        # Execute - also needs dict
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
        "offboard": controller.drone.offboard_mode
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8002)  # Changed to 8002

