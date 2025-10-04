#!/usr/bin/env python3
"""
Simple text API - VLM only
Audio processing happens on MacBook
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import sys
sys.path.append('/app')
from vlm_drone_controller import VLMDroneController
import asyncio

app = FastAPI(title="Drone VLM API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize drone controller (runs once)
drone_controller = None

@app.on_event("startup")
async def startup():
    global drone_controller
    drone_controller = VLMDroneController()
    print("✅ VLM Drone Controller ready!")

@app.post("/command")
async def process_command(text_command: str):
    """
    Process text command with VLM
    Returns: Text response
    """
    print(f"📝 Received command: {text_command}")
    
    # VLM processes and executes
    response = drone_controller.execute_mission(text_command)
    
    return {"response": response, "status": "success"}

@app.get("/status")
async def get_status():
    """Get drone status"""
    pos = drone_controller.drone.get_position()
    return {"position": pos, "armed": drone_controller.drone.armed}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
