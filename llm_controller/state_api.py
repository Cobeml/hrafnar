#!/usr/bin/env python3
"""
Lightweight API to serve drone state
No OpenAI - just serves data
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
import threading

app = FastAPI(title="Drone State API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global state
drone_state = {
    'position': None,
    'velocity': None,
    'gps': None,
    'armed': False,
    'last_update': None
}

class StateCollector(Node):
    def __init__(self):
        super().__init__('state_collector')
        
        self.create_subscription(PoseStamped, '/drone1/state/pose', self.pose_cb, 10)
        self.create_subscription(TwistStamped, '/drone1/state/twist', self.twist_cb, 10)
        self.create_subscription(NavSatFix, '/drone1/sensors/gps', self.gps_cb, 10)
        
    def pose_cb(self, msg):
        drone_state['position'] = {
            'x': round(msg.pose.position.x, 2),
            'y': round(msg.pose.position.y, 2),
            'z': round(msg.pose.position.z, 2)
        }
        
    def twist_cb(self, msg):
        drone_state['velocity'] = {
            'x': round(msg.twist.linear.x, 2),
            'y': round(msg.twist.linear.y, 2),
            'z': round(msg.twist.linear.z, 2)
        }
        
    def gps_cb(self, msg):
        drone_state['gps'] = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }

@app.get("/state")
def get_state():
    """Get current drone state"""
    return drone_state

@app.get("/")
def root():
    return {"status": "Drone State API", "endpoint": "/state"}

def ros_thread():
    rclpy.init()
    collector = StateCollector()
    rclpy.spin(collector)

if __name__ == "__main__":
    # Start ROS2 in background thread
    threading.Thread(target=ros_thread, daemon=True).start()
    
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
