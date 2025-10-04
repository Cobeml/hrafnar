#!/usr/bin/env python3
"""
PX4 Drone Controller with position verification
"""

from pymavlink import mavutil
import time
import math
import threading

class DroneController:
    def __init__(self, connection_string='udp:127.0.0.1:14540'):
        """Connect to PX4 via MAVLink"""
        print(f"Connecting to PX4 at {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("✅ Connected to PX4!")
        
        # Current state
        self.current_pos = [0, 0, 0]  # x, y, z in NED frame
        self.current_yaw = 0
        self.armed = False
        self.offboard_mode = False
        
        # Target setpoint (continuously sent)
        self.target_pos = [0, 0, -1.0]  # Default: hover at 1m
        self.target_yaw = 0
        
        # Start setpoint streaming thread
        self.streaming = False
        self.stream_thread = None
        
        # Start position listener
        self.listen_thread = threading.Thread(target=self._listen_position, daemon=True)
        self.listen_thread.start()
        
    def _listen_position(self):
        """Listen for position updates from PX4"""
        while True:
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg:
                self.current_pos = [msg.x, msg.y, msg.z]
                
    def _stream_setpoints(self):
        """Continuously stream setpoints to PX4 (required for OFFBOARD)"""
        while self.streaming:
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000100111111000,  # Use position + yaw
                self.target_pos[0], self.target_pos[1], self.target_pos[2],
                0, 0, 0,
                0, 0, 0,
                self.target_yaw, 0)
            time.sleep(0.05)  # 20Hz streaming
            
    def start_streaming(self):
        """Start continuous setpoint streaming"""
        if not self.streaming:
            self.streaming = True
            self.stream_thread = threading.Thread(target=self._stream_setpoints, daemon=True)
            self.stream_thread.start()
            print("📡 Started setpoint streaming")
            
    def stop_streaming(self):
        """Stop setpoint streaming"""
        self.streaming = False
        if self.stream_thread:
            self.stream_thread.join(timeout=1)
        print("📡 Stopped setpoint streaming")
        
    def get_position(self):
        """Get current position"""
        return {
            'x': self.current_pos[0],
            'y': self.current_pos[1], 
            'z': self.current_pos[2],
            'altitude': -self.current_pos[2]  # Convert NED to altitude
        }
        
    def arm(self):
        """Arm the drone"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        
        print("🔓 Arming...")
        time.sleep(2)
        self.armed = True
        
    def set_offboard_mode(self):
        """Set PX4 to OFFBOARD mode"""
        # Must be streaming setpoints BEFORE setting mode
        if not self.streaming:
            self.start_streaming()
            time.sleep(0.5)
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            6,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
            0, 0, 0, 0, 0)
        
        print("🎮 Setting OFFBOARD mode...")
        time.sleep(1)
        self.offboard_mode = True
        
    def goto_position(self, x, y, z, yaw=None, speed=0.5):
        """
        Go to position (NED frame)
        z is negative for altitude (z=-1.5 means 1.5m altitude)
        """
        if not self.offboard_mode:
            print("⚠️  Entering OFFBOARD mode...")
            self.set_offboard_mode()
            
        if not self.armed:
            self.arm()
        
        print(f"📍 Current: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f})")
        print(f"🎯 Target:  ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Calculate distance
        dx = x - self.current_pos[0]
        dy = y - self.current_pos[1]
        dz = z - self.current_pos[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if distance < 0.1:
            print("✅ Already at target")
            return
            
        # Update target (streaming thread will send it)
        self.target_pos = [x, y, z]
        if yaw is not None:
            self.target_yaw = yaw
        
        # Wait for arrival (check every 0.5s)
        duration = distance / speed
        for i in range(int(duration * 2)):
            time.sleep(0.5)
            current_distance = math.sqrt(
                (x - self.current_pos[0])**2 +
                (y - self.current_pos[1])**2 +
                (z - self.current_pos[2])**2
            )
            print(f"  Distance to target: {current_distance:.2f}m")
            
            if current_distance < 0.2:  # Within 20cm
                print("✅ Reached target!")
                break
        
        pos = self.get_position()
        print(f"📍 Final: ({pos['x']:.2f}, {pos['y']:.2f}, altitude={pos['altitude']:.2f}m)")
        
    def move_forward(self, distance=2.0):
        """Move forward (positive X in body frame)"""
        print(f"➡️  Moving forward {distance}m...")
        target_x = self.current_pos[0] + distance
        self.goto_position(target_x, self.current_pos[1], self.current_pos[2])
        
    def move_right(self, distance=2.0):
        """Move right (positive Y in NED)"""
        print(f"➡️  Moving right {distance}m...")
        target_y = self.current_pos[1] + distance
        self.goto_position(self.current_pos[0], target_y, self.current_pos[2])
        
    def climb(self, altitude_change=0.5):
        """Climb up (more negative Z)"""
        print(f"⬆️  Climbing {altitude_change}m...")
        target_z = self.current_pos[2] - altitude_change
        self.goto_position(self.current_pos[0], self.current_pos[1], target_z)
        
    def descend(self, altitude_change=0.5):
        """Descend down (less negative Z)"""
        print(f"⬇️  Descending {altitude_change}m...")
        target_z = self.current_pos[2] + altitude_change
        self.goto_position(self.current_pos[0], self.current_pos[1], target_z)
        
    def land(self):
        """Land at current position"""
        print("🛬 Landing...")
        self.goto_position(self.current_pos[0], self.current_pos[1], 0, speed=0.3)
        time.sleep(2)
        self.stop_streaming()
        
        # Disarm
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)
        print("🔒 Disarmed")
        self.armed = False

if __name__ == "__main__":
    drone = DroneController()
    
    print("\n=== PX4 Drone Controller Test ===\n")
    print("⚠️  Make sure Isaac Sim is PLAYING!\n")
    
    time.sleep(2)
    print(f"📍 Starting position: {drone.get_position()}")
    
    input("\nPress Enter to start OFFBOARD mode...")
    drone.set_offboard_mode()
    drone.arm()
    
    input("Press Enter to move forward 2m...")
    drone.move_forward(2.0)
    
    input("Press Enter to move right 2m...")
    drone.move_right(2.0)
    
    input("Press Enter to climb 0.5m...")
    drone.climb(0.5)
    
    input("Press Enter to land...")
    drone.land()
    
    print("\n✅ Test complete!")
