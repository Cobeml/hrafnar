#!/usr/bin/env python3
"""
High-level PX4 Drone Controller for LLM integration
"""

from pymavlink import mavutil
import time
import math

class DroneController:
    def __init__(self, connection_string='udp:127.0.0.1:14540'):
        """Connect to PX4 via MAVLink"""
        print(f"Connecting to PX4 at {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("✅ Connected to PX4!")
        
        self.current_altitude = 0
        self.armed = False
        self.offboard_mode = False
        
    def set_mode_offboard(self):
        """Set PX4 to OFFBOARD mode"""
        # PX4 custom mode for OFFBOARD
        PX4_MODE_OFFBOARD = 6
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_MODE_OFFBOARD,
            0, 0, 0, 0, 0)
        
        print("🎮 Setting OFFBOARD mode...")
        time.sleep(1)
        self.offboard_mode = True
        
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
        
    def takeoff(self, altitude=1.5):
        """Takeoff to specified altitude"""
        print(f"🚁 Taking off to {altitude}m...")
        
        # Start streaming setpoints BEFORE arming/offboard
        # PX4 requires setpoint stream before entering OFFBOARD
        for i in range(10):
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                0, 0, 0,  # Stay at current position
                0, 0, 0,
                0, 0, 0,
                0, 0)
            time.sleep(0.05)
        
        if not self.armed:
            self.arm()
            
        if not self.offboard_mode:
            self.set_mode_offboard()
        
        # Now climb to target altitude
        for i in range(100):
            progress = min(1.0, i / 50.0)
            target_z = -altitude * progress  # NED: negative is up
            
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                0, 0, target_z,
                0, 0, 0,
                0, 0, 0,
                0, 0)
            time.sleep(0.1)
        
        self.current_altitude = altitude
        print(f"✅ Reached {altitude}m")
        return f"Takeoff complete at {altitude}m"
        
    def move_forward(self, distance=2.0, speed=0.5):
        """Move forward by distance"""
        print(f"➡️  Moving forward {distance}m at {speed}m/s...")
        
        duration = distance / speed
        steps = int(duration * 10)
        
        for i in range(steps):
            progress = (i + 1) / steps
            x_pos = distance * progress
            
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                x_pos, 0, -self.current_altitude,
                0, 0, 0,
                0, 0, 0,
                0, 0)
            time.sleep(0.1)
        
        print(f"✅ Moved forward {distance}m")
        return f"Moved forward {distance}m"
        
    def rotate(self, degrees):
        """Rotate by degrees"""
        print(f"🔄 Rotating {degrees}°...")
        
        yaw_rad = math.radians(degrees)
        steps = int(abs(degrees) / 5)  # 5 degrees per step
        
        for i in range(steps):
            progress = (i + 1) / steps
            current_yaw = yaw_rad * progress
            
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000100111111000,  # Use yaw
                0, 0, -self.current_altitude,
                0, 0, 0,
                0, 0, 0,
                current_yaw, 0)
            time.sleep(0.1)
        
        print(f"✅ Rotated {degrees}°")
        return f"Rotated {degrees}°"
        
    def land(self):
        """Land the drone"""
        print("🛬 Landing...")
        
        # Descend gradually
        steps = int(self.current_altitude * 10)
        for i in range(steps):
            progress = (i + 1) / steps
            z_pos = -self.current_altitude * (1 - progress)
            
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                0, 0, z_pos,
                0, 0, 0,
                0, 0, 0,
                0, 0)
            time.sleep(0.1)
        
        time.sleep(2)
        
        # Disarm
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)
        
        print("🔒 Disarmed")
        self.armed = False
        self.current_altitude = 0
        return "Landed"

if __name__ == "__main__":
    drone = DroneController()
    
    print("\n=== Testing Drone Controller ===\n")
    print("⚠️  Make sure Isaac Sim simulation is PLAYING!\n")
    
    input("Press Enter to start takeoff...")
    drone.takeoff(altitude=1.5)
    
    input("Press Enter to move forward...")
    drone.move_forward(distance=2.0)
    
    input("Press Enter to rotate...")
    drone.rotate(degrees=90)
    
    input("Press Enter to land...")
    drone.land()
    
    print("\n✅ Mission complete!")
