#!/usr/bin/env python3
"""
High-level PX4 Drone Controller for LLM integration
Provides simple commands: takeoff, land, move, rotate
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
        
        # Current state
        self.current_altitude = 0
        self.armed = False
        
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
        return "Armed"
        
    def disarm(self):
        """Disarm the drone"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)
        
        print("🔒 Disarming...")
        self.armed = False
        time.sleep(1)
        return "Disarmed"
        
    def takeoff(self, altitude=1.5):
        """
        Takeoff to specified altitude (default 1.5m = eye level)
        """
        if not self.armed:
            self.arm()
        
        # For PX4, use position setpoints instead of takeoff command
        # Set to OFFBOARD mode by sending setpoints
        print(f"🚁 Taking off to {altitude}m...")
        
        # Send setpoints to climb
        for i in range(50):  # Send for 5 seconds
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # Position mask
                0, 0, -altitude,  # NED: z is negative for up
                0, 0, 0,
                0, 0, 0,
                0, 0)
            time.sleep(0.1)
        
        self.current_altitude = altitude
        return f"Takeoff complete at {altitude}m"
        
    def land(self):
        """Land the drone"""
        print("🛬 Landing...")
        
        # Gradually descend to ground
        for alt in range(int(self.current_altitude * 10), -1, -1):
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                0, 0, -alt/10.0,
                0, 0, 0,
                0, 0, 0,
                0, 0)
            time.sleep(0.1)
        
        time.sleep(2)
        self.disarm()
        return "Landing complete"
        
    def move_forward(self, distance=1.0, speed=0.5):
        """Move forward by distance (meters)"""
        print(f"➡️  Moving forward {distance}m...")
        return self._move_relative(distance, 0, 0, speed)
        
    def move_backward(self, distance=1.0, speed=0.5):
        """Move backward by distance (meters)"""
        print(f"⬅️  Moving backward {distance}m...")
        return self._move_relative(-distance, 0, 0, speed)
        
    def move_left(self, distance=1.0, speed=0.5):
        """Move left by distance (meters)"""
        print(f"⬅️  Moving left {distance}m...")
        return self._move_relative(0, -distance, 0, speed)
        
    def move_right(self, distance=1.0, speed=0.5):
        """Move right by distance (meters)"""
        print(f"➡️  Moving right {distance}m...")
        return self._move_relative(0, distance, 0, speed)
        
    def rotate(self, degrees):
        """Rotate heading by degrees (positive = clockwise)"""
        print(f"🔄 Rotating {degrees}°...")
        
        yaw_rad = math.radians(degrees)
        duration = abs(degrees) / 45.0  # ~45 deg/sec
        
        # Send yaw setpoint repeatedly
        for i in range(int(duration * 10)):
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,  # Yaw angle mask
                0, 0, -self.current_altitude,
                0, 0, 0,
                0, 0, 0,
                yaw_rad, 0)
            time.sleep(0.1)
        
        return f"Rotated {degrees} degrees"
        
    def _move_relative(self, x, y, z, speed):
        """Internal: Move relative to current position"""
        duration = abs(max(x, y, z)) / speed
        steps = int(duration * 10)
        
        for i in range(steps):
            progress = (i + 1) / steps
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111111000,
                x * progress, y * progress, -(self.current_altitude + z * progress),
                0, 0, 0,
                0, 0, 0,
                0, 0)
            time.sleep(0.1)
        
        return f"Moved relative position ({x}, {y}, {z})"

if __name__ == "__main__":
    # Test the controller
    drone = DroneController()
    
    print("\n=== Testing Drone Controller ===\n")
    drone.takeoff(altitude=1.5)
    time.sleep(3)
    drone.move_forward(distance=2.0)
    time.sleep(3)
    drone.rotate(degrees=90)
    time.sleep(3)
    drone.land()
