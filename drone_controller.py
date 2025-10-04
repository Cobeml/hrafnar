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
        
    def set_mode(self, mode='OFFBOARD'):
        """Set flight mode"""
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        print(f"🎮 Mode set to {mode}")
        
    def takeoff(self, altitude=1.5):
        """
        Takeoff to specified altitude (default 1.5m = eye level)
        """
        if not self.armed:
            self.arm()
        
        self.set_mode('OFFBOARD')
        
        # Send takeoff command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)
        
        print(f"🚁 Taking off to {altitude}m...")
        self.current_altitude = altitude
        time.sleep(5)  # Wait for takeoff
        return f"Takeoff complete at {altitude}m"
        
    def land(self):
        """Land the drone"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0)
        
        print("🛬 Landing...")
        time.sleep(5)
        self.armed = False
        return "Landing complete"
        
    def move_forward(self, distance=1.0, speed=0.5):
        """Move forward by distance (meters)"""
        return self._move_relative(distance, 0, 0, speed)
        
    def move_backward(self, distance=1.0, speed=0.5):
        """Move backward by distance (meters)"""
        return self._move_relative(-distance, 0, 0, speed)
        
    def move_left(self, distance=1.0, speed=0.5):
        """Move left by distance (meters)"""
        return self._move_relative(0, -distance, 0, speed)
        
    def move_right(self, distance=1.0, speed=0.5):
        """Move right by distance (meters)"""
        return self._move_relative(0, distance, 0, speed)
        
    def rotate(self, degrees):
        """Rotate heading by degrees (positive = clockwise)"""
        yaw_rate = math.radians(degrees) / 2.0  # Convert to rad/s
        
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # Use yaw rate
            0, 0, 0,  # Position (ignored)
            0, 0, 0,  # Velocity (ignored)
            0, 0, 0,  # Acceleration (ignored)
            0, yaw_rate)
        
        print(f"🔄 Rotating {degrees}°...")
        time.sleep(2)
        return f"Rotated {degrees} degrees"
        
    def _move_relative(self, x, y, z, speed):
        """Internal: Move relative to current position"""
        duration = abs(max(x, y, z)) / speed
        
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111111000,  # Use position
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0)
        
        print(f"➡️  Moving ({x:.1f}, {y:.1f}, {z:.1f})m...")
        time.sleep(duration)
        return f"Moved to relative position ({x}, {y}, {z})"

# LLM-friendly function interface
def execute_command(command: str, **kwargs) -> str:
    """
    Execute a drone command from natural language
    
    Examples:
        execute_command("takeoff")
        execute_command("move_forward", distance=2.0)
        execute_command("rotate", degrees=90)
        execute_command("land")
    """
    controller = DroneController()
    
    commands = {
        "takeoff": controller.takeoff,
        "land": controller.land,
        "move_forward": controller.move_forward,
        "move_backward": controller.move_backward,
        "move_left": controller.move_left,
        "move_right": controller.move_right,
        "rotate": controller.rotate
    }
    
    if command in commands:
        return commands[command](**kwargs)
    else:
        return f"Unknown command: {command}"

if __name__ == "__main__":
    # Test the controller
    drone = DroneController()
    
    print("\n=== Testing Drone Controller ===\n")
    drone.takeoff(altitude=1.5)
    time.sleep(2)
    drone.move_forward(distance=2.0)
    time.sleep(2)
    drone.rotate(degrees=90)
    time.sleep(2)
    drone.land()
