#!/usr/bin/env python3
"""
PX4 Drone Controller with position verification
"""

from pymavlink import mavutil
import time
import math
import threading

class DroneController:
    def __init__(self, connection_string='udp:127.0.0.1:14540', disable_battery_failsafe=True):
        """Connect to PX4 via MAVLink"""
        print(f"Connecting to PX4 at {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("✅ Connected to PX4!")

        # Disable battery simulation to prevent failsafes during long missions
        if disable_battery_failsafe:
            self._disable_battery_failsafe()

        # Current state
        self.current_pos = [0, 0, 0]  # x, y, z in NED frame
        self.current_yaw = 0  # Current yaw in radians (from ATTITUDE)
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
        
    def _disable_battery_failsafe(self):
        """Disable battery simulation and failsafes to prevent interruptions during testing"""
        print("🔋 Disabling battery failsafe...")

        # Method 1: Disable battery simulation entirely
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'SIM_BAT_ENABLE',
            0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        time.sleep(0.1)

        # Method 2: Disable low battery action (failsafe won't trigger)
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'COM_LOW_BAT_ACT',
            0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        time.sleep(0.1)

        # Method 3: Set minimum battery percentage high (stays charged)
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'SIM_BAT_MIN_PCT',
            0.9,  # Stay at 90% minimum
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.1)

        print("✅ Battery failsafe disabled (simulation mode)")

    def _listen_position(self):
        """Listen for position and attitude updates from PX4"""
        while True:
            # Listen for both position and attitude messages
            msg = self.master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'], blocking=True, timeout=1)
            if msg:
                if msg.get_type() == 'LOCAL_POSITION_NED':
                    self.current_pos = [msg.x, msg.y, msg.z]
                elif msg.get_type() == 'ATTITUDE':
                    self.current_yaw = msg.yaw  # Yaw in radians
                
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

    def _body_to_ned(self, forward, right):
        """
        Transform body-frame movement to NED frame using current yaw

        Args:
            forward: Distance to move forward (positive) or backward (negative) in body frame
            right: Distance to move right (positive) or left (negative) in body frame

        Returns:
            (delta_x, delta_y): NED frame offsets
        """
        # Rotation matrix from body frame to NED frame
        # NED_x = forward * cos(yaw) - right * sin(yaw)
        # NED_y = forward * sin(yaw) + right * cos(yaw)
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        delta_x = forward * cos_yaw - right * sin_yaw
        delta_y = forward * sin_yaw + right * cos_yaw

        return delta_x, delta_y

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
        """Move forward in camera direction (body frame)"""
        print(f"➡️  Moving forward {distance}m (camera-relative)...")
        # Transform body-frame movement to NED
        delta_x, delta_y = self._body_to_ned(forward=distance, right=0)
        target_x = self.current_pos[0] + delta_x
        target_y = self.current_pos[1] + delta_y
        self.goto_position(target_x, target_y, self.current_pos[2])
        
    def move_right(self, distance=2.0):
        """Move right relative to camera direction (body frame)"""
        print(f"➡️  Moving right {distance}m (camera-relative)...")
        # Transform body-frame movement to NED
        delta_x, delta_y = self._body_to_ned(forward=0, right=distance)
        target_x = self.current_pos[0] + delta_x
        target_y = self.current_pos[1] + delta_y
        self.goto_position(target_x, target_y, self.current_pos[2])

    def move_backward(self, distance=2.0):
        """Move backward (opposite of camera direction)"""
        print(f"⬅️  Moving backward {distance}m...")
        self.move_forward(-distance)

    def move_left(self, distance=2.0):
        """Move left (opposite of camera's right)"""
        print(f"⬅️  Moving left {distance}m...")
        self.move_right(-distance)

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
    def rotate(self, degrees):
        """
        Rotate by degrees (positive = clockwise, negative = counter-clockwise)
        """
        print(f"🔄 Rotating {degrees}°...")
        
        # Convert degrees to radians
        yaw_rad = math.radians(degrees)
        
        # Current yaw + change
        new_yaw = self.target_yaw + yaw_rad
        
        if not self.offboard_mode:
            print("⚠️  Entering OFFBOARD mode...")
            self.set_offboard_mode()
            
        if not self.armed:
            self.arm()
        
        # Update target yaw
        self.target_yaw = new_yaw
        
        # Wait for rotation (assume ~45 deg/sec)
        duration = abs(degrees) / 45.0
        time.sleep(max(duration, 1.0))
        
        print(f"✅ Rotated {degrees}°")


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
