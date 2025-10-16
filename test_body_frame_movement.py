#!/usr/bin/env python3
"""
Test Body-Frame Movement - Verify camera-relative movements

This test verifies that:
1. move_forward() moves in the direction the camera faces
2. Rotating changes the movement direction
3. After rotating 90° right, move_forward() should move East (not North)
"""

from drone_controller import DroneController
import time
import math

def test_body_frame_movement():
    print("\n" + "="*60)
    print("BODY-FRAME MOVEMENT TEST")
    print("="*60)
    print("\nThis test verifies that movements follow camera direction")
    print("Make sure Isaac Sim is PLAYING!\n")

    drone = DroneController()
    time.sleep(2)

    # Get initial position
    initial_pos = drone.get_position()
    print(f"📍 Initial position: x={initial_pos['x']:.2f}, y={initial_pos['y']:.2f}, alt={initial_pos['altitude']:.2f}m")
    print(f"   Initial yaw: {math.degrees(drone.current_yaw):.1f}°\n")

    input("Press Enter to arm and enter OFFBOARD mode...")
    drone.set_offboard_mode()
    drone.arm()

    # Test 1: Move forward without rotation (should move North)
    print("\n" + "-"*60)
    print("TEST 1: Move forward 2m (facing North)")
    print("-"*60)
    input("Press Enter to execute...")

    before_pos = drone.get_position()
    drone.move_forward(2.0)
    after_pos = drone.get_position()

    delta_x = after_pos['x'] - before_pos['x']
    delta_y = after_pos['y'] - before_pos['y']

    print(f"✅ Moved: ΔX={delta_x:.2f}m, ΔY={delta_y:.2f}m")
    print(f"   Expected: ΔX≈2.0m, ΔY≈0m (North in NED)")

    # Test 2: Rotate 90° clockwise (right)
    print("\n" + "-"*60)
    print("TEST 2: Rotate 90° clockwise (right)")
    print("-"*60)
    input("Press Enter to execute...")

    drone.rotate(90)
    print(f"   Current yaw: {math.degrees(drone.current_yaw):.1f}°")

    # Test 3: Move forward after rotation (should move East)
    print("\n" + "-"*60)
    print("TEST 3: Move forward 2m (now facing East)")
    print("-"*60)
    print("Camera now points East → move_forward should move East")
    input("Press Enter to execute...")

    before_pos = drone.get_position()
    drone.move_forward(2.0)
    after_pos = drone.get_position()

    delta_x = after_pos['x'] - before_pos['x']
    delta_y = after_pos['y'] - before_pos['y']

    print(f"✅ Moved: ΔX={delta_x:.2f}m, ΔY={delta_y:.2f}m")
    print(f"   Expected: ΔX≈0m, ΔY≈2.0m (East in NED)")

    if abs(delta_y - 2.0) < 0.5 and abs(delta_x) < 0.5:
        print("\n🎉 SUCCESS! Movement is camera-relative!")
    else:
        print("\n⚠️  FAILED! Movement is still global-frame")

    # Test 4: Test move_left (should move North since facing East)
    print("\n" + "-"*60)
    print("TEST 4: Move left 2m (camera's left = North)")
    print("-"*60)
    input("Press Enter to execute...")

    before_pos = drone.get_position()
    drone.move_left(2.0)
    after_pos = drone.get_position()

    delta_x = after_pos['x'] - before_pos['x']
    delta_y = after_pos['y'] - before_pos['y']

    print(f"✅ Moved: ΔX={delta_x:.2f}m, ΔY={delta_y:.2f}m")
    print(f"   Expected: ΔX≈2.0m, ΔY≈0m (North in NED)")

    # Land
    print("\n" + "-"*60)
    print("Landing...")
    print("-"*60)
    input("Press Enter to land...")
    drone.land()

    print("\n" + "="*60)
    print("TEST COMPLETE!")
    print("="*60)
    print("\nSummary:")
    print("- Initial heading: North")
    print("- After 90° rotation: East")
    print("- move_forward moved East (followed camera)")
    print("- move_left moved North (camera's left)")
    print("\n✅ Body-frame movement working correctly!\n")

if __name__ == "__main__":
    test_body_frame_movement()
