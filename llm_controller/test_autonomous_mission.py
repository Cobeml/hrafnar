#!/usr/bin/env python3
"""
Automated Test Script for Autonomous Emergency Response Mission
Simulates person responses to test full conversation flow
"""

import rclpy
import time
import threading
from mission_controller import MissionController, MissionConfig

class AutomatedMissionTest:
    def __init__(self, controller):
        self.controller = controller
        self.test_scenarios = [
            # Scenario 1: Person needs help
            {
                "responses": [
                    "Hello! I'm John Smith",
                    "Yes, I need medical assistance. My leg is injured.",
                    "Thank you, please hurry!"
                ],
                "delay_after_approach": 3.0  # seconds after approach before speaking
            },
            # Scenario 2: Person is okay
            {
                "responses": [
                    "Hi there, I'm Sarah",
                    "No, I'm fine, just sheltering here",
                    "Thanks for checking!"
                ],
                "delay_after_approach": 3.0
            }
        ]
        self.current_scenario = 0
        self.response_index = 0
        
    def run_test(self):
        """Run automated test with canned responses"""
        print("\n🧪 Starting Automated Mission Test\n")
        
        # Start monitoring for when to inject responses
        threading.Thread(target=self._monitor_mission, daemon=True).start()
        
        # Start the mission
        self.controller.start_mission()
        
    def _monitor_mission(self):
        """Monitor mission state and inject canned responses"""
        last_target = None
        approach_time = None
        
        while self.controller.mission_active:
            time.sleep(0.5)
            
            # Detect when drone approaches a new person
            if self.controller.current_target != last_target and self.controller.current_target is not None:
                last_target = self.controller.current_target
                approach_time = time.time()
                self.response_index = 0
                print(f"\n🎬 Drone approaching person {last_target} - starting scenario {self.current_scenario + 1}")
                
            # Inject canned responses after approach
            if approach_time and time.time() - approach_time > 2.0:
                self._inject_response()
                approach_time = None  # Only once per approach
                
    def _inject_response(self):
        """Inject a canned response into the speech queue"""
        if self.current_scenario >= len(self.test_scenarios):
            return
            
        scenario = self.test_scenarios[self.current_scenario]
        
        if self.response_index < len(scenario["responses"]):
            # Wait a bit
            time.sleep(scenario["delay_after_approach"])
            
            response = scenario["responses"][self.response_index]
            print(f"🤖 [TEST] Injecting response: '{response}'")
            
            # Put response in queue
            self.controller.speech_queue.put(response)
            self.response_index += 1
            
            # Schedule next response
            if self.response_index < len(scenario["responses"]):
                threading.Timer(3.0, self._inject_response).start()
            else:
                # Move to next scenario after this one completes
                self.current_scenario += 1

def main():
    rclpy.init()
    
    config = MissionConfig(
        max_duration_seconds=300,  # 5 min test
        search_altitude=2.0,
        no_detection_timeout=30  # Shorter for testing
    )
    
    controller = MissionController(config)
    
    # Wait for systems ready
    print("⏳ Waiting for camera and YOLO...")
    while controller.vlm.latest_image is None:
        rclpy.spin_once(controller.vlm, timeout_sec=0.1)
        time.sleep(0.1)
    
    print("✅ All systems ready!")
    
    # Create test harness
    test = AutomatedMissionTest(controller)
    
    input("\n🚀 Press Enter to start automated test...")
    test.run_test()
    
    print("\n✅ Test complete!")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
