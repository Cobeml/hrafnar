#!/usr/bin/env python3
"""
Mission Controller - Event-driven autonomous emergency response
Orchestrates VLM + YOLO + Drone for search and survey missions
"""

import rclpy
from rclpy.node import Node
import time
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List, Dict
import math
import queue

from vlm_drone_controller import VLMDroneController
from mission_database import MissionDatabase

class EventType(Enum):
    NEW_PERSON_DETECTED = "new_person"
    PERSON_LOST = "person_lost"
    BBOX_CHANGED_SIGNIFICANTLY = "bbox_changed"
    MOVEMENT_COMPLETE = "movement_complete"
    CONVERSATION_COMPLETE = "conversation_complete"
    NO_DETECTIONS_TIMEOUT = "no_detections"
    MISSION_TIMEOUT = "mission_timeout"

@dataclass
class MissionConfig:
    """Configuration for emergency response mission"""
    max_duration_seconds: int = 600  # 10 minutes
    search_altitude: float = 2.0  # meters
    approach_distance: float = 1.5  # meters from person
    bbox_change_threshold: int = 50  # pixels
    no_detection_timeout: int = 60  # seconds
    search_pattern: str = "grid"  # or "spiral"

@dataclass
class PersonEncounter:
    """Track encounters with people"""
    track_id: int
    first_seen: float
    last_bbox: List[int]
    approached: bool = False
    conversation_complete: bool = False
    
class MissionController(Node):
    def __init__(self, config: MissionConfig):
        super().__init__('mission_controller')
        
        self.config = config
        self.mission_start_time = None
        self.mission_active = False
        
        # Initialize VLM controller (includes YOLO + Drone)
        print("Initializing VLM Drone Controller...")
        self.vlm = VLMDroneController()
        
        # Track encountered people
        self.encounters: Dict[int, PersonEncounter] = {}
        self.current_target: Optional[int] = None
        
        # Timing for event triggers
        self.last_detection_time = time.time()
        self.last_movement_time = time.time()

         # Queue for simulated speech input
        self.speech_queue = queue.Queue()
        
        # Start input listener in separate thread
        self.input_thread = threading.Thread(target=self._listen_for_input, daemon=True)
        self.input_thread.start()

        self.db = MissionDatabase()
        
        print("✅ Mission Controller ready!")
        
    def start_mission(self):
        """Begin autonomous emergency response mission"""
        print("\n" + "="*60)
        print("🚁 STARTING EMERGENCY RESPONSE MISSION")
        print("="*60)
        
        self.mission_start_time = time.time()
        self.mission_active = True
        
        # Takeoff to search altitude
        print(f"Taking off to {self.config.search_altitude}m...")
        self.vlm.drone.set_offboard_mode()
        self.vlm.drone.arm()
        self.vlm.drone.climb(self.config.search_altitude)
        
        print("✅ Airborne! Beginning search pattern...")
        
        # Start event monitoring loop
        self._event_loop()
        
    def _event_loop(self):
        """Main event-driven decision loop"""
        while self.mission_active:
            rclpy.spin_once(self.vlm, timeout_sec=0.1)
            
            # Check for events
            event = self._detect_event()
            
            if event:
                print(f"\n🔔 EVENT DETECTED: {event.name}")
                self._handle_event(event)
            
            # Check mission timeout
            if time.time() - self.mission_start_time > self.config.max_duration_seconds:
                self._handle_event(EventType.MISSION_TIMEOUT)
                break
                
            time.sleep(0.1)

    def _listen_for_input(self):
        """Listen for typed 'speech' from person"""
        while self.mission_active or not hasattr(self, 'mission_active'):
            try:
                text = input("💬 Person says: ")
                if text.strip():
                    self.speech_queue.put(text)
            except EOFError:
                break
            
    def _detect_event(self) -> Optional[EventType]:
        """Monitor for events and return first detected"""
        detections = self.vlm.yolo_tracker.get_detections()
        current_time = time.time()
        
        # Event 1: New person detected
        for det in detections:
            track_id = det['id']
            if track_id not in self.encounters and track_id != -1:
                self.encounters[track_id] = PersonEncounter(
                    track_id=track_id,
                    first_seen=current_time,
                    last_bbox=det['bbox']
                )
                return EventType.NEW_PERSON_DETECTED
        
        # Event 2: Bounding box changed significantly (for current target)
        if self.current_target is not None:
            for det in detections:
                if det['id'] == self.current_target:
                    encounter = self.encounters[self.current_target]
                    if self._bbox_changed_significantly(encounter.last_bbox, det['bbox']):
                        encounter.last_bbox = det['bbox']
                        return EventType.BBOX_CHANGED_SIGNIFICANTLY
        
        # Event 3: Person lost (current target disappeared)
        if self.current_target is not None:
            target_found = any(det['id'] == self.current_target for det in detections)
            if not target_found:
                return EventType.PERSON_LOST
        
        # Event 4: No detections timeout
        if detections:
            self.last_detection_time = current_time
        elif current_time - self.last_detection_time > self.config.no_detection_timeout:
            return EventType.NO_DETECTIONS_TIMEOUT

        # Event: Person spoke (check queue)
        if not self.speech_queue.empty():
            return EventType.PERSON_SPOKE
            
        return None
        
    def _bbox_changed_significantly(self, old_bbox, new_bbox) -> bool:
        """Check if bounding box moved or resized significantly"""
        old_center = [(old_bbox[0] + old_bbox[2]) / 2, (old_bbox[1] + old_bbox[3]) / 2]
        new_center = [(new_bbox[0] + new_bbox[2]) / 2, (new_bbox[1] + new_bbox[3]) / 2]
        
        distance = math.sqrt((new_center[0] - old_center[0])**2 + 
                           (new_center[1] - old_center[1])**2)
        
        return distance > self.config.bbox_change_threshold
        
    def _handle_event(self, event: EventType):
        """Route event to appropriate handler"""
        handlers = {
            EventType.NEW_PERSON_DETECTED: self._on_new_person,
            EventType.PERSON_LOST: self._on_person_lost,
            EventType.BBOX_CHANGED_SIGNIFICANTLY: self._on_bbox_changed,
            EventType.MOVEMENT_COMPLETE: self._on_movement_complete,
            EventType.CONVERSATION_COMPLETE: self._on_conversation_complete,
            EventType.NO_DETECTIONS_TIMEOUT: self._on_no_detections,
            EventType.MISSION_TIMEOUT: self._on_mission_timeout,
        }
        
        handler = handlers.get(event)
        if handler:
            handler()
            
    def _on_new_person(self):
        """Handle new person detection"""
        # Find the newest person
        newest = max(self.encounters.values(), key=lambda e: e.first_seen)
        
        if not newest.approached:
            print(f"👤 New person detected (ID {newest.track_id})")
            
            # Ask VLM for decision
            decision = self.vlm.process_command(
                f"I detect a new person (ID {newest.track_id}). Should I approach them?"
            )
            
            if decision['action'] and decision['action'] != 'land':
                self.current_target = newest.track_id
                result = self.vlm.execute_action(decision)
                print(f"✅ {result}")
                self.last_movement_time = time.time()
                
    def _on_bbox_changed(self):
        """Handle significant movement of tracked person"""
        print(f"📐 Person {self.current_target} moved significantly")
        
        detections = self.vlm.yolo_tracker.get_detections()
        target_det = next((d for d in detections if d['id'] == self.current_target), None)
        
        if target_det:
            decision = self.vlm.process_command(
                f"Person {self.current_target} moved to {target_det['center']}. Adjust position to keep them centered?"
            )
            
            if decision['action']:
                result = self.vlm.execute_action(decision)
                print(f"✅ {result}")
                
    def _on_person_lost(self):
        """Handle losing track of current target"""
        print(f"⚠️  Lost sight of person {self.current_target}")
        
        decision = self.vlm.process_command(
            f"Lost track of person {self.current_target}. Should I search or move to next target?"
        )
        
        self.current_target = None
        
        if decision['action']:
            result = self.vlm.execute_action(decision)
            print(f"✅ {result}")
            
    def _on_movement_complete(self):
        """Handle completion of movement command"""
        print("✅ Movement complete")
        # This would be triggered by drone controller when movement finishes
        
    def _on_conversation_complete(self):
        """Handle completion of conversation with person"""
        if self.current_target:
            self.encounters[self.current_target].conversation_complete = True
            print(f"✅ Conversation with person {self.current_target} complete")

            position = self.vlm.drone.get_position()
            self.db.log_encounter(
                track_id=self.current_target,
                position=position,
                notes="Conversation completed"
            )

            
        # Continue mission
        self._continue_search()
        
    def _on_no_detections(self):
        """Handle no people detected for extended period"""
        print("⏱️  No people detected for 60 seconds")
        
        # Check if mission should complete
        all_approached = all(e.approached for e in self.encounters.values())
        
        if self.encounters and all_approached:
            print("✅ All detected people have been approached")
            self._complete_mission()
        else:
            # Continue search pattern
            decision = self.vlm.process_command(
                "No people detected recently. Should I continue searching or complete mission?"
            )
            
            if decision['action'] == 'land':
                self._complete_mission()
            elif decision['action']:
                result = self.vlm.execute_action(decision)
                print(f"✅ {result}")
                
    def _on_mission_timeout(self):
        """Handle mission timeout"""
        print("⏱️  Mission timeout reached")
        self._complete_mission()

    def _on_person_spoke(self):
        """Handle person speaking"""
        speech_text = self.speech_queue.get()
        print(f"👤 Person: {speech_text}")
        
        # Ask VLM to respond
        decision = self.vlm.process_command(
            f"The person said: '{speech_text}'. How should I respond?"
        )
        
        # VLM should return speak action with response text
        if decision['action'] == 'speak':
            self.vlm.speak(decision['params'].get('text', 'Hello'))
        
    def _continue_search(self):
        """Continue search pattern after handling a person"""
        print("🔍 Continuing search...")
        
        decision = self.vlm.process_command(
            "Person interaction complete. Continue search pattern or check other areas?"
        )
        
        self.current_target = None
        
        if decision['action']:
            result = self.vlm.execute_action(decision)
            print(f"✅ {result}")
            
    def _complete_mission(self):
        """Complete mission and land"""
        print("\n" + "="*60)
        print("🏁 MISSION COMPLETE")
        print(f"   Total people encountered: {len(self.encounters)}")
        print(f"   Duration: {time.time() - self.mission_start_time:.1f}s")
        print("="*60)
        
        self.vlm.drone.land()
        self.mission_active = False
        
def main():
    rclpy.init()
    
    config = MissionConfig(
        max_duration_seconds=600,
        search_altitude=2.0,
        approach_distance=1.5
    )
    
    controller = MissionController(config)
    
    # Wait for systems ready
    print("Waiting for camera and YOLO...")
    while controller.vlm.latest_image is None:
        rclpy.spin_once(controller.vlm, timeout_sec=0.1)
        time.sleep(0.1)
    
    print("✅ All systems ready!")
    
    input("\nPress Enter to start emergency response mission...")
    controller.start_mission()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
