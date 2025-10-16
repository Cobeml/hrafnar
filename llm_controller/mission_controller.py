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
from dataclasses import dataclass, field
from typing import Optional, List, Dict
import math
import queue
import numpy as np
import io

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
    no_detection_timeout: int = 15  # seconds
    search_pattern: str = "grid"  # or "spiral"

@dataclass
class PersonEncounter:
    """Track encounters with people - includes visual memory for re-identification"""
    track_id: int
    first_seen: float
    last_bbox: List[int]
    description: str = ""  # VLM description of the person (appearance, clothing, etc.)
    location: tuple = (0.0, 0.0, 0.0)  # Last known drone position when seeing this person
    approached: bool = False
    conversation_complete: bool = False

    # Memory-based recognition fields (Phase B)
    name: str = ""  # Person's name (collected via conversation)
    detailed_info: str = ""  # Additional information about the person
    image_data: Optional[bytes] = None  # PNG snapshot for visual matching
    embedding: Optional[np.ndarray] = field(default=None, repr=False)  # CLIP embedding [512]
    conversation_state: str = "not_started"  # Conversation state machine
    embedding_similarity: float = 0.0  # Similarity score if matched via visual recognition
    
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

        # Track action history to detect repetitions
        self.action_history: List[str] = []  # Track last 4 actions

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

    def _reflect_on_action(self, action_taken: str, intended_goal: str) -> dict:
        """
        ReAct Framework: Reflection step after action execution
        Verifies if the action achieved its goal and informs next decision
        """
        # Wait for action to complete and camera to update
        time.sleep(0.5)

        # Spin to get fresh camera frame and detections
        for _ in range(5):
            rclpy.spin_once(self.vlm, timeout_sec=0.1)
            rclpy.spin_once(self.vlm.yolo_tracker, timeout_sec=0.1)
            time.sleep(0.1)

        # Ask VLM to reflect on the action
        current_pos = self.vlm.drone.get_position()
        reflection_prompt = f"""REFLECTION AFTER ACTION:
I just executed: {action_taken}
My goal was: {intended_goal}
Current altitude: {current_pos['altitude']:.1f}m

Please analyze:
1. Did my action achieve the goal? (Compare before/after using visual bounding boxes)
2. What changed in the scene? (person position, visibility, etc.)
3. What should I do next to continue toward my mission objective?
"""

        print(f"\n🔄 Reflecting on action: {action_taken}")
        reflection = self.vlm.process_command(reflection_prompt)

        if 'observation' in reflection:
            print(f"💭 Reflection: {reflection['observation'][:150]}...")

        return reflection

    def _validate_and_clamp_movement(self, decision: dict, max_distance: float = 5.0) -> dict:
        """Validate and clamp movement distances to prevent absurd movements"""
        movement_actions = ['move_forward', 'move_backward', 'move_right', 'move_left']

        if decision.get('action') in movement_actions:
            distance = decision.get('params', {}).get('distance', 0)
            if abs(distance) > max_distance:
                original = distance
                clamped = max_distance if distance > 0 else -max_distance
                decision['params']['distance'] = clamped
                print(f"⚠️  Clamped unreasonable movement: {original:.1f}m → {clamped:.1f}m")

        # Check for illogical lateral movement when person is centered
        if decision.get('action') in ['move_left', 'move_right']:
            obs = decision.get('observation', '').lower()
            thinking = decision.get('thinking', '').lower()
            combined = obs + ' ' + thinking

            if any(phrase in combined for phrase in ['center', 'centered', 'closer to center', 'middle']):
                print(f"⚠️  WARNING: Person appears CENTERED but VLM chose lateral movement ({decision.get('action')})")
                print(f"    Observation: {obs[:100]}")
                print(f"    Suggestion: Should use move_forward to approach centered person")
                # Don't override - let VLM learn from feedback, but warn operator

        # Track action history and detect repetitions
        action = decision.get('action')
        if action:
            self.action_history.append(action)
            # Keep only last 4 actions
            if len(self.action_history) > 4:
                self.action_history.pop(0)

            # Detect if last 3 actions are identical
            if len(self.action_history) >= 3:
                if len(set(self.action_history[-3:])) == 1:
                    print(f"🔄 REPETITION DETECTED: Last 3 actions were all '{action}'")
                    print(f"    Forcing different action to break loop...")
                    # Force a rotate action to break the loop
                    decision = {
                        'action': 'rotate',
                        'params': {'degrees': 45},
                        'reasoning': 'Breaking repetition loop by rotating to find new perspective'
                    }
                    self.action_history.append('rotate')  # Update history with forced action

        return decision

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
            rclpy.spin_once(self.vlm.yolo_tracker, timeout_sec=0.1)  
            
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
        # print(f"🔍 DEBUG: Got {len(detections)} detections")  # Commented out to reduce log spam
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

            # Check if this might be a previously seen person (YOLO ID changed)
            context = ""
            current_pos = self.vlm.drone.get_position()
            for prev_id, prev_enc in list(self.encounters.items()):
                if prev_enc.description and prev_id != newest.track_id:
                    # Calculate distance from previous encounter location
                    dist = ((current_pos['x'] - prev_enc.location[0])**2 +
                            (current_pos['y'] - prev_enc.location[1])**2)**0.5
                    if dist < 5.0:  # Within 5 meters of previous encounter
                        context = f"\n\nNOTE: Previously at this location, I saw: {prev_enc.description}. This might be the same person."
                        break

            # Ask VLM to describe and decide (with altitude context)
            decision = self.vlm.process_command(
                f"At {current_pos['altitude']:.1f}m altitude, I detect a person (YOLO ID {newest.track_id}). Please describe what they look like (clothing, appearance) and decide if you should approach them.{context}"
            )

            # Store VLM's description from observation
            if 'observation' in decision:
                newest.description = decision['observation']
                newest.location = (current_pos['x'], current_pos['y'], current_pos['altitude'])
                print(f"📝 Stored description: {newest.description[:80]}...")

            # Validate and clamp movement distances
            decision = self._validate_and_clamp_movement(decision)

            if decision['action'] and decision['action'] != 'land':
                self.current_target = newest.track_id
                action_str = f"{decision['action']}({decision.get('params', {})})"
                intended_goal = f"Approach person ID {newest.track_id}"

                # Execute action
                result = self.vlm.execute_action(decision)
                print(f"✅ {result}")
                self.last_movement_time = time.time()

                # ReAct Framework: Reflect on action
                reflection = self._reflect_on_action(action_str, intended_goal)

                # If reflection suggests follow-up action, execute it
                reflection = self._validate_and_clamp_movement(reflection)
                if reflection.get('action') and reflection['action'] not in ['land', 'speak']:
                    print(f"🔄 Follow-up action from reflection: {reflection['action']}")
                    follow_result = self.vlm.execute_action(reflection)
                    print(f"✅ {follow_result}")
                
    def _on_bbox_changed(self):
        """Handle significant movement of tracked person"""
        print(f"📐 Person {self.current_target} moved significantly")

        detections = self.vlm.yolo_tracker.get_detections()
        target_det = next((d for d in detections if d['id'] == self.current_target), None)

        if target_det:
            # Don't pass pixel coordinates - VLM will interpret them as meters!
            pos = self.vlm.drone.get_position()
            decision = self.vlm.process_command(
                f"At {pos['altitude']:.1f}m altitude, person {self.current_target} has moved in the frame. Should you adjust your position slightly (0.5-2m) to keep them visible and centered?"
            )

            # Validate and clamp movement distances
            decision = self._validate_and_clamp_movement(decision)

            if decision['action']:
                action_str = f"{decision['action']}({decision.get('params', {})})"
                intended_goal = f"Keep person {self.current_target} visible and centered"

                # Execute action
                result = self.vlm.execute_action(decision)
                print(f"✅ {result}")

                # ReAct Framework: Reflect on action
                reflection = self._reflect_on_action(action_str, intended_goal)

                # If reflection suggests follow-up, execute
                reflection = self._validate_and_clamp_movement(reflection)
                if reflection.get('action') and reflection['action'] not in ['land', 'speak']:
                    print(f"🔄 Follow-up action: {reflection['action']}")
                    follow_result = self.vlm.execute_action(reflection)
                    print(f"✅ {follow_result}")
                
    def _on_person_lost(self):
        """Handle losing track of current target"""
        print(f"⚠️  Lost sight of person {self.current_target}")

        # Include person description if available
        encounter = self.encounters.get(self.current_target)
        person_desc = ""
        if encounter and encounter.description:
            person_desc = f" (They were: {encounter.description[:100]})"

        pos = self.vlm.drone.get_position()
        decision = self.vlm.process_command(
            f"At {pos['altitude']:.1f}m altitude, I lost track of person {self.current_target}{person_desc}. Should I rotate left/right to search, or move forward 1-2m to explore?"
        )

        # Validate and clamp movement distances
        decision = self._validate_and_clamp_movement(decision)

        if decision['action']:
            action_str = f"{decision['action']}({decision.get('params', {})})"
            intended_goal = f"Search for lost person {self.current_target}"

            # Execute action
            result = self.vlm.execute_action(decision)
            print(f"✅ {result}")

            # ReAct Framework: Reflect to see if person found
            reflection = self._reflect_on_action(action_str, intended_goal)

            # Check if person was found in reflection
            if reflection.get('action'):
                reflection = self._validate_and_clamp_movement(reflection)
                if reflection['action'] not in ['land']:
                    print(f"🔄 Follow-up: {reflection['action']}")
                    follow_result = self.vlm.execute_action(reflection)
                    print(f"✅ {follow_result}")

        self.current_target = None
            
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
        self.last_detection_time = time.time()  # RESET THE TIMER
        print("⏱️  No people detected for 60 seconds")
        
        # Check if mission should complete
        all_approached = all(e.approached for e in self.encounters.values())
        
        if self.encounters and all_approached:
            print("✅ All detected people have been approached")
            self._complete_mission()
        else:
            # Continue search pattern
            decision = self.vlm.process_command(
                "No people detected. Choose movement: rotate, move_forward, or land to complete mission."
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
        speech_text = self.speech_queue.get()
        print(f"👤 Person: {speech_text}")
        
        decision = self.vlm.process_command(f"Person said: '{speech_text}'. Respond or end conversation?")
        
        result = self.vlm.execute_action(decision)
        
        # Check if conversation ended
        if result == "conversation_ended":
            self._handle_event(EventType.CONVERSATION_COMPLETE)

        
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
