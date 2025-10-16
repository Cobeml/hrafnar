#!/usr/bin/env python3
"""
Conversation Manager - State machine for person identification conversations
Handles name collection, identity verification, and info gathering
"""

from enum import Enum
from typing import Optional, Dict
import re


class ConversationState(Enum):
    """States in the person identification conversation flow"""
    NOT_STARTED = "not_started"
    ASKING_NAME = "asking_name"
    WAITING_NAME = "waiting_name"
    VERIFYING_IDENTITY = "verifying_identity"
    GATHERING_INFO = "gathering_info"
    WAITING_INFO = "waiting_info"
    COMPLETED = "completed"
    RECOGNIZED = "recognized"  # Person already known, skipping


class ConversationManager:
    """
    Manages conversations for person identification and information gathering.

    Workflow:
    1. Start conversation (ask for name)
    2. Wait for name response
    3. Extract name using VLM
    4. Verify identity in database
    5. If new: gather additional info
    6. Store complete record
    """

    def __init__(self, vlm_controller, database, image_similarity_matcher):
        """
        Initialize conversation manager.

        Args:
            vlm_controller: VLMDroneController instance
            database: MissionDatabase instance
            image_similarity_matcher: ImageSimilarityMatcher instance
        """
        self.vlm = vlm_controller
        self.db = database
        self.similarity = image_similarity_matcher

        # Track current conversation
        self.current_person_id = None
        self.current_state = ConversationState.NOT_STARTED
        self.collected_name = ""
        self.collected_info = ""

    def start_conversation(self, person_id: int) -> ConversationState:
        """
        Initiate greeting and name request.

        Args:
            person_id: Track ID of the person

        Returns:
            New conversation state
        """
        self.current_person_id = person_id
        self.current_state = ConversationState.ASKING_NAME

        # Greet and ask for name
        self.vlm.execute_action({
            'action': 'speak',
            'params': {'text': 'Hello! I am a rescue drone. May I ask your name?'}
        })

        self.current_state = ConversationState.WAITING_NAME
        print(f"💬 Started conversation with person {person_id}")
        return self.current_state

    def process_name_response(self, speech_text: str) -> Dict:
        """
        Extract name from person's response using VLM.

        Args:
            speech_text: What the person said

        Returns:
            Dict with 'name' and 'success' keys
        """
        if self.current_state != ConversationState.WAITING_NAME:
            return {'success': False, 'name': '', 'reason': 'Not waiting for name'}

        # Use VLM to extract name
        extraction_prompt = f"""Extract the person's name from this response: "{speech_text}"

Rules:
- If they say "I'm X" or "My name is X" or "This is X" → return X
- If they refuse or say they don't know → return "REFUSED"
- If unclear → return "UNCLEAR"

Return only the name, nothing else."""

        print(f"🧠 Extracting name from: '{speech_text}'")
        result = self.vlm.process_command(extraction_prompt)

        # Extract name from VLM observation
        name = result.get('observation', '').strip()

        # Also try regex patterns as fallback
        if not name or name == "UNCLEAR":
            patterns = [
                r"I'?m\s+([A-Z][a-z]+(?:\s+[A-Z][a-z]+)*)",  # I'm John Smith
                r"name\s+is\s+([A-Z][a-z]+(?:\s+[A-Z][a-z]+)*)",  # My name is John
                r"call\s+me\s+([A-Z][a-z]+)",  # Call me John
                r"this\s+is\s+([A-Z][a-z]+)",  # This is John
            ]
            for pattern in patterns:
                match = re.search(pattern, speech_text, re.IGNORECASE)
                if match:
                    name = match.group(1)
                    break

        if name and name not in ["REFUSED", "UNCLEAR"]:
            self.collected_name = name
            print(f"✅ Extracted name: {name}")
            return {'success': True, 'name': name}
        elif name == "REFUSED":
            print(f"⚠️  Person refused to give name")
            return {'success': False, 'name': '', 'reason': 'refused'}
        else:
            print(f"⚠️  Unclear response, asking again")
            return {'success': False, 'name': '', 'reason': 'unclear'}

    def verify_identity_by_name(self, name: str) -> Dict:
        """
        Check if person with this name already exists in database.

        Args:
            name: Person's name

        Returns:
            Dict with 'found', 'matches' keys
        """
        self.current_state = ConversationState.VERIFYING_IDENTITY
        print(f"🔍 Checking database for: {name}")

        matches = self.db.find_person_by_name(name)

        if matches:
            print(f"📋 Found {len(matches)} person(s) named '{name}' in database")
            return {'found': True, 'matches': matches, 'count': len(matches)}
        else:
            print(f"📋 No previous record of '{name}' found")
            return {'found': False, 'matches': [], 'count': 0}

    def handle_known_person(self, person_name: str):
        """
        Handle re-encounter with previously met person.

        Args:
            person_name: Name of recognized person
        """
        self.current_state = ConversationState.RECOGNIZED

        self.vlm.execute_action({
            'action': 'speak',
            'params': {'text': f'Nice to see you again, {person_name}! I remember we spoke before.'}
        })

        print(f"👋 Recognized known person: {person_name}")
        self.current_state = ConversationState.COMPLETED

    def gather_additional_info(self, name: str):
        """
        Ask for details about the person's situation.

        Args:
            name: Person's name (to personalize question)
        """
        self.current_state = ConversationState.GATHERING_INFO

        self.vlm.execute_action({
            'action': 'speak',
            'params': {
                'text': f'Nice to meet you, {name}! Can you tell me about your situation? Do you need any assistance?'
            }
        })

        self.current_state = ConversationState.WAITING_INFO
        print(f"💬 Gathering info from {name}")

    def process_info_response(self, speech_text: str) -> Dict:
        """
        Collect additional information from person's response.

        Args:
            speech_text: What the person said

        Returns:
            Dict with 'info' and 'success' keys
        """
        if self.current_state != ConversationState.WAITING_INFO:
            return {'success': False, 'info': '', 'reason': 'Not waiting for info'}

        # Store the information directly
        self.collected_info = speech_text
        print(f"📝 Collected info: {speech_text[:50]}...")

        return {'success': True, 'info': speech_text}

    def finalize_encounter(self, image_data: bytes, embedding) -> int:
        """
        Store complete person record in database.

        Args:
            image_data: PNG snapshot bytes
            embedding: CLIP embedding numpy array

        Returns:
            Database ID of stored record
        """
        if not self.collected_name:
            print("⚠️  Cannot finalize: No name collected")
            return -1

        print(f"💾 Storing person: {self.collected_name}")

        # Get current drone position
        position = self.vlm.drone.get_position()

        # Store in database
        db_id = self.db.store_person(
            track_id=self.current_person_id,
            name=self.collected_name,
            detailed_info=self.collected_info,
            image_data=image_data,
            embedding=embedding,
            position=position,
            first_seen=0.0  # Will be updated by mission_controller
        )

        # Thank the person
        self.vlm.execute_action({
            'action': 'speak',
            'params': {'text': f'Thank you, {self.collected_name}. Your information has been recorded. Stay safe!'}
        })

        self.current_state = ConversationState.COMPLETED
        print(f"✅ Stored person {self.collected_name} with ID {db_id}")

        return db_id

    def reset(self):
        """Reset conversation state for next person"""
        self.current_person_id = None
        self.current_state = ConversationState.NOT_STARTED
        self.collected_name = ""
        self.collected_info = ""
