#!/usr/bin/env python3
"""
Mission Database - SQLite storage for encounter logging
"""

import sqlite3
import json
from datetime import datetime
from typing import Optional, List, Dict

class MissionDatabase:
    def __init__(self, db_path="mission_log.db"):
        self.db_path = db_path
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self._create_tables()
        
    def _create_tables(self):
        """Initialize database schema"""
        cursor = self.conn.cursor()
        
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS encounters (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                track_id INTEGER NOT NULL,
                timestamp REAL NOT NULL,
                location_x REAL,
                location_y REAL,
                location_z REAL,
                needs_assistance BOOLEAN,
                notes TEXT,
                conversation_log TEXT
            )
        """)
        
        self.conn.commit()
        
    def log_encounter(self, track_id: int, position: Dict, 
                     needs_assistance: Optional[bool] = None,
                     notes: str = "", conversation: List[str] = None):
        """Log an encounter with a person"""
        cursor = self.conn.cursor()
        
        cursor.execute("""
            INSERT INTO encounters 
            (track_id, timestamp, location_x, location_y, location_z, 
             needs_assistance, notes, conversation_log)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            track_id,
            datetime.now().timestamp(),
            position.get('x', 0),
            position.get('y', 0),
            position.get('z', 0),
            needs_assistance,
            notes,
            json.dumps(conversation or [])
        ))
        
        self.conn.commit()
        return cursor.lastrowid
        
    def has_encountered(self, track_id: int) -> bool:
        """Check if person has been logged"""
        cursor = self.conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM encounters WHERE track_id = ?", (track_id,))
        return cursor.fetchone()[0] > 0
        
    def get_summary(self) -> Dict:
        """Get mission summary statistics"""
        cursor = self.conn.cursor()
        
        cursor.execute("SELECT COUNT(*) FROM encounters")
        total = cursor.fetchone()[0]
        
        cursor.execute("SELECT COUNT(*) FROM encounters WHERE needs_assistance = 1")
        need_help = cursor.fetchone()[0]
        
        return {
            "total_encounters": total,
            "need_assistance": need_help
        }
        
    def close(self):
        self.conn.close()
