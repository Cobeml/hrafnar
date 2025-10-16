#!/usr/bin/env python3
"""
Mission Database - SQLite storage for encounter logging
"""

import sqlite3
import json
import numpy as np
from datetime import datetime
from typing import Optional, List, Dict, Tuple
import io

class MissionDatabase:
    def __init__(self, db_path="mission_log.db"):
        self.db_path = db_path
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self._create_tables()
        
    def _create_tables(self):
        """Initialize database schema with memory-based recognition fields"""
        cursor = self.conn.cursor()

        # Create main encounters table
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
                conversation_log TEXT,

                -- Memory-based recognition fields (Phase B)
                name TEXT,
                detailed_info TEXT,
                image_data BLOB,
                embedding BLOB,
                embedding_similarity REAL DEFAULT 0.0,
                first_seen REAL,
                last_seen REAL,
                approached INTEGER DEFAULT 0,
                conversation_complete INTEGER DEFAULT 0
            )
        """)

        # Create index on name for fast lookups
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_name ON encounters(name)
        """)

        # Migration: Add new columns if they don't exist (for existing databases)
        self._migrate_schema(cursor)

        self.conn.commit()

    def _migrate_schema(self, cursor):
        """Add new columns to existing database if needed"""
        # Get existing columns
        cursor.execute("PRAGMA table_info(encounters)")
        existing_columns = {row[1] for row in cursor.fetchall()}

        # Add missing columns
        new_columns = {
            'name': 'TEXT',
            'detailed_info': 'TEXT',
            'image_data': 'BLOB',
            'embedding': 'BLOB',
            'embedding_similarity': 'REAL DEFAULT 0.0',
            'first_seen': 'REAL',
            'last_seen': 'REAL',
            'approached': 'INTEGER DEFAULT 0',
            'conversation_complete': 'INTEGER DEFAULT 0'
        }

        for col_name, col_type in new_columns.items():
            if col_name not in existing_columns:
                print(f"  Migrating database: Adding column '{col_name}'")
                cursor.execute(f"ALTER TABLE encounters ADD COLUMN {col_name} {col_type}")

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
        
    def _serialize_embedding(self, embedding: np.ndarray) -> bytes:
        """Convert NumPy embedding to bytes for storage"""
        if embedding is None:
            return None
        buffer = io.BytesIO()
        np.save(buffer, embedding)
        return buffer.getvalue()

    def _deserialize_embedding(self, blob: bytes) -> Optional[np.ndarray]:
        """Convert bytes back to NumPy embedding"""
        if blob is None:
            return None
        buffer = io.BytesIO(blob)
        return np.load(buffer)

    def store_person(
        self,
        track_id: int,
        name: str,
        detailed_info: str,
        image_data: bytes,
        embedding: np.ndarray,
        position: Dict,
        first_seen: float,
        embedding_similarity: float = 0.0
    ) -> int:
        """
        Store a complete person record with visual memory.

        Args:
            track_id: YOLO tracking ID
            name: Person's name
            detailed_info: Additional information
            image_data: PNG snapshot as bytes
            embedding: CLIP embedding (NumPy array)
            position: Drone position dict with x, y, z
            first_seen: Timestamp of first encounter
            embedding_similarity: Similarity score if re-identified

        Returns:
            Database ID of stored record
        """
        cursor = self.conn.cursor()

        embedding_blob = self._serialize_embedding(embedding)

        cursor.execute("""
            INSERT INTO encounters
            (track_id, name, detailed_info, image_data, embedding,
             location_x, location_y, location_z,
             first_seen, last_seen, timestamp,
             approached, conversation_complete, embedding_similarity)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 1, 1, ?)
        """, (
            track_id,
            name,
            detailed_info,
            image_data,
            embedding_blob,
            position.get('x', 0),
            position.get('y', 0),
            position.get('z', 0),
            first_seen,
            datetime.now().timestamp(),
            datetime.now().timestamp(),
            embedding_similarity
        ))

        self.conn.commit()
        return cursor.lastrowid

    def find_person_by_name(self, name: str) -> List[Dict]:
        """
        Find people by name (partial match).

        Args:
            name: Name or partial name to search for

        Returns:
            List of matching person records
        """
        cursor = self.conn.cursor()

        cursor.execute("""
            SELECT id, track_id, name, detailed_info,
                   location_x, location_y, location_z,
                   first_seen, last_seen, embedding
            FROM encounters
            WHERE name LIKE ?
            ORDER BY last_seen DESC
        """, (f"%{name}%",))

        results = []
        for row in cursor.fetchall():
            results.append({
                'id': row[0],
                'track_id': row[1],
                'name': row[2],
                'detailed_info': row[3],
                'location': (row[4], row[5], row[6]),
                'first_seen': row[7],
                'last_seen': row[8],
                'embedding': self._deserialize_embedding(row[9])
            })

        return results

    def get_all_embeddings(self) -> List[Tuple[int, np.ndarray]]:
        """
        Retrieve all stored embeddings for visual matching.

        Returns:
            List of (person_id, embedding) tuples
        """
        cursor = self.conn.cursor()

        cursor.execute("""
            SELECT id, embedding
            FROM encounters
            WHERE embedding IS NOT NULL
        """)

        embeddings = []
        for row in cursor.fetchall():
            person_id = row[0]
            embedding_blob = row[1]
            embedding = self._deserialize_embedding(embedding_blob)
            if embedding is not None:
                embeddings.append((person_id, embedding))

        return embeddings

    def get_person_by_id(self, person_id: int) -> Optional[Dict]:
        """Retrieve complete person record by database ID"""
        cursor = self.conn.cursor()

        cursor.execute("""
            SELECT id, track_id, name, detailed_info, image_data,
                   location_x, location_y, location_z,
                   first_seen, last_seen, embedding, embedding_similarity
            FROM encounters
            WHERE id = ?
        """, (person_id,))

        row = cursor.fetchone()
        if not row:
            return None

        return {
            'id': row[0],
            'track_id': row[1],
            'name': row[2],
            'detailed_info': row[3],
            'image_data': row[4],
            'location': (row[5], row[6], row[7]),
            'first_seen': row[8],
            'last_seen': row[9],
            'embedding': self._deserialize_embedding(row[10]),
            'embedding_similarity': row[11]
        }

    def close(self):
        self.conn.close()

