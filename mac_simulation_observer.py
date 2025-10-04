#!/usr/bin/env python3
"""
MacBook Simulation Observer
Polls Ubuntu PC for drone state, generates OpenAI summaries
"""

import requests
import time
from openai import OpenAI
from datetime import datetime
import os

# Your Ubuntu PC Tailscale IP
DRONE_API = "http://100.64.1.5:8001"

class MacSimulationObserver:
    def __init__(self, openai_api_key):
        self.client = OpenAI(api_key=openai_api_key)
        self.summary_interval = 10  # seconds
        
    def get_drone_state(self):
        """Fetch current state from Ubuntu PC"""
        try:
            response = requests.get(f"{DRONE_API}/state", timeout=5)
            return response.json()
        except Exception as e:
            print(f"❌ Cannot connect to drone: {e}")
            return None
            
    def generate_summary(self, state):
        """Generate OpenAI summary"""
        if not state or not state.get('position'):
            return None
            
        state_text = f"""
Drone Simulation State at {datetime.now().strftime('%H:%M:%S')}:

Position: {state['position']} meters
Velocity: {state.get('velocity', 'Unknown')} m/s
GPS: {state.get('gps', 'No GPS')}

Environment: Indoor curved gridroom with people walking around
"""
        
        try:
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": "You are a drone mission analyst. Provide concise tactical summaries."},
                    {"role": "user", "content": f"Summarize this in 2-3 sentences:\n{state_text}"}
                ],
                max_tokens=150
            )
            
            return response.choices[0].message.content
            
        except Exception as e:
            print(f"❌ OpenAI error: {e}")
            return None
            
    def run(self):
        """Main loop"""
        print("🔍 Simulation Observer running on MacBook")
        print(f"   Polling {DRONE_API} every {self.summary_interval}s")
        print(f"   Press Ctrl+C to stop\n")
        
        while True:
            # Get state
            state = self.get_drone_state()
            
            if state:
                # Generate summary
                summary = self.generate_summary(state)
                
                if summary:
                    print(f"\n{'='*60}")
                    print(f"🤖 OpenAI Summary ({datetime.now().strftime('%H:%M:%S')})")
                    print(f"{'-'*60}")
                    print(summary)
                    print(f"{'='*60}\n")
                    
                    # Save to log
                    with open('simulation_log.txt', 'a') as f:
                        f.write(f"\n{datetime.now().isoformat()}\n")
                        f.write(f"{summary}\n")
            
            time.sleep(self.summary_interval)

def main():
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        print("❌ Set OPENAI_API_KEY environment variable")
        return
    
    observer = MacSimulationObserver(api_key)
    
    try:
        observer.run()
    except KeyboardInterrupt:
        print("\n👋 Shutting down observer")

if __name__ == "__main__":
    main()
