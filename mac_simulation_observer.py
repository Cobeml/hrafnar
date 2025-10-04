#!/usr/bin/env python3
"""
Enhanced MacBook Simulation Observer
With OpenAI + Perplexity integration
"""

import requests
import time
from openai import OpenAI
from datetime import datetime
import os
import json
from dotenv import load_dotenv

DRONE_API = "http://100.99.98.39:8003"

class EnhancedSimulationObserver:
    def __init__(self, openai_api_key, perplexity_api_key=None):
        self.openai_client = OpenAI(api_key=openai_api_key)
        self.perplexity_api_key = perplexity_api_key
        self.summary_interval = 10
        
        # Tool definition for OpenAI
        self.tools = [{
            "type": "function",
            "function": {
                "name": "search_web",
                "description": "Search the web for real-time information about drone operations, locations, or tactical situations",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The search query"
                        }
                    },
                    "required": ["query"]
                }
            }
        }]
        
    def search_with_perplexity(self, query):
        """Use Perplexity for web search"""
        if not self.perplexity_api_key:
            return "Perplexity API key not set"
            
        try:
            response = requests.post(
                "https://api.perplexity.ai/chat/completions",
                headers={
                    "Authorization": f"Bearer {self.perplexity_api_key}",
                    "Content-Type": "application/json"
                },
                json={
                    "model": "llama-3.1-sonar-small-128k-online",
                    "messages": [
                        {"role": "user", "content": query}
                    ]
                }
            )
            
            result = response.json()
            return result['choices'][0]['message']['content']
            
        except Exception as e:
            return f"Perplexity error: {e}"
            
    def get_drone_state(self):
        """Fetch current state from Ubuntu PC"""
        try:
            response = requests.get(f"{DRONE_API}/state", timeout=5)
            return response.json()
        except Exception as e:
            print(f"❌ Cannot connect to drone: {e}")
            return None
            
    def generate_summary(self, state):
        """Generate OpenAI summary with tool calling"""
        if not state or not state.get('position'):
            return None
            
        state_text = f"""
Drone Simulation State at {datetime.now().strftime('%H:%M:%S')}:

Position: {state['position']} meters
Velocity: {state.get('velocity', 'Unknown')} m/s
GPS: {state.get('gps', 'No GPS')}
Armed: {state.get('armed', False)}

Environment: Indoor curved gridroom with people walking around
"""
        
        try:
            # Initial request with tools
            response = self.openai_client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": """You are a drone mission analyst. 
                    Provide concise tactical summaries. If you need real-time information 
                    about drone operations, regulations, or tactical situations, use the 
                    search_web tool."""},
                    {"role": "user", "content": f"Analyze this drone state:\n{state_text}"}
                ],
                tools=self.tools if self.perplexity_api_key else None,
                max_tokens=200
            )
            
            message = response.choices[0].message
            
            # Check if tool was called
            if message.tool_calls:
                print("🔍 OpenAI requested web search...")
                
                tool_call = message.tool_calls[0]
                function_args = json.loads(tool_call.function.arguments)
                search_query = function_args['query']
                
                print(f"   Query: {search_query}")
                
                # Execute Perplexity search
                search_result = self.search_with_perplexity(search_query)
                
                print(f"   Result: {search_result[:100]}...")
                
                # Send result back to OpenAI
                final_response = self.openai_client.chat.completions.create(
                    model="gpt-4o-mini",
                    messages=[
                        {"role": "system", "content": "You are a drone mission analyst."},
                        {"role": "user", "content": f"Analyze this drone state:\n{state_text}"},
                        message,
                        {
                            "role": "tool",
                            "tool_call_id": tool_call.id,
                            "content": search_result
                        }
                    ],
                    max_tokens=200
                )
                
                return final_response.choices[0].message.content
            
            return message.content
            
        except Exception as e:
            print(f"❌ OpenAI error: {e}")
            return None
            
    def run(self):
        """Main loop"""
        print("🔍 Enhanced Simulation Observer running on MacBook")
        print(f"   Polling {DRONE_API} every {self.summary_interval}s")
        if self.perplexity_api_key:
            print("   ✅ Perplexity integration enabled")
        print(f"   Press Ctrl+C to stop\n")
        
        while True:
            # Get state with debug
            print(f"📡 Fetching state from {DRONE_API}/state...")
            state = self.get_drone_state()
            
            print(f"📊 Received state: {state}")  # DEBUG
            
            if state:
                print(f"✅ State received, generating summary...")  # DEBUG
                summary = self.generate_summary(state)
                
                if summary:
                    print(f"\n{'='*60}")
                    print(f"🤖 AI Summary ({datetime.now().strftime('%H:%M:%S')})")
                    print(f"{'-'*60}")
                    print(summary)
                    print(f"{'='*60}\n")
                    
                    # Save to log
                    with open('simulation_log.txt', 'a') as f:
                        f.write(f"\n{'='*60}\n")
                        f.write(f"{datetime.now().isoformat()}\n")
                        f.write(f"{summary}\n")
                else:
                    print("⚠️  No summary generated")  # DEBUG
            else:
                print("❌ No state received")  # DEBUG
            
            print(f"⏳ Waiting {self.summary_interval}s...\n")
            time.sleep(self.summary_interval)

def main():
    # Load environment variables from .env file
    load_dotenv()
    
    openai_key = os.getenv('OPENAI_API_KEY')
    perplexity_key = os.getenv('PERPLEXITY_API_KEY')  # Optional
    
    if not openai_key:
        print("❌ Set OPENAI_API_KEY environment variable")
        return
    
    observer = EnhancedSimulationObserver(openai_key, perplexity_key)
    
    try:
        observer.run()
    except KeyboardInterrupt:
        print("\n👋 Shutting down observer")

if __name__ == "__main__":
    main()
