#!/usr/bin/env python3
"""
Text-only Multilingual Drone Client
Type in ANY language → DeepL translates → Drone understands
"""

import requests
import os
from dotenv import load_dotenv

load_dotenv()

DRONE_API = "http://localhost:8002"
DEEPL_API_KEY = os.getenv("deepl_api_key")
DEEPL_API_URL = "https://api-free.deepl.com/v2/translate"

class SimpleDroneClient:
    def __init__(self, user_language='auto'):
        """
        Args:
            user_language: Preferred language code (e.g., 'ES', 'FR', 'ZH', 'AR')
                          'auto' = auto-detect (recommended)
        """
        self.user_language = user_language
        print(f"✅ Text Drone Client ready! (Preferred language: {user_language})")
        
    def translate_to_english(self, text, source_lang='auto'):
        """Translate any language → English using DeepL"""
        if not DEEPL_API_KEY:
            print("⚠️  No DeepL API key, using original text")
            return text, 'EN'
            
        try:
            response = requests.post(
                DEEPL_API_URL,
                data={
                    'auth_key': DEEPL_API_KEY,
                    'text': text,
                    'target_lang': 'EN',
                    'source_lang': source_lang if source_lang != 'auto' else None
                }
            )
            
            result = response.json()
            translated = result['translations'][0]['text']
            detected_lang = result['translations'][0]['detected_source_language']
            
            # Only show translation if it actually changed
            if text.strip() != translated.strip():
                print(f"🌍 Translated from {detected_lang}: {text} → {translated}")
            
            return translated, detected_lang
            
        except Exception as e:
            print(f"⚠️  Translation error: {e}")
            return text, 'EN'
            
    def translate_from_english(self, text, target_lang):
        """Translate English → User's language"""
        # Don't translate if target is English
        if target_lang == 'EN':
            return text
            
        if not DEEPL_API_KEY:
            return text
            
        try:
            response = requests.post(
                DEEPL_API_URL,
                data={
                    'auth_key': DEEPL_API_KEY,
                    'text': text,
                    'target_lang': target_lang,
                    'source_lang': 'EN'
                }
            )
            
            result = response.json()
            translated = result['translations'][0]['text']
            
            # Only show translation if it actually changed
            if text.strip() != translated.strip():
                print(f"🌍 Translated to {target_lang}: {text} → {translated}")
            
            return translated
            
        except Exception as e:
            print(f"⚠️  Translation error: {e}")
            return text
            
    def send_command(self, text_command):
        """Send command to drone"""
        try:
            response = requests.post(
                f"{DRONE_API}/command",
                params={"text_command": text_command},
                timeout=30
            )
            return response.json()["response"]
        except Exception as e:
            return f"Error: {e}"
            
    def interactive_mode(self):
        """Text-based control - auto-detects your language"""
        print("\n💬 Type commands in any language (type 'exit' to quit)")
        print("    Responses will match your language automatically\n")
        
        while True:
            # Get text input
            user_input = input("🎯 You: ").strip()
            
            if not user_input:
                continue
                
            if user_input.lower() in ['exit', 'quit', 'salir', 'quitter']:
                print("👋 Goodbye!")
                break
            
            # Translate to English for drone (and detect user's language)
            english_command, detected_lang = self.translate_to_english(
                user_input, 
                'auto'  # Always auto-detect
            )
            
            # Send to drone
            print("⏳ Processing...")
            english_response = self.send_command(english_command)
            
            # Translate response back to DETECTED language (not selected!)
            translated_response = self.translate_from_english(
                english_response, 
                detected_lang  # Use detected, not self.user_language
            )
            
            # Display response
            print(f"🤖 Drone: {translated_response}\n")

def main():
    print("\n" + "="*60)
    print("     Text-Based Multilingual Drone Controller")
    print("="*60)
    
    print("\n💡 Tip: Just type in your language, it will auto-detect!")
    print("   (Or select a preferred language below)")
    
    print("\nLanguages: ES (Spanish), FR (French), DE (German),")
    print("           ZH (Chinese), JA (Japanese), AR (Arabic),")
    print("           RU (Russian), or 'auto' for auto-detect")
    
    lang = input("\nYour language code (or press Enter for 'auto'): ").strip().upper() or 'auto'
    
    client = SimpleDroneClient(user_language=lang)
    
    # Test connection
    try:
        print(f"\n🔌 Connecting to {DRONE_API}...")
        status = requests.get(f"{DRONE_API}/status", timeout=5).json()
        print(f"✅ Connected to drone!")
    except Exception as e:
        print(f"❌ Cannot connect to {DRONE_API}: {e}")
        print("   Make sure the drone API is running.")
        return
    
    client.interactive_mode()

if __name__ == "__main__":
    main()
