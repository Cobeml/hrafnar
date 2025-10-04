#!/usr/bin/env python3
"""
Text-only Multilingual Drone Client
Type in ANY language → DeepL translates → Drone understands
"""

import requests
import os
from dotenv import load_dotenv

load_dotenv()

DRONE_API = "http://100.99.98.39:8002"  # Changed to 8002
DEEPL_API_KEY = os.getenv("deepl_api_key")
DEEPL_API_URL = "https://api-free.deepl.com/v2/translate"

class SimpleDroneClient:
    def __init__(self, user_language='auto'):
        """
        Args:
            user_language: Your language code (e.g., 'ES', 'FR', 'ZH', 'AR')
                          'auto' = auto-detect
        """
        self.user_language = user_language
        print(f"✅ Text Drone Client ready! (Language: {user_language})")
        
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
            
            print(f"🌍 Translated from {detected_lang}: {text} → {translated}")
            return translated, detected_lang
            
        except Exception as e:
            print(f"⚠️  Translation error: {e}")
            return text, 'EN'
            
    def translate_from_english(self, text, target_lang):
        """Translate English → User's language"""
        if target_lang == 'EN' or not DEEPL_API_KEY:
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
        """Text-based control in ANY language"""
        print("\n💬 Type commands in any language (type 'exit' to quit)")
        
        source_lang = self.user_language
        
        while True:
            # Get text input
            user_input = input("\n🎯 You: ").strip()
            
            if not user_input:
                continue
                
            if user_input.lower() in ['exit', 'quit', 'salir', 'quitter']:
                print("👋 Goodbye!")
                break
            
            # Translate to English for drone
            english_command, detected_lang = self.translate_to_english(
                user_input, 
                source_lang
            )
            
            # Remember detected language for responses
            if detected_lang != 'EN':
                source_lang = detected_lang
            
            # Send to drone
            print("⏳ Processing...")
            english_response = self.send_command(english_command)
            
            # Translate response back
            translated_response = self.translate_from_english(
                english_response, 
                source_lang
            )
            
            # Display response
            print(f"🤖 Drone: {translated_response}")

def main():
    print("\n" + "="*60)
    print("     Text-Based Multilingual Drone Controller")
    print("="*60)
    
    # Choose language
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
