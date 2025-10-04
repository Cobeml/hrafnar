#!/usr/bin/env python3
"""
Multilingual Drone Client with DeepL Translation
Speak in ANY language → DeepL translates → Drone understands
"""

import requests
import whisper
import pyttsx3
import sounddevice as sd
import soundfile as sf
import numpy as np
import os
from dotenv import load_dotenv

load_dotenv()  # Load .env file

DRONE_API = "http://100.99.98.39:8000"
DEEPL_API_KEY = os.getenv("deepl_api_key")  # Your DeepL key
DEEPL_API_URL = "https://api-free.deepl.com/v2/translate"  # Use 'api.deepl.com' for Pro

class MultilingualDroneClient:
    def __init__(self, user_language='auto'):
        """
        Args:
            user_language: Your language code (e.g., 'ES', 'FR', 'ZH', 'AR')
                          'auto' = auto-detect from speech
        """
        self.audio = pyaudio.PyAudio()
        self.user_language = user_language
        self.detected_language = None
        
        # Whisper (multilingual!)
        print("Loading Whisper multilingual model...")
        self.whisper = whisper.load_model("base")  # Supports 99 languages
        
        # TTS
        print("Loading TTS...")
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        
        print(f"✅ Ready! Speak in any language (detecting: {user_language})")
        
    def translate_to_english(self, text, source_lang='auto'):
        """Translate any language → English using DeepL"""
        if not DEEPL_API_KEY:
            print("⚠️  No DeepL API key found, using original text")
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
            print(f"Translation error: {e}")
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
            print(f"Translation error: {e}")
            return text
            
    def speak(self, text, language='en'):
        """Text-to-speech (English only for now)"""
        print(f"🔊 Speaking: {text}")
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()
        
    def listen(self, duration=5):
        """Record and transcribe"""
        print(f"🎤 Listening ({duration}s)...")
        
        # Record with sounddevice (much simpler!)
        audio_data = sd.rec(
            int(duration * 16000), 
            samplerate=16000, 
            channels=1, 
            dtype='float32'
        )
        sd.wait()  # Wait until recording is finished
        
        # Transcribe
        print("🧠 Transcribing...")
        result = self.whisper.transcribe(audio_data.flatten(), fp16=False)
        
        original_text = result["text"]
        detected_lang = result.get("language", "unknown")
        
        print(f"📝 Transcribed ({detected_lang}): {original_text}")
        self.detected_language = detected_lang.upper()
        
        return original_text
        
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
        """Voice control in ANY language"""
        self.speak("Multilingual drone system ready")
        
        while True:
            input("\nPress Enter to speak (any language, Ctrl+C to exit)...")
            
            # Listen in user's language
            original_command = self.listen(duration=4)
            
            if "exit" in original_command.lower() or "salir" in original_command.lower():
                self.speak("Shutting down")
                break
            
            # Translate to English for VLM
            english_command, source_lang = self.translate_to_english(
                original_command, 
                self.user_language
            )
            
            # Send to drone
            self.speak("Processing")
            english_response = self.send_command(english_command)
            
            # Translate response back to user's language
            translated_response = self.translate_from_english(
                english_response, 
                source_lang
            )
            
            # Speak response (in English for now - TTS for other languages needs additional setup)
            self.speak(translated_response)
            print(f"\n💬 Response: {translated_response}")

def main():
    print("\n=== Multilingual Voice-Controlled Drone ===\n")
    
    # Choose your language
    print("Available languages: ES (Spanish), FR (French), DE (German), ZH (Chinese), JA (Japanese), AR (Arabic), RU (Russian)")
    print("Or 'auto' for automatic detection")
    
    lang = input("Your language code (or 'auto'): ").strip().upper() or 'auto'
    
    client = MultilingualDroneClient(user_language=lang)
    
    # Test connection
    try:
        status = requests.get(f"{DRONE_API}/status", timeout=5).json()
        print(f"✅ Connected to drone")
    except:
        print(f"❌ Cannot connect to {DRONE_API}")
        return
    
    client.interactive_mode()

if __name__ == "__main__":
    main()
