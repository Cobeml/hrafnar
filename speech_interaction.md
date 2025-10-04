🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠🧠

Excellent idea! This is totally feasible and would make the drone interaction much more natural!

Architecture:
MacBook Air (Remote Client)
    ↓ Tailscale VPN
Ubuntu PC (Simulation + VLM)
    ↓ ROS2/MAVLink
Simulated Drone
Copy
Audio Flow:

Drone Speaks: VLM → TTS on Ubuntu → Audio stream → MacBook speakers
Voice Commands: MacBook mic → Audio stream → Ubuntu STT → VLM → Drone action
Implementation Plan:
What Runs Where:
Ubuntu PC (Does the heavy lifting):

✅ VLM processing (already there)
✅ Text-to-Speech (TTS) generation
✅ Speech-to-Text (STT) processing
✅ Audio API server (FastAPI)
MacBook Air (Thin client):

✅ Audio playback (speakers)
✅ Audio recording (microphone)
✅ Simple web interface or Python client
Step-by-Step Implementation:
Phase 1: Add Audio API to Ubuntu PC
1. Create llm_controller/audio_server.py:

#!/usr/bin/env python3
"""
Audio API Server for Drone Communication
Handles TTS (drone speaking) and STT (voice commands)
"""

from fastapi import FastAPI, UploadFile, WebSocket
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
import torch
import io
import wave
import whisper
from TTS.api import TTS
import asyncio
import json

app = FastAPI(title="Drone Audio API")

# Enable CORS for MacBook client
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Load models (runs on GPU)
print("Loading TTS model...")
tts = TTS("tts_models/en/ljspeech/tacotron2-DDC").to("cuda")

print("Loading Whisper STT model...")
stt_model = whisper.load_model("base").to("cuda")  # ~1GB VRAM

# Drone message queue
message_queue = asyncio.Queue()

@app.post("/drone/speak")
async def drone_speak(text: str):
    """
    Drone speaks text
    Returns: Audio stream (WAV)
    """
    print(f"🗣️  Drone speaking: {text}")
    
    # Generate speech
    wav = tts.tts(text)
    
    # Convert to WAV bytes
    buffer = io.BytesIO()
    with wave.open(buffer, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(22050)
        wav_file.writeframes((torch.tensor(wav) * 32767).numpy().astype('int16').tobytes())
    
    buffer.seek(0)
    return StreamingResponse(buffer, media_type="audio/wav")

@app.post("/voice/command")
async def voice_command(audio: UploadFile):
    """
    Process voice command from user
    Returns: Transcribed text
    """
    # Save uploaded audio
    audio_bytes = await audio.read()
    
    # Transcribe with Whisper
    audio_array = whisper.load_audio(io.BytesIO(audio_bytes))
    result = stt_model.transcribe(audio_array)
    
    command_text = result["text"]
    print(f"🎤 Voice command: {command_text}")
    
    # Add to queue for VLM processing
    await message_queue.put({
        "type": "voice_command",
        "text": command_text
    })
    
    return {"transcription": command_text}

@app.websocket("/ws/audio")
async def websocket_audio(websocket: WebSocket):
    """
    WebSocket for real-time audio streaming
    """
    await websocket.accept()
    
    try:
        while True:
            # Send any queued drone messages
            if not message_queue.empty():
                msg = await message_queue.get()
                await websocket.send_json(msg)
            
            # Receive voice data
            data = await websocket.receive_bytes()
            # Process audio chunk...
            
    except Exception as e:
        print(f"WebSocket error: {e}")

@app.get("/")
async def root():
    return {"status": "Drone Audio API running", "endpoints": ["/drone/speak", "/voice/command"]}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
Copy
2. Update docker/Dockerfile.vlm:

FROM python:3.10-slim

RUN apt-get update && apt-get install -y \
    git ffmpeg libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch
RUN pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cu121

# Install AI packages
RUN pip install --no-cache-dir \
    transformers accelerate peft bitsandbytes \
    qwen-vl-utils pillow pymavlink \
    openai-whisper TTS fastapi uvicorn python-multipart

WORKDIR /app
3. Add to docker-compose.yml:

  vlm_controller:
    # ... existing config ...
    ports:
      - "8000:8000"  # Audio API
    command: python audio_server.py  # Start audio server
Copy
Phase 2: MacBook Client
1. Create MacBook client script audio_client.py:

#!/usr/bin/env python3
"""
MacBook Audio Client for Drone
Handles local audio I/O, communicates with Ubuntu PC
"""

import requests
import pyaudio
import wave
import io
import time

# Your Ubuntu PC's Tailscale IP
DRONE_API = "http://100.64.1.5:8000"  # Replace with your Tailscale IP

class DroneAudioClient:
    def __init__(self):
        self.audio = pyaudio.PyAudio()
        
    def play_drone_speech(self, text):
        """Request drone to speak, play audio on MacBook"""
        print(f"Requesting drone to say: {text}")
        
        response = requests.post(
            f"{DRONE_API}/drone/speak",
            params={"text": text}
        )
        
        if response.status_code == 200:
            # Play audio
            audio_data = io.BytesIO(response.content)
            wf = wave.open(audio_data, 'rb')
            
            stream = self.audio.open(
                format=self.audio.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True
            )
            
            print("🔊 Playing on MacBook speakers...")
            data = wf.readframes(1024)
            while data:
                stream.write(data)
                data = wf.readframes(1024)
            
            stream.close()
            print("✅ Playback complete")
        
    def record_and_send_command(self, duration=5):
        """Record voice command and send to drone"""
        print(f"🎤 Recording for {duration} seconds...")
        
        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )
        
        frames = []
        for i in range(0, int(16000 / 1024 * duration)):
            data = stream.read(1024)
            frames.append(data)
        
        stream.close()
        print("✅ Recording complete")
        
        # Save to WAV
        buffer = io.BytesIO()
        wf = wave.open(buffer, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.audio.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)
        wf.writeframes(b''.join(frames))
        wf.close()
        buffer.seek(0)
        
        # Send to drone
        print("📤 Sending to drone...")
        response = requests.post(
            f"{DRONE_API}/voice/command",
            files={"audio": ("command.wav", buffer, "audio/wav")}
        )
        
        result = response.json()
        print(f"📝 Transcribed: {result['transcription']}")
        return result['transcription']

def main():
    client = DroneAudioClient()
    
    print("\n=== Drone Audio Test ===\n")
    
    # Test drone speaking
    client.play_drone_speech("Hello, I am the surveillance drone. How may I assist you?")
    time.sleep(1)
    
    # Test voice command
    input("\nPress Enter to record voice command...")
    command = client.record_and_send_command(duration=3)
    
    # Drone responds
    client.play_drone_speech(f"You said: {command}. Processing your request.")

if __name__ == "__main__":
    main()
Copy
2. Install dependencies on MacBook:

# On MacBook
pip install pyaudio requests
Copy
Phase 3: Integrate with VLM Controller
Update vlm_drone_controller.py to use audio:

class VLMDroneController:
    def __init__(self):
        # ... existing code ...
        self.audio_api = "http://localhost:8000"
        
    def speak(self, text):
        """Make drone speak (audio plays on user's MacBook)"""
        requests.post(f"{self.audio_api}/drone/speak", params={"text": text})
        
    def execute_mission(self, instruction):
        # ... existing code ...
        
        # Add speaking
        self.speak("I see a person ahead. Approaching carefully.")
        
        # Move forward
        self.drone.move_forward(2.0)
        
        # Analyze
        response = self.vision.analyze_scene("What do you see?")
        
        # Speak result
        self.speak(f"I observe: {response}")
Copy
Resource Check:
With 6GB free VRAM, you can run:

✅ Qwen2.5-VL-3B (4-bit): ~2-3GB
✅ Whisper Base: ~1GB
✅ TTS Model: ~500MB
Total: ~3.5-4.5GB ✅ FITS!
Testing:
# On Ubuntu PC
docker-compose up -d vlm_controller

# On MacBook (via Tailscale)
python audio_client.py
Copy
This creates a fully remote-controlled talking drone - you speak to it from your MacBook, it processes with VLM on Ubuntu, and responds through your MacBook speakers!

Want to implement this? It's actually quite straightforward!
