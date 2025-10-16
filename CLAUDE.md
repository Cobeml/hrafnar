# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Hrafnar** is an AI-driven autonomous military drone simulation system for testing autonomous behaviors in denied/degraded communication environments. The system combines:

- **Physics simulation**: Isaac Sim 4.5+ with Pegasus Simulator for drone simulation
- **Flight control**: PX4 SITL (Software-In-The-Loop) via MAVLink protocol
- **AI autonomy**: Vision-Language Models (Qwen2.5-VL-3B) for vision-based command interpretation
- **Multi-language support**: DeepL translation API for multilingual voice/text control

The project simulates autonomous military drones that can operate with minimal ground control, using on-board AI to interpret commands and navigate using camera vision.

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────────┐
│  Isaac Sim +    │────▶│   ROS2 Bridge    │────▶│  Mission Controller │
│  PX4 SITL       │     │  (ROS2 Humble)   │     │  (Orchestration)    │
│  (Simulation)   │◀────│                  │◀────│                     │
└─────────────────┘     └──────────────────┘     └──────────┬──────────┘
         │                      │                           │
         │                      │                           │
         ▼                      ▼                           ▼
   MAVLink UDP           ROS2 Topics              ┌──────────────────┐
    (14540)         (/drone1/camera, etc.)       │  VLM Controller  │
                            │                     │  (Qwen2.5-VL)   │
                            │                     └────────┬─────────┘
                            │                              │
                            ▼                              │
                    ┌───────────────┐                     │
                    │ YOLO Tracker  │                     │
                    │ (YOLOv11)     │                     │
                    └───────┬───────┘                     │
                            │                              │
                            ▼                              ▼
                    /drone1/yolo_detections      /drone1/vlm_response
                            │                              │
                            └──────────┬───────────────────┘
                                       │
                                       ▼
                                ┌──────────────┐
                                │  FastAPI     │
                                │  (8002)      │
                                │              │
                                │ MacBook      │
                                │ Client       │
                                └──────────────┘
```

## Key Components

### 1. Drone Controller (`drone_controller.py`)
- Low-level MAVLink interface to PX4 autopilot
- Provides high-level movement primitives: `move_forward()`, `move_right()`, `move_backward()`, `move_left()`, `climb()`, `descend()`, `rotate()`, `land()`
- **Movements are camera-relative (body-frame)**: `move_forward()` moves in the direction the camera faces
- Internally uses NED (North-East-Down) coordinate frame where Z is negative for altitude
- Tracks current yaw from ATTITUDE messages and transforms body-frame commands to NED
- Maintains continuous setpoint streaming required for PX4 OFFBOARD mode

### 2. VLM Controller (`llm_controller/vlm_drone_controller.py`)
- ROS2 node that subscribes to `/drone1/camera/color/image_raw`
- Uses Qwen2.5-VL-3B (4-bit quantized) for vision-language processing
- Interprets natural language commands with camera context
- **Visual Grounding (Oct 2025)**: Annotates camera images with YOLO bounding boxes before VLM processing
  - Colored boxes labeled with ID, position (LEFT/CENTER/RIGHT), and confidence
  - Improves spatial reasoning by visually aligning detections with camera view
- Structured prompt engineering with tool-calling format:
  - VISUAL_CHECK: Verify bounding boxes in image
  - YOLO_CHECK: Cross-reference with YOLO text
  - GROUNDING: Confirm visual-text alignment
  - THINKING/OBSERVATION/ACTION/PARAMS/REASONING
- Executes actions via `DroneController`

### 3. VLM API Server (`llm_controller/vlm_api.py`)
- FastAPI server on port 8002
- Endpoints:
  - `POST /command?text_command=<command>` - Process and execute command
  - `GET /status` - Get drone position, armed state, camera status
- Includes fallback regex parser when camera feed unavailable

### 4. State API (`llm_controller/state_api.py`)
- Lightweight ROS2 state collector on port 8003
- Subscribes to `/drone1/state/pose`, `/drone1/state/twist`, `/drone1/sensors/gps`
- Uses BEST_EFFORT QoS to match Isaac Sim publishers
- `GET /state` endpoint for real-time drone telemetry

### 5. MacBook Client (`macbook_client.py`)
- Text-based remote control interface
- Auto-detects input language and translates to English via DeepL
- Translates drone responses back to user's language
- Requires `DEEPL_API_KEY` in `.env` file

### 6. YOLO Tracker (`llm_controller/yolo_tracker.py`)
- ROS2 node using YOLOv11-small for real-time person detection
- Subscribes to `/drone1/camera/color/image_raw`
- Publishes detections to `/drone1/yolo_detections` as JSON
- Publishes annotated frames to `/drone1/yolo_annotated`
- Maintains persistent track IDs across frames for tracking
- Filters for person class (COCO class 0) only

### 7. Mission Controller (`llm_controller/mission_controller.py`)
- High-level event-driven orchestration for autonomous missions
- Integrates VLM + YOLO + DroneController for search-and-survey operations
- Event types: person detection, person lost, bbox changes, movement complete, timeouts
- **ReAct Framework (Oct 2025)**: Reflection loop after action execution
  - Observe → Act → Reflect → Follow-up
  - Verifies if actions achieved intended goals
  - Suggests corrective follow-up actions based on outcomes
- Maintains encounter history via `MissionDatabase`
- Configurable mission parameters: search altitude, approach distance, timeouts
- Supports interactive speech input during flight (simulated via console)

### 8. Mission Database (`llm_controller/mission_database.py`)
- SQLite database for tracking person encounters during missions
- Records: track_id, first_seen, last_seen, approached status, conversation logs
- Enables persistent mission state across drone runs
- Useful for multi-session reconnaissance missions

## Recent Improvements (October 2025)

### Phase 1: Visual Grounding + ReAct Reflection

Implemented research-backed improvements to address spatial reasoning and memory issues:

1. **Visual Bounding Box Overlays**: Camera images are annotated with colored YOLO bboxes before VLM processing
2. **Grounding Verification**: VLM explicitly cross-references visual boxes with text descriptions
3. **ReAct Reflection Loop**: After each action, VLM reflects on outcome and suggests follow-ups

**Expected Impact**: +30-50% improvement in spatial reasoning and person tracking

See `IMPROVEMENTS.md` for detailed documentation, testing instructions, and research references.

**Key Files Modified**:
- `llm_controller/vlm_drone_controller.py`: Visual annotation + grounding prompt
- `llm_controller/mission_controller.py`: Reflection loop integration

**Testing**:
```bash
docker compose build vlm_controller  # Rebuild after changes
docker compose up -d
docker compose logs -f vlm_controller  # Look for 🔄 Reflecting on action
docker exec -it hrafnar-vlm_controller-1 python3 /app/llm_controller/mission_controller.py
```

## Development Commands

### Operational Modes

Hrafnar supports **two operational modes** depending on your needs:

#### Mode 1: GUI Mode (Native Isaac Sim + ROS2)
**Use when**: Debugging, visualization, testing drone movements with visual feedback

**What runs where**:
- Isaac Sim: Native (with GUI window)
- PX4 SITL: Auto-launched by Isaac Sim
- ROS2 topics: Published by Isaac Sim's ROS2 bridge
- VLM Controller: Docker container
- Camera feed: ✅ Available via ROS2

**Launch**:
```bash
# Terminal 1: Start Isaac Sim with GUI + ROS2
./scripts/run_isaac_sim_gui.sh 9_people.py

# Terminal 2: Start VLM controller in Docker
docker compose up vlm_controller

# Terminal 3: Run client
python3 drone_llm_client.py
```

**Advantages**:
- ✅ Visual feedback (see drone movements in real-time)
- ✅ Camera feed published to ROS2
- ✅ VLM controller receives camera images
- ✅ Full debugging capabilities

**Disadvantages**:
- ❌ Requires GUI environment (can't run on headless servers)
- ❌ More resource intensive

#### Mode 2: Headless Mode (Full Docker) - **RECOMMENDED**
**Use when**: Running on servers, development with camera feed, long-duration tests

**What runs where**:
- Everything in Docker containers
- Isaac Sim runs headless
- GUI accessible via WebRTC in browser

**Launch**:
```bash
# Quick start (builds if needed, starts all services)
./scripts/start_headless_mode.sh

# OR manually:
docker compose build isaac_sim  # First time only
docker compose up -d

# View logs
docker compose logs -f isaac_sim
docker compose logs -f vlm_controller
```

**Access**:
- 📺 **WebRTC GUI**: http://localhost:8211/streaming/webrtc-client/
- 🦊 **Foxglove**: ws://localhost:8765
- 🎛️ **ROSboard**: http://localhost:8888
- 🤖 **VLM API**: http://localhost:8002

**Advantages**:
- ✅ Runs on headless servers
- ✅ Camera feed works reliably (ROS2 topics publish correctly)
- ✅ Visual feedback via WebRTC browser GUI
- ✅ All dependencies containerized
- ✅ Pegasus Simulator auto-installs on startup

**Disadvantages**:
- ❌ WebRTC has slight latency vs native GUI
- ❌ Requires Docker + NVIDIA runtime

---

### Docker Compose Stack
```bash
# Start all services (Isaac Sim, ROS2 bridge, VLM controller)
docker compose up -d

# View logs
docker compose logs -f vlm_controller
docker compose logs -f ros2_bridge

# Restart VLM controller (after code changes)
docker compose restart vlm_controller

# Stop all services
docker compose down
```

### Running Isaac Sim with GUI (Mode 1)
```bash
# Use the provided launch script (sets up ROS2 environment)
./scripts/run_isaac_sim_gui.sh 9_people.py

# OR manually set environment and run:
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/isaacsim/exts/isaacsim.ros2.bridge/humble/lib
export ROS_DOMAIN_ID=0
~/isaacsim/python.sh 9_people.py

# Isaac Sim GUI will open with:
# - Minimal UI: 2 viewports (main god-view + drone camera) + console
# - Drone simulation visible
# - PX4 auto-launched
# - Camera feed published to /drone1/camera/color/image_raw
```

**UI Layout**:
```
┌────────────────────────────────────────────────┐
│  Main God View    │    Drone Camera Feed       │
│  (Scene Overview) │    (VLM's View)           │
│                   │                            │
│                   │                            │
├────────────────────────────────────────────────┤
│            Console (Logs)                      │
└────────────────────────────────────────────────┘
```
- **Left viewport**: Main god-mode view (overview of scene)
- **Right viewport**: Drone's onboard camera feed (what the VLM sees)
- **Bottom panel**: Console for logs and debugging
- **Closed**: Stage, Layer, Property, Content browser, and other unnecessary panels

**Important**: The launch script configures Isaac Sim's internal ROS2 so camera feeds are published. Without this, the VLM controller won't receive camera images.

### PX4 SITL (if running outside Docker)
```bash
cd ~/Developing/PX4-Autopilot/
make px4_sitl_default

# PX4 listens on UDP 14540 for MAVLink
```

### Testing Drone Control
```bash
# Direct MAVLink control (no VLM)
python3 drone_controller.py

# VLM control with camera
python3 llm_controller/vlm_drone_controller.py

# YOLO person tracker (standalone)
python3 llm_controller/yolo_tracker.py

# Autonomous mission with VLM + YOLO
python3 llm_controller/mission_controller.py

# Test autonomous mission scenarios
python3 llm_controller/test_autonomous_mission.py

# Start API server
python3 llm_controller/vlm_api.py

# Remote client from MacBook
python3 macbook_client.py
```

### ROS2 Topics (for debugging)
```bash
# Inside ROS2 container
docker exec -it hrafnar_ros2_bridge bash

source /opt/ros/humble/setup.bash

# List active topics
ros2 topic list

# Echo camera feed
ros2 topic echo /drone1/camera/color/image_raw

# Echo YOLO detections
ros2 topic echo /drone1/yolo_detections

# Check topic QoS
ros2 topic info /drone1/camera/color/image_raw -v

# View annotated YOLO output (requires visualization)
ros2 run rqt_image_view rqt_image_view /drone1/yolo_annotated
```

## Important Technical Details

### Coordinate Systems
- **Body Frame (Camera-Relative)**: All movement commands are relative to the drone's orientation
  - `move_forward()` = move in the direction the camera faces
  - `move_right()` = move to the camera's right side
  - `move_backward()` = move opposite to camera direction
  - `move_left()` = move to the camera's left side
  - After rotating, movements automatically follow the new camera direction
- **NED (North-East-Down)**: Internal coordinate system used by PX4/MAVLink
  - X = north, Y = east, Z = down (negative altitude)
  - Body-frame movements are transformed to NED using current yaw
  - Example: `z=-1.5` means 1.5m above ground
- **Backward compatibility**: Negative distances still work (e.g., `move_forward(-2.0)` = backward)

### ROS2 QoS Profiles
Isaac Sim publishes with `BEST_EFFORT` reliability. All ROS2 subscribers MUST use:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```
Using `RELIABLE` will cause subscription failures.

### VLM Prompt Format
The VLM expects structured outputs with visual grounding (updated Oct 2025):
```
VISUAL_CHECK: <bounding boxes visible in image: ID, position, color>
YOLO_CHECK: <YOLO text descriptions>
GROUNDING: <verification that visual and text align>
THINKING: <step-by-step spatial reasoning>
OBSERVATION: <what the camera sees>
ACTION: <move_forward|move_right|climb|descend|rotate|land>
PARAMS: {"distance": 2.0} or {"degrees": 90}
REASONING: <why this action>
```
See `SYSTEM_PROMPT` in `vlm_drone_controller.py` for full template with examples.

### YOLO Detection Format
Published to `/drone1/yolo_detections` as JSON:
```json
{
  "detections": [
    {
      "id": 1,
      "bbox": [x1, y1, x2, y2],
      "center": [cx, cy],
      "confidence": 0.87,
      "area": 12500
    }
  ],
  "count": 1,
  "timestamp": 1696234567.89
}
```
- `id`: Persistent track ID maintained across frames (YOLOv11 tracking)
- `bbox`: Bounding box in pixel coordinates
- `center`: Center point for targeting
- `area`: Used to determine if person is approaching/receding

### MAVLink Connection
- Default: `udp:127.0.0.1:14540`
- Must send setpoints at ≥20Hz for OFFBOARD mode
- Arming requires OFFBOARD mode active first
- Position updates arrive via `LOCAL_POSITION_NED` messages

### Battery Failsafe Handling
PX4 SITL simulates battery depletion which can interrupt long missions. The `DroneController` automatically disables battery failsafes on connection via three methods:
1. `SIM_BAT_ENABLE=0` - Disables battery simulation entirely
2. `COM_LOW_BAT_ACT=0` - Disables low battery failsafe action
3. `SIM_BAT_MIN_PCT=0.9` - Keeps battery at 90% minimum

To enable battery simulation (for testing battery logic), pass `disable_battery_failsafe=False` to `DroneController()`.

**Important**: PX4 parameters reset after `make clean`, so battery settings must be reconfigured if PX4 is rebuilt.

### GPU Requirements
- VLM controller requires NVIDIA GPU with CUDA 12.4+
- Model runs in 4-bit quantization (~3-4GB VRAM)
- Docker containers need `runtime: nvidia` and `NVIDIA_VISIBLE_DEVICES=all`

## Configuration Files

### Environment Variables (`.env`)
```
OPENAI_API_KEY=sk-...          # For OpenAI-based features (not currently used)
PERPLEXITY_API_KEY=...         # Optional
deepl_api_key=...              # Required for macbook_client.py translation
```

### Docker Compose Services
- `isaac_sim`: Isaac Sim 4.5.0 container (if using containerized Isaac)
- `ros2_bridge`: ROS2 Humble + ROSboard dashboard (port 8888)
- `foxglove_bridge`: Foxglove WebSocket bridge (port 8765)
- `vlm_controller`: VLM API server (port 8002)

### Ports
- 8002: VLM Command API
- 8003: State API (telemetry)
- 8888: ROSboard dashboard
- 8765: Foxglove bridge
- 14540: MAVLink (PX4 SITL)

## Common Workflows

### Running an Autonomous Mission
1. Start headless mode: `./scripts/start_headless_mode.sh`
2. Wait for Isaac Sim + ROS2 bridge to initialize (~60s)
3. In separate terminal: `python3 llm_controller/mission_controller.py`
4. Mission will:
   - Takeoff to search altitude
   - Use YOLO to detect people in camera feed
   - Approach detected persons
   - Use VLM to analyze scene and engage in conversation
   - Track encounters in SQLite database
   - Continue search pattern until mission timeout

### Adding New Drone Actions
1. Add method to `DroneController` class in `drone_controller.py`
2. Update `SYSTEM_PROMPT` in `vlm_drone_controller.py` with new tool
3. Add case in `execute_action()` method
4. Update response parser if needed

### Adding New Mission Behaviors
1. Define new `EventType` in `mission_controller.py`
2. Add event detection logic in `_detect_events()`
3. Implement handler method (e.g., `_handle_new_behavior()`)
4. Add case in `_event_loop()` switch statement
5. Update `MissionConfig` if new parameters are needed

### Testing Without Isaac Sim
The system requires Isaac Sim running for:
- Camera feed (`/drone1/camera/color/image_raw`)
- Position updates (MAVLink `LOCAL_POSITION_NED`)

For testing VLM parsing without simulation, use the fallback parser in `vlm_api.py` which works without vision.

### Debugging ROS2 Issues
1. Check QoS compatibility: `ros2 topic info <topic> -v`
2. Verify publishers: `ros2 topic list`
3. Echo raw messages: `ros2 topic echo <topic>`
4. Check for topic name mismatches (e.g., `/drone1/` prefix)

### Model Downloads
First run downloads models to `~/.cache/huggingface/`. This is mounted in Docker via:
```yaml
volumes:
  - ~/.cache/huggingface:/root/.cache/huggingface
```

**Downloaded models**:
- Qwen2.5-VL-3B-Instruct (~7GB) - Vision-Language Model
- yolo11s.pt (~10MB) - YOLOv11 small variant for person detection

### Simulation Scene Configuration
The `9_people.py` script configures Isaac Sim with:
- **Scene**: "Curved Gridroom" environment from Pegasus Simulator
- **People**: Multiple animated characters (construction worker, business person, etc.)
- **Person controllers**: Characters can move in patterns (circles) or to fixed waypoints
- **Drone**: Single Iris quadcopter with PX4 autopilot
- **Sensors**: RGB camera mounted on drone (640x480 default)
- **WebRTC**: Enabled for browser-based GUI access in headless mode
- **ROS2**: Publishes camera, pose, twist, GPS to ROS2 topics

To modify the scene:
1. Edit `9_people.py` to change environment, add/remove people, adjust spawn positions
2. Restart Isaac Sim container: `docker compose restart isaac_sim`

## Data Flow and Decision Loop

### Autonomous Mission Flow (mission_controller.py)
```
1. Takeoff → Search altitude
2. Begin search pattern (grid/spiral)
3. YOLO detects person in camera feed
4. Event: NEW_PERSON_DETECTED
   → Approach person (move toward bbox center)
5. Event: BBOX_CHANGED_SIGNIFICANTLY
   → Adjust position to maintain view
6. Event: MOVEMENT_COMPLETE
   → VLM analyzes scene
   → VLM generates conversation/question
   → User provides speech input (console simulation)
   → VLM processes response
7. Event: CONVERSATION_COMPLETE
   → Log encounter to database
   → Mark person as "approached"
   → Resume search pattern
8. Repeat until mission timeout or manual stop
```

### Event-Driven Architecture
The `MissionController` monitors multiple event sources:
- **YOLO tracker**: New detections, lost tracks, bbox changes
- **Drone state**: Movement completion, position updates
- **User input**: Speech/text commands during flight
- **Timers**: No detection timeout, mission duration timeout

This event-driven model enables reactive behaviors without blocking the main loop.

## Strategic Context

This system is designed for testing autonomous behaviors in military scenarios where:
- Communication is denied/degraded (DDIL environments)
- Drones must operate with minimal ground control
- Visual interpretation is required for navigation
- Multi-language command interfaces support diverse operators

Target scenarios include amphibious defense, reconnaissance, swarm coordination, and outlying island operations (see `application.md` and `spec.md` for detailed mission profiles).

## Dependencies

**External installations** (not in requirements.txt):
- NVIDIA Isaac Sim 4.5+
- Pegasus Simulator 4.5+
- PX4 Autopilot v1.14+
- Docker + NVIDIA Container Toolkit

**Python packages** (see `requirements.txt`):
- pymavlink (MAVLink protocol)
- transformers, torch (VLM inference)
- fastapi, uvicorn (API servers)
- rclpy (ROS2 Python bindings, installed via Docker)
- qwen-vl-utils, bitsandbytes (quantization)
- ultralytics (YOLOv11 for person detection)
- cv_bridge (ROS2 image conversion)
- python-dotenv, pyyaml (config)

## Network Configuration

Default setup assumes:
- Drone API at `100.99.98.39:8002` (update `DRONE_API` in `macbook_client.py`)
- All services on same host using `network_mode: host` in Docker

For remote access, ensure firewall allows ports 8002, 8003, 8888, 8765.
