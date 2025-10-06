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
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Isaac Sim +    │────▶│   ROS2 Bridge    │────▶│  VLM Controller │
│  PX4 SITL       │     │  (ROS2 Humble)   │     │  (Qwen2.5-VL)   │
│  (Simulation)   │◀────│                  │◀────│                 │
└─────────────────┘     └──────────────────┘     └─────────────────┘
         │                      │                         │
         │                      │                         │
         ▼                      ▼                         ▼
   MAVLink UDP           ROS2 Topics              FastAPI (8002)
    (14540)         (/drone1/camera, etc.)             │
                                                        ▼
                                                ┌──────────────┐
                                                │ MacBook      │
                                                │ Client       │
                                                │ (Any Lang)   │
                                                └──────────────┘
```

## Key Components

### 1. Drone Controller (`drone_controller.py`)
- Low-level MAVLink interface to PX4 autopilot
- Provides high-level movement primitives: `move_forward()`, `move_right()`, `climb()`, `descend()`, `rotate()`, `land()`
- Uses NED (North-East-Down) coordinate frame where Z is negative for altitude
- Maintains continuous setpoint streaming required for PX4 OFFBOARD mode

### 2. VLM Controller (`llm_controller/vlm_drone_controller.py`)
- ROS2 node that subscribes to `/drone1/camera/color/image_raw`
- Uses Qwen2.5-VL-3B (4-bit quantized) for vision-language processing
- Interprets natural language commands with camera context
- Structured prompt engineering with tool-calling format (OBSERVATION/ACTION/PARAMS/REASONING)
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

## Development Commands

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

### Running Isaac Sim (Standalone)
```bash
# Isaac Sim is typically installed at ~/isaacsim/
# Pegasus Simulator at ~/Developing/PegasusSimulator/

# Start Isaac Sim with Pegasus example
cd ~/Developing/PegasusSimulator/
$ISAACSIM_PYTHON_EXE standalone/quadcopter_cameras_example.py

# Wait for Isaac Sim GUI, then click PLAY button
```

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

# Check topic QoS
ros2 topic info /drone1/camera/color/image_raw -v
```

## Important Technical Details

### Coordinate Systems
- **NED (North-East-Down)**: PX4/MAVLink use NED where:
  - X = forward (north)
  - Y = right (east)
  - Z = down (negative altitude)
- Example: `z=-1.5` means 1.5m above ground
- To move left: `move_right(-2.0)` (negative distance)
- To move backward: `move_forward(-2.0)`

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
The VLM expects structured outputs:
```
OBSERVATION: <what the camera sees>
ACTION: <move_forward|move_right|climb|descend|rotate|land>
PARAMS: {"distance": 2.0} or {"degrees": 90}
REASONING: <why this action>
```
See `SYSTEM_PROMPT` in `vlm_drone_controller.py` for full template.

### MAVLink Connection
- Default: `udp:127.0.0.1:14540`
- Must send setpoints at ≥20Hz for OFFBOARD mode
- Arming requires OFFBOARD mode active first
- Position updates arrive via `LOCAL_POSITION_NED` messages

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

### Adding New Drone Actions
1. Add method to `DroneController` class in `drone_controller.py`
2. Update `SYSTEM_PROMPT` in `vlm_drone_controller.py` with new tool
3. Add case in `execute_action()` method
4. Update response parser if needed

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

### Model Download
First run downloads Qwen2.5-VL-3B-Instruct (~7GB) to `~/.cache/huggingface/`. This is mounted in Docker via:
```yaml
volumes:
  - ~/.cache/huggingface:/root/.cache/huggingface
```

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
- python-dotenv, pyyaml (config)

## Network Configuration

Default setup assumes:
- Drone API at `100.99.98.39:8002` (update `DRONE_API` in `macbook_client.py`)
- All services on same host using `network_mode: host` in Docker

For remote access, ensure firewall allows ports 8002, 8003, 8888, 8765.
