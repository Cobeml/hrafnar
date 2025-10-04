# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Hrafnar** is an AI-driven autonomous military drone system designed for operations in denied, degraded, intermittent, and limited (DDIL) communication environments. The system uses small language models (SLMs) and vision-language models (VLMs) running locally on drone platforms for intelligent decision-making with minimal ground control dependency.

Primary application: Taiwan Strait defense scenarios, swarm coordination, amphibious assault defense.

## System Architecture

### Dual-Mode Operation

1. **High-Fidelity Simulation**: Full physics-based simulation using Isaac Sim + PX4 autopilot + Pegasus Simulator
2. **Strategic Planning Mode**: Abstracted engagement simulation with analytics dashboard (planned)

### Technology Stack

- **Simulation**: NVIDIA Isaac Sim 4.5+, Pegasus Simulator 4.5+, PX4 Autopilot v1.14+
- **ROS2**: Humble (running in Docker on Ubuntu 24.04)
- **Languages**: Python 3.10+
- **AI/ML** (planned): SLMs (Phi-3, Llama 3.2, Qwen2.5), VLMs (Phi-3-Vision, Moondream2)
- **Communication**: MAVLink (PX4 ↔ companion), ROS2 topics/services

## Key Paths and Locations

```
/home/cobe-liu/isaacsim/                    # Isaac Sim installation
/home/cobe-liu/Developing/PegasusSimulator/ # Pegasus Simulator framework
/home/cobe-liu/PX4-Autopilot/               # PX4 autopilot (SITL)
/home/cobe-liu/Developing/hrafnar/          # This repository

# Project structure:
├── 9_people.py                    # Example: drone + people simulation
├── spec.md                        # Full technical specification
├── application.md                 # Military application scenarios
├── requirements.txt               # Python dependencies
├── docker-compose.yml             # ROS2 services (rosboard, foxglove, rosbridge)
├── scripts/
│   └── run_with_docker_ros2.sh   # Primary execution script
└── ros2_ws/                       # ROS2 workspace (empty initially)
```

## Development Commands

### Environment Setup

```bash
# Set Isaac Sim Python environment (required before running)
export ISAACSIM_PYTHON_EXE=/home/cobe-liu/isaacsim/python.sh

# Install Python dependencies
pip install -r requirements.txt
```

### Running Simulations

```bash
# Primary method: Run with Docker ROS2 bridge + ROSboard dashboard
./scripts/run_with_docker_ros2.sh 9_people.py

# Direct execution (if ROS2 already running)
$ISAACSIM_PYTHON_EXE 9_people.py
```

### ROS2 Management (Docker)

```bash
# Start ROS2 services
docker-compose up -d ros2_bridge

# Start with Foxglove bridge
docker-compose --profile foxglove up -d

# Start with ROSbridge WebSocket
docker-compose --profile rosbridge up -d

# Stop all services
docker-compose down

# View ROS2 topics
docker-compose exec ros2_bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Check logs
docker-compose logs -f ros2_bridge
```

### Dashboards

- **ROSboard**: http://localhost:8888 (web-based ROS2 visualization)
- **Foxglove**: ws://localhost:8765 (if foxglove profile enabled)

### Other Tools

```bash
# QGroundControl (PX4 ground station)
~/QGroundControl.AppImage

# Build PX4 SITL (if needed)
cd ~/Developing/PX4-Autopilot
make px4_sitl_default
```

## Critical Configuration Notes

### ROS2 Domain ID

**Must match between Isaac Sim and Docker containers**: `ROS_DOMAIN_ID=0`

The `run_with_docker_ros2.sh` script automatically sets:
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_DOMAIN_ID=0`
- `LD_LIBRARY_PATH` to include Isaac Sim's ROS2 libraries

### Python Script Requirements

All simulation scripts must:

1. **Import and initialize SimulationApp first**:
   ```python
   from isaacsim import SimulationApp
   simulation_app = SimulationApp({"headless": False})
   # ALL other imports come after this
   ```

2. **Enable ROS2 bridge extension**:
   ```python
   from isaacsim.core.utils.extensions import enable_extension
   enable_extension("isaacsim.ros2.bridge")
   enable_extension("foxglove.tools.ws_bridge")  # optional
   ```

3. **Configure drone backends**:
   ```python
   from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
   from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend

   mavlink_config = PX4MavlinkBackendConfig({
       "vehicle_id": 0,
       "px4_autolaunch": True,
       "px4_dir": "/home/cobe-liu/PX4-Autopilot"
   })

   config_multirotor.backends = [
       PX4MavlinkBackend(mavlink_config),
       ROS2Backend(
           vehicle_id=1,
           config={
               "namespace": 'drone',
               "pub_sensors": True,
               "pub_graphical_sensors": True,
               "pub_state": True,
               "pub_tf": True,
               "sub_control": False
           }
       )
   ]
   ```

### Conda Environment Conflicts

The `run_with_docker_ros2.sh` script automatically deactivates conda environments, as they conflict with Isaac Sim's internal Python.

## Simulation Components

### Available from Pegasus Simulator

```python
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.people.person import Person, PersonController
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
```

- **Robots**: `ROBOTS['Iris']` (quadrotor platform)
- **Environments**: `SIMULATION_ENVIRONMENTS["Curved Gridroom"]`, etc.
- **Sensors**: MonocularCamera, IMU, GPS, LiDAR (see Pegasus docs)

### Typical Simulation Structure

```python
class PegasusApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load environment
        self.pg.load_asset(SIMULATION_ENVIRONMENTS["Curved Gridroom"], "/World/layout")

        # Create multirotor with backends
        Multirotor("/World/quadrotor", ROBOTS['Iris'], 0, [0.0, 0.0, 0.07], ...)

        # Reset to initialize
        self.world.reset()

    def run(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
```

## Military Application Context

**Primary scenarios** (see `application.md` for details):
- Amphibious assault defense (high-value targets: Type 075 LHDs, Type 071 LPDs, RO-RO ferries)
- Blockade/quarantine operations
- ISR for coastal defense fires
- Outlying island defense (Kinmen, Matsu)
- Counter-swarm operations

**Design principles**:
- Autonomous decision-making in DDIL environments
- Swarm coordination with mesh networking
- Cost-asymmetric warfare (cheap drones vs expensive ships)
- Rapid iteration on prompt engineering and tactical behaviors

## AI/ML Integration (Planned)

**Model requirements**:
- **SLM**: <100ms inference, <8GB RAM, structured JSON output (Phi-3-mini, Llama 3.2, Qwen2.5)
- **VLM**: <200ms frame analysis, object detection, low-res tolerance (Phi-3-Vision, Moondream2)

**Prompt structure**: System role + mission objectives + constraints + sensor data → JSON decision output

See `spec.md` sections 4 (AI Model Integration) and 10 (Prompt Engineering) for detailed design.

## External Dependencies

Must be installed separately (not in requirements.txt):
- Isaac Sim 4.5+ (~/isaacsim/)
- Pegasus Simulator 4.5+ (~/Developing/PegasusSimulator/)
- PX4 Autopilot v1.14+ (~/Developing/PX4-Autopilot/)
- Docker (for ROS2)
- QGroundControl (optional, ~/QGroundControl.AppImage)

## Development Workflow

1. **Set environment**: `export ISAACSIM_PYTHON_EXE=/home/cobe-liu/isaacsim/python.sh`
2. **Start Docker ROS2** (if needed): `docker-compose up -d ros2_bridge`
3. **Run simulation**: `./scripts/run_with_docker_ros2.sh <script.py>` or `$ISAACSIM_PYTHON_EXE <script.py>`
4. **Monitor via ROSboard**: http://localhost:8888
5. **Control via QGroundControl**: `~/QGroundControl.AppImage`

## Common Issues

- **"ISAACSIM_PYTHON_EXE not set"**: Run `export ISAACSIM_PYTHON_EXE=/home/cobe-liu/isaacsim/python.sh`
- **ROS2 topics not visible**: Check `ROS_DOMAIN_ID=0` in both Isaac Sim and Docker
- **Conda conflicts**: Script auto-deactivates, or manually run `conda deactivate`
- **Docker not running**: `sudo systemctl start docker`
- **PX4 fails to launch**: Check `px4_dir` path in `PX4MavlinkBackendConfig`
