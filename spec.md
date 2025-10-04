# Hrafnar: Autonomous Military Drone System - Technical Specification

## 1. Executive Summary

**Project Name:** Hrafnar

**Version:** 1.0

**Last Updated:** October 1, 2025

Hrafnar is an AI-driven autonomous drone system designed for military operations in denied, degraded, intermittent, and limited (DDIL) communication environments. The system leverages small language models (SLMs) and vision-language models (VLMs) running locally on drone platforms to enable intelligent decision-making with minimal ground control dependency.

The project consists of two primary operational modes:

1. **High-Fidelity Simulation Mode** : Full physics-based simulation using PX4 autopilot and Isaac Sim via Pegasus Simulator
2. **Strategic Planning Mode** : Abstracted engagement simulation with analytics dashboard for mission planning and prompt engineering

---

## 2. System Architecture

### 2.1 Core Components

```
┌─────────────────────────────────────────────────────────────┐
│                     Hrafnar System                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────────────┐    ┌──────────────────────────┐ │
│  │  Strategic Planning  │    │   Simulation Engine      │ │
│  │      Module          │    │                          │ │
│  │                      │    │  ├─ Physics Sim (Isaac) │ │
│  │  ├─ Analytics UI     │    │  ├─ PX4 SITL           │ │
│  │  ├─ Mission Planner  │    │  ├─ Pegasus Interface  │ │
│  │  ├─ Prompt Engineer  │    │  └─ Sensor Models      │ │
│  │  └─ Engagement Sim   │    │                          │ │
│  └──────────────────────┘    └──────────────────────────┘ │
│              │                           │                  │
│              └───────────┬───────────────┘                  │
│                          │                                  │
│                ┌─────────▼──────────┐                      │
│                │  Autonomy Stack    │                      │
│                │                    │                      │
│                │  ├─ SLM Runtime    │                      │
│                │  ├─ VLM Runtime    │                      │
│                │  ├─ Decision Core  │                      │
│                │  ├─ State Manager  │                      │
│                │  └─ Comm Handler   │                      │
│                └────────────────────┘                      │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Technology Stack

**Simulation Layer:**

* **Isaac Sim** : NVIDIA physics simulation engine
* **Pegasus Simulator** : ROS2/Isaac Sim bridge for drone simulation
* **PX4 Autopilot** : Flight control software (SITL mode)
* **MAVLink** : Communication protocol between autopilot and companion computer

**AI/ML Layer:**

* **Small Language Models** : 1-7B parameter models (e.g., Phi-3, Llama 3.2, Qwen2.5)
* **Vision-Language Models** : Multimodal models (e.g., LLaVA, Moondream, Phi-3-Vision)
* **Inference Runtime** : ONNX Runtime, llama.cpp, or MLX (depending on deployment target)

**Development Stack:**

* **Languages** : Python (primary), C++ (performance-critical components)
* **Frameworks** : ROS2 (Humble/Iron), PyTorch, ONNX
* **Build System** : Colcon (ROS2), CMake
* **Containerization** : Docker for reproducible environments

---

## 3. System Modes

### 3.1 High-Fidelity Simulation Mode

**Purpose:** Test autonomy algorithms in realistic physics-based scenarios with full sensor simulation and flight dynamics.

**Components:**

#### 3.1.1 Physics Simulation

* **Isaac Sim Integration** : Photorealistic 3D environments with accurate physics
* **Sensor Models** :
* Camera (RGB, depth)
* LiDAR
* IMU, GPS, magnetometer, barometer
* Optical flow
* **Environmental Factors** : Wind, lighting conditions, terrain

#### 3.1.2 Flight Control Stack

* **PX4 SITL** : Software-in-the-loop autopilot simulation
* **Pegasus Simulator** : Bridges Isaac Sim with PX4 via MAVLink
* **Flight Modes** : Position, velocity, attitude control
* **Safety Systems** : Geofencing, return-to-launch, failsafes

#### 3.1.3 Autonomy Integration

* **Perception Pipeline** :
* VLM processes camera feeds for object detection, scene understanding
* Feature extraction for navigation and threat assessment
* **Decision Engine** :
* SLM receives sensor data, mission objectives, and system prompts
* Generates high-level commands (waypoints, behaviors, tactical decisions)
* **Control Interface** :
* Translates AI decisions into MAVLink commands
* Monitors vehicle state and enforces constraints

**Data Flow:**

```
Isaac Sim → Sensors → VLM → Feature Vector →
                              ↓
Mission Prompt → SLM ← Vehicle State ← PX4
                  ↓
            MAVLink Commands → PX4 → Vehicle Control
```

### 3.2 Strategic Planning Mode

**Purpose:** Rapid iteration on mission planning, prompt engineering, and tactical analysis without computational overhead of full physics simulation.

**Components:**

#### 3.2.1 Abstracted Engagement Simulator

* **Simplified Physics** : Kinematic models without full dynamics
* **Agent Representation** :
* Position, velocity, heading
* Health/status
* Sensor footprint (detection ranges)
* **Environment Model** :
* 2D/2.5D terrain
* No-fly zones, objectives, threats
* **Combat Model** :
* Detection probability based on range/conditions
* Engagement outcomes (simplified)
* Communication ranges and reliability

#### 3.2.2 Analytics Dashboard

**Mission Planning Interface:**

* **Scenario Builder** :
* Define terrain, objectives, threats
* Set initial drone positions and quantities
* Configure enemy forces and behaviors
* **Fleet Configuration** :
* Assign roles (reconnaissance, strike, support)
* Set individual drone capabilities
* Configure swarm parameters
* **Timeline Control** : Playback, pause, step-through simulation
* **Performance Metrics** :
* Mission success rate
* Drone survivability
* Objective completion time
* Communication usage patterns

**Prompt Engineering Workspace:**

* **System Prompt Editor** :
* Base instructions for all drones
* Role-specific prompt templates
* Mission-specific directives
* **Prompt Variables** :
* Dynamic insertion of mission parameters
* Real-time state information
* Tactical constraints and rules of engagement
* **Version Control** : Track prompt iterations and performance
* **A/B Testing** : Compare multiple prompt strategies

**Analytics Visualization:**

* **Real-time Dashboards** :
* Drone positions and trajectories
* Decision logs from SLM
* Communication network topology
* Threat detection timeline
* **Post-Mission Analysis** :
* Heatmaps of drone activity
* Decision tree visualization
* Failure mode analysis
* Resource utilization graphs

#### 3.2.3 Engagement Simulation Engine

**Agent Behavior:**

* Each drone runs autonomous decision loop
* SLM receives:
  * System prompt (from dashboard)
  * Current state (position, fuel, ammo, damage)
  * Sensor data (detected entities within range)
  * Mission objectives
  * Communication messages (if available)
* SLM outputs structured decisions (JSON):
  ```json
  {  "action": "move_to_position",  "target": [x, y, z],  "speed": 15.0,  "altitude_mode": "terrain_follow",  "communication": "Send recon data to leader",  "reasoning": "Moving to investigate suspected target area"}
  ```

**Simulation Loop:**

```
1. Update all agent states (physics)
2. Process sensor detection (visibility checks)
3. Handle communications (range-limited)
4. Execute AI decision cycle for each agent
5. Update environment (threats, objectives)
6. Log all events and decisions
7. Check termination conditions
```

---

## 4. AI Model Integration

### 4.1 Model Selection Criteria

**Small Language Models (1-7B parameters):**

* **Inference Speed** : <100ms per decision on target hardware
* **Memory Footprint** : <8GB RAM for model + context
* **Reasoning Capability** : Chain-of-thought, multi-step planning
* **Instruction Following** : Strong prompt adherence
* **Structured Output** : JSON generation capability

**Recommended Models:**

* **Phi-3-mini (3.8B)** : Excellent reasoning, efficient
* **Llama 3.2 (3B)** : Good general capability
* **Qwen2.5 (1.5-7B)** : Strong multilingual and reasoning

**Vision-Language Models:**

* **Real-time Processing** : <200ms for frame analysis
* **Object Detection** : Military assets, terrain features, threats
* **Scene Understanding** : Contextual awareness
* **Low Res Tolerance** : Operate on compressed/degraded imagery

**Recommended Models:**

* **Phi-3-Vision** : Excellent quality-to-size ratio
* **Moondream2** : Very efficient, fast inference
* **LLaVA-1.6** : Strong general vision understanding

### 4.2 Model Optimization

**Quantization:**

* 4-bit or 8-bit quantization for reduced memory
* GGUF format for llama.cpp deployment
* ONNX quantization for cross-platform deployment

**Context Management:**

* **Context Window** : 2048-4096 tokens sufficient
* **Context Compression** : Summarize old information
* **Priority Queue** : Critical info stays in context

**Inference Optimization:**

* **Batch Processing** : Group decisions when possible
* **KV Cache** : Reuse computations across turns
* **Speculative Decoding** : Speed up token generation

### 4.3 Prompt Engineering

**System Prompt Structure:**

```
<role>
You are an autonomous military drone AI operating in [MISSION_TYPE].
Your designation is [DRONE_ID] with role: [ROLE].
</role>

<mission>
Primary Objective: [OBJECTIVE]
Secondary Objectives: [SECONDARY_OBJECTIVES]
Rules of Engagement: [ROE]
</mission>

<constraints>
- Maximum altitude: [MAX_ALT]m
- Communication: [COMM_STATUS]
- Fuel remaining: [FUEL]%
- Available weapons: [WEAPONS]
</constraints>

<capabilities>
[SENSOR_LIST]
[WEAPON_LIST]
[COMMUNICATION_MODES]
</capabilities>

<instructions>
[TACTICAL_GUIDANCE]
[COORDINATION_PROTOCOLS]
[SAFETY_PROCEDURES]
</instructions>

Given your current state and sensor data, provide your next action in JSON format.
```

**Decision Prompt Template:**

```
Current Status:
- Position: [LAT, LON, ALT]
- Heading: [HEADING]° at [SPEED] m/s
- Fuel: [FUEL]% | Ammo: [AMMO]
- Health: [HEALTH]%

Sensor Data:
[DETECTED_ENTITIES]

Recent Communications:
[MESSAGES]

Mission Progress:
[OBJECTIVE_STATUS]

Provide your decision as JSON with: action, target, parameters, reasoning
```

---

## 5. Communication Architecture

### 5.1 Communication Modes

**High-Fidelity Mode (Simulation):**

* **MAVLink** : PX4 <-> Companion Computer
* **ROS2 Topics/Services** : Inter-module communication
* **Simulated Radio** : Range-limited, lossy channel model

**Strategic Planning Mode:**

* **Event-Driven Messages** : Simplified message passing
* **Configurable Reliability** : Packet loss, delay, jamming simulation
* **Communication Budget** : Limited bandwidth forcing prioritization

### 5.2 Swarm Coordination

**Mesh Network Simulation:**

* **Peer-to-peer** : Direct drone-to-drone when in range
* **Multi-hop** : Messages relay through intermediaries
* **Leader Election** : Dynamic based on connectivity/capability
* **Information Fusion** : Aggregate sensor data across swarm

**Communication Protocols:**

* **Position Broadcasts** : Periodic location sharing
* **Task Assignment** : Dynamic mission allocation
* **Threat Alerts** : Priority messages
* **Status Updates** : Health, fuel, ammo reports

---

## 6. Data Management

### 6.1 Logging and Telemetry

**High-Fidelity Simulation:**

* **ULog** : PX4 flight logs
* **ROS2 Bags** : Full topic recording
* **AI Decision Logs** : JSON records of each decision cycle
* **Performance Metrics** : Inference time, memory usage

**Strategic Planning:**

* **Mission Logs** : Complete engagement timeline
* **Decision Database** : All AI outputs with inputs
* **Analytics Data** : Aggregated statistics
* **Replay Files** : Deterministic simulation replay

### 6.2 Data Schema

**Decision Record:**

```json
{
  "timestamp": 1696348952.342,
  "drone_id": "raven_01",
  "mission_tick": 1842,
  "state": {
    "position": [47.3977, 8.5456, 150.0],
    "velocity": [12.5, 0.3, -0.5],
    "fuel": 68.2,
    "health": 95.0
  },
  "perception": {
    "detected_entities": [...],
    "environment_assessment": "..."
  },
  "prompt": {
    "system": "...",
    "user": "..."
  },
  "ai_response": {
    "raw": "...",
    "parsed": {...},
    "confidence": 0.87
  },
  "action_taken": {...},
  "reasoning": "...",
  "inference_time_ms": 87.3
}
```

---

## 7. Development Roadmap

### Phase 1: Foundation (Weeks 1-4)

* [ ] Set up Isaac Sim + Pegasus Simulator environment
* [ ] PX4 SITL integration and basic flight control
* [ ] Model selection and quantization pipeline
* [ ] Basic ROS2 autonomy node skeleton

### Phase 2: Core Autonomy (Weeks 5-8)

* [ ] VLM integration for perception
* [ ] SLM decision engine implementation
* [ ] Prompt engineering framework
* [ ] Basic waypoint navigation with AI decisions

### Phase 3: Strategic Planning (Weeks 9-12)

* [ ] Abstracted engagement simulator
* [ ] Analytics dashboard (web-based, React/Vue)
* [ ] Prompt engineering UI
* [ ] Multi-agent simulation

### Phase 4: Advanced Features (Weeks 13-16)

* [ ] Swarm coordination algorithms
* [ ] Communication network simulation
* [ ] Threat response behaviors
* [ ] Mission replay and analysis tools

### Phase 5: Optimization & Testing (Weeks 17-20)

* [ ] Performance profiling and optimization
* [ ] Extensive scenario testing
* [ ] Documentation and tutorials
* [ ] Validation against military scenarios

---

## 8. System Requirements

### 8.1 Development Machine

**Minimum:**

* CPU: 8-core Intel/AMD
* GPU: NVIDIA RTX 3060 (12GB VRAM)
* RAM: 32GB
* Storage: 500GB SSD
* OS: Ubuntu 22.04 LTS

**Recommended:**

* CPU: 16-core Intel/AMD
* GPU: NVIDIA RTX 4090 (24GB VRAM)
* RAM: 64GB
* Storage: 1TB NVMe SSD
* OS: Ubuntu 22.04 LTS

### 8.2 Software Dependencies

* Isaac Sim 2023.1.1+
* PX4 Autopilot v1.14+
* ROS2 Humble
* Python 3.10+
* CUDA 12.1+
* PyTorch 2.1+

---

## 9. Key Technical Challenges

### 9.1 Real-time AI Inference

**Challenge:** Running VLM + SLM in control loop (<10Hz desired)

**Approach:**

* Aggressive quantization
* Asynchronous inference (stale but fast decisions acceptable)
* GPU optimization with TensorRT/ONNX Runtime

### 9.2 Prompt Context Management

**Challenge:** Limited context window vs. long mission duration

**Approach:**

* Sliding window with summarization
* Hierarchical memory (short-term tactical, long-term strategic)
* Importance-weighted context retention

### 9.3 Simulation Fidelity vs. Speed

**Challenge:** Isaac Sim is computationally expensive

**Approach:**

* Dual-mode architecture (high/low fidelity)
* Distributed simulation for swarms
* LOD techniques for sensors

### 9.4 Decision Validation

**Challenge:** Ensuring AI decisions are safe and tactically sound

**Approach:**

* Rule-based safety layer (geofencing, collision avoidance)
* Decision explanation requirements
* Human-in-the-loop for strategic planning mode
* Extensive offline testing before deployment

---

## 10. Security Considerations

* **Model Integrity** : Verify model weights haven't been tampered with
* **Prompt Injection** : Sanitize any external inputs to AI
* **Communication Security** : Encrypted channels (simulated)
* **Fail-safe Behaviors** : Default to safe actions on AI failure
* **Data Privacy** : No real operational data in development phase

---

## 11. Testing Strategy

### 11.1 Unit Testing

* Individual AI components (perception, decision, control)
* Model inference performance
* Communication protocols

### 11.2 Integration Testing

* Full autonomy stack in simple scenarios
* Multi-drone coordination
* Dashboard-to-simulation integration

### 11.3 Scenario Testing

* **Reconnaissance** : Area surveillance with minimal detection
* **Strike** : Target engagement with coordinated attack
* **Defensive** : Base protection and threat interception
* **DDIL** : Degraded communication scenarios
* **Adversarial** : Enemy jamming and counter-drone tactics

### 11.4 Performance Testing

* Inference latency under load
* Simulation frame rate with N drones
* Dashboard responsiveness

---

## 12. Documentation Deliverables

* API documentation (Sphinx/Doxygen)
* User guide for strategic planning dashboard
* Prompt engineering best practices
* Scenario creation tutorials
* System architecture diagrams
* Model performance benchmarks

---

## 13. Future Enhancements

* Hardware-in-the-loop (HIL) testing with real flight controllers
* Integration with actual military simulation standards (DIS, HLA)
* Multi-spectral sensor simulation (IR, SAR)
* Electronic warfare simulation (jamming, spoofing)
* Reinforcement learning for emergent tactics
* Edge deployment optimization (Jetson, embedded SoCs)

---

## 14. Success Metrics

* **Autonomy** : Drones complete missions with <10% ground control input
* **Performance** : AI decisions at >5Hz in simulation
* **Reliability** : <5% catastrophic failures (crashes, mission aborts)
* **Scalability** : Support 10+ drones in swarm simulation
* **Usability** : Non-technical operators can create scenarios in <30 min

---

## Appendices

### A. Glossary

* **SITL** : Software-in-the-Loop
* **DDIL** : Denied, Degraded, Intermittent, Limited (communications)
* **SLM** : Small Language Model
* **VLM** : Vision-Language Model
* **MAVLink** : Micro Air Vehicle Communication Protocol

### B. References

* PX4 Autopilot Documentation
* Pegasus Simulator GitHub
* Isaac Sim Documentation
* ROS2 Documentation

### C. Contact & Resources

* Project Repository: [TBD]
* Discussion Forum: [TBD]
* Issue Tracker: [TBD]

---

**Document Version:** 1.0

**Status:** Draft for Review

**Next Review:** [Date]
