# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Autonomous robot vehicle using **Raspberry Pi 4/5** (Python) + **ESP32-S3** (Arduino/C++). Implements Ackermann steering geometry, A* pathfinding, PID control, and real-time obstacle avoidance.

## Running the System

**Install Python dependencies:**
```bash
pip3 install pyserial flask flask-socketio
```

**CLI mode (with hardware):**
```bash
cd raspberry_pi
python3 main.py ../maps/example_map.json --port /dev/ttyUSB0
```

**Simulation mode (no ESP32 needed):**
```bash
cd raspberry_pi
python3 main.py --simulate ../maps/example_map.json
```

**Web dashboard:**
```bash
cd raspberry_pi
python3 web_ui.py --simulate          # simulation
python3 web_ui.py --port /dev/ttyUSB0 # with hardware
python3 web_ui.py --simulate --web-port 8080  # custom port
```
Access at `http://localhost:5000` (default).

**ESP32 firmware:** Flash `esp32/car_controller/car_controller.ino` via Arduino IDE. Requires libraries: `Adafruit HMC5883 Unified`, `Adafruit Unified Sensor`.

## Architecture

### Two-tier system
- **ESP32-S3**: Real-time hardware control (motors, sensors, encoders). Communicates via UART at 115200 baud.
- **Raspberry Pi**: High-level planning (A* pathfinding, PID navigation, web dashboard).

### Raspberry Pi modules (`raspberry_pi/`)
| File | Role |
|------|------|
| `config.py` | All tunable parameters (PID gains, speeds, tolerances, pin assignments) |
| `communicator.py` | UART serial layer; background thread parses `DATA:` / `WARN:` telemetry from ESP32 |
| `map_manager.py` | JSON map loading + A* pathfinding; outputs waypoints as (heading, distance) pairs |
| `navigator.py` | Two PID controllers (heading turn + straight driving); executes waypoints with obstacle checks |
| `main.py` | CLI entrypoint; `SimulatedCommunicator` class for hardware-free testing |
| `web_ui.py` | Flask + SocketIO web dashboard; manual control + real-time telemetry streaming |

### Serial protocol (Pi ↔ ESP32)
**Pi → ESP32:**
- `CMD:speed,steer_angle\n` — drive (speed: -255..255, angle: -30..+30°)
- `STOP\n`, `PING\n`, `RESET_ENC\n`

**ESP32 → Pi (10 Hz):**
- `DATA:encL,encR,heading,rpmL,rpmR,distFront,distFrontRight,distFrontLeft,distRight,distLeft,distRear\n`
- `WARN:OBSTACLE\n`, `WARN:OBSTACLE_REAR\n`, `WARN:TIMEOUT\n`

### Navigation flow
1. `MapManager.load_from_file()` reads JSON grid map
2. `MapManager.find_path()` runs A* (Manhattan heuristic)
3. `MapManager.generate_waypoints()` groups consecutive steps into (heading°, distance_m) pairs
4. `Navigator.navigate_waypoints()` executes: turn to heading → drive distance, with PID corrections and obstacle stops

### Map format (`maps/*.json`)
```json
{ "name": "...", "cell_size": 1.0, "start": [row, col], "end": [row, col],
  "map": [[1,0,1,...], ...] }
```
`1` = passable, `0` = obstacle.

## Key Configuration (`config.py`)
- `WHEEL_DIAMETER = 0.28` — radius 14cm = diameter 28cm → circumference ≈ 0.8796m
- `OBSTACLE_DISTANCE = 0.5` — emergency stop threshold in meters
- `BASE_SPEED = 150`, `TURN_SPEED = 100` — PWM values (0–255)
- `HEADING_TOLERANCE = 5.0°`, `TURN_TIMEOUT_SEC = 30.0`
- PID gains: `PID_KP/KI/KD` (straight), `TURN_PID_KP/KI/KD` (turning)
- `SERIAL_BAUD = 115200`, `COMMAND_RATE_HZ = 10`

## Critical Fixes Applied (2026-03)
| # | Bug | Fix |
|---|-----|-----|
| 1 | `RESET_ENC` reset `heading=0` → car steered in circles | ESP32: heading no longer reset on `RESET_ENC` |
| 2 | `WHEEL_CIRC=0.785` (wrong, was for 25cm dia) | ESP32: fixed to `0.8796` (π×0.28m) |
| 3 | `DEBOUNCE_MS=10` → pulses lost at speed | ESP32: reduced to `2ms` |
| 4 | Odometry updated every 200ms | ESP32: reduced to `50ms` |
| 5 | Only front sensor checked for obstacle | ESP32: all 3 front sensors checked |
| 6 | `inputBuf` unbounded → RAM overflow | ESP32: `MAX_CMD_LEN=64` added |
| 7 | Stale encoder values after reset | Pi: waits for enc=0 confirmation |
| 8 | PING response missed (DATA lines in buffer) | Pi: loops until PONG found |
| 9 | File upload path traversal | Pi: `secure_filename()` added |
| 10 | HMC5883L compass removed in Copilot PR | ESP32: compass re-added as primary heading source (falls back to odometry if not wired) |

## Compass (HMC5883L)
- Wired to I2C: `SDA=GPIO8`, `SCL=GPIO9` (same as original code)
- If detected at boot: absolute heading used (no drift, Toshkent declination +5°)
- If not detected: odometry differential heading used as fallback
- Arduino libraries required: `Adafruit HMC5883 Unified`, `Adafruit Unified Sensor`
