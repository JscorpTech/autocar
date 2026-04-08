# Copilot Instructions for Autonomous Car Project

This repository implements an autonomous robot vehicle using **Raspberry Pi 4/5** (Python) + **ESP32-S3** (Arduino/C++). The system uses Ackermann steering geometry, A* pathfinding, PID control, and real-time obstacle avoidance.

## Running the System

### Install Dependencies

**Raspberry Pi:**
```bash
pip3 install pyserial flask flask-socketio
```

**ESP32:**
1. Install Arduino IDE or PlatformIO
2. Add ESP32 board package (`esp32 by Espressif Systems`)
3. Flash `esp32/car_controller/car_controller.ino`
4. Required libraries: `Adafruit HMC5883 Unified`, `Adafruit Unified Sensor`

### Test Commands

**Simulation mode (no hardware):**
```bash
cd raspberry_pi
python3 main.py --simulate ../maps/example_map.json
```

**With ESP32 hardware:**
```bash
cd raspberry_pi
python3 main.py ../maps/example_map.json --port /dev/ttyUSB0
```

**Web dashboard:**
```bash
cd raspberry_pi
python3 web_ui.py --simulate          # simulation
python3 web_ui.py --port /dev/ttyUSB0 # with hardware
python3 web_ui.py --simulate --web-port 8080  # custom port
```
Access at `http://localhost:5000` (default).

## Architecture

### Two-tier System
- **ESP32-S3**: Real-time hardware control (motors, sensors, encoders). Communicates via UART at 115200 baud.
- **Raspberry Pi**: High-level planning (A* pathfinding, PID navigation, web dashboard).

### Raspberry Pi Modules (`raspberry_pi/`)

| File | Role |
|------|------|
| `config.py` | All tunable parameters (PID gains, speeds, tolerances, physical dimensions) |
| `communicator.py` | UART serial layer; background thread parses `DATA:` / `WARN:` telemetry from ESP32 |
| `map_manager.py` | JSON map loading + A* pathfinding; outputs waypoints as (heading, distance) pairs |
| `navigator.py` | Two PID controllers (heading turn + straight driving); executes waypoints with obstacle checks |
| `main.py` | CLI entrypoint; `SimulatedCommunicator` class for hardware-free testing |
| `web_ui.py` | Flask + SocketIO web dashboard; manual control + real-time telemetry streaming |
| `logger.py` | Centralized logging setup (console INFO+, file DEBUG+ to `logs/comm.log`, rotating 1MB×5) |

### Serial Communication Protocol (Pi ↔ ESP32)

**Pi → ESP32 (commands):**
- `CMD:speed,steer_angle\n` — drive (speed: -255..255, angle: -30..+30°)
- `STOP\n` — emergency stop
- `PING\n` — connection check (expects `PONG`)
- `RESET_ENC\n` — reset encoder counters (but NOT heading)

**ESP32 → Pi (telemetry at 10 Hz):**
- `DATA:encL,encR,heading,rpmL,rpmR,distFront,distFrontRight,distFrontLeft,distRight,distLeft,distRear\n`
- `WARN:OBSTACLE\n` — front obstacle ≤ 0.5m
- `WARN:OBSTACLE_REAR\n` — rear obstacle ≤ 0.5m
- `WARN:TIMEOUT\n` — no command received for 2 seconds

### Navigation Flow

1. `MapManager.load_from_file()` reads JSON grid map
2. `MapManager.find_path()` runs A* with Manhattan heuristic
3. `MapManager.generate_waypoints()` groups consecutive steps into (heading°, distance_m) pairs
4. `Navigator.navigate_waypoints()` executes: turn to heading → drive distance, with PID corrections and obstacle stops

### Map Format (`maps/*.json`)

```json
{
  "name": "Example Map",
  "cell_size": 1.0,
  "start": [0, 0],
  "end": [5, 5],
  "map": [
    [1, 1, 0, 0],
    [1, 1, 1, 0],
    [1, 0, 1, 1]
  ]
}
```
- `1` = passable
- `0` = obstacle
- `start`/`end` = `[row, col]`

## Key Configuration (`config.py`)

### Physical Dimensions
- `WHEEL_DIAMETER = 0.35` — diameter 35cm → circumference ≈ 1.0996m
- `PULSES_PER_REV = 18` — 18 encoder signals per full wheel revolution → 0.0611m (61mm) per pulse
- `WHEEL_BASE = 0.95` — front-rear axle distance 95cm
- `CAR_WIDTH = 0.75` — track width (wheel-to-wheel) 75cm
- `MAX_STEER_ANGLE = 20.0°` — real measured steering limit
- `MIN_TURN_RADIUS` ≈ 2.61m — use `cell_size ≥ 3.0m` in maps for safe turning

### Control Parameters
- `BASE_SPEED = 150`, `TURN_SPEED = 100` — PWM values (0–255 range)
- `OBSTACLE_DISTANCE = 0.5` — emergency stop threshold in meters
- `HEADING_TOLERANCE = 5.0°`, `TURN_TIMEOUT_SEC = 30.0`
- PID gains: `PID_KP/KI/KD` (straight driving), `TURN_PID_KP/KI/KD` (turning)
- `SERIAL_BAUD = 115200`, `COMMAND_RATE_HZ = 10`

## Code Style

### Python (PEP-8)
- Max line length: **88 characters**
- `snake_case` for variables/functions, `UPPERCASE` for constants
- Type hints required for function signatures
- Import order: standard library → third-party → local modules
- Catch specific exceptions (avoid bare `except:`)

### C++/Arduino
- `camelCase` for methods, `PascalCase` for classes
- Use `if` checks for hardware connections
- Consistent indentation and brace alignment

### Logging
Use consistent prefixes:
- `[COMM]` — communication/serial
- `[MAP]` — map loading/pathfinding
- `[NAV]` — navigation/waypoint execution
- `[WEB]` — web UI events

Call `logger.setup_logging()` once at startup in `main.py` / `web_ui.py`.

## Critical Fixes Applied (2026-03)

Historical bugs that should NOT be reintroduced:

| # | Bug | Fix |
|---|-----|-----|
| 1 | `RESET_ENC` reset `heading=0` → car steered in circles | ESP32: heading no longer reset on `RESET_ENC` |
| 2 | `WHEEL_CIRC=0.785` (wrong, was for 25cm diameter) | ESP32: fixed to `0.8796` (π×0.28m) |
| 3 | `DEBOUNCE_MS=10` → pulses lost at speed | ESP32: reduced to `2ms` |
| 4 | Odometry updated every 200ms | ESP32: reduced to `50ms` |
| 5 | Only front sensor checked for obstacle | ESP32: all 3 front sensors checked |
| 6 | `inputBuf` unbounded → RAM overflow | ESP32: `MAX_CMD_LEN=64` added |
| 7 | Stale encoder values after reset | Pi: waits for enc=0 confirmation |
| 8 | PING response missed (DATA lines in buffer) | Pi: loops until PONG found |
| 9 | File upload path traversal vulnerability | Pi: `secure_filename()` added |
| 10 | HMC5883L compass accidentally removed | ESP32: compass re-added as primary heading source (falls back to odometry if not wired) |

## Hardware Components

| Component | Quantity | Purpose |
|-----------|----------|---------|
| Raspberry Pi 3/4/5 | 1 | Main computer — pathfinding, navigation, Web UI |
| ESP32-S3 | 1 | Motor control, sensor reading, odometry |
| BTS7960 Motor Driver | 3 | Drive motors (front/rear) + steering motor |
| LM393 IR Encoder | 4 | Wheel rotation tracking (odometry) |
| HC-SR04 Ultrasonic | 6 | Obstacle detection (front×3, sides×2, rear×1) |
| HMC5883L Compass | 1 | Absolute heading (I2C: SDA=GPIO8, SCL=GPIO9) |

### Compass (HMC5883L)
- If detected at boot: absolute heading used (no drift, Tashkent declination +5°)
- If not detected: odometry differential heading used as fallback
- Arduino libraries required: `Adafruit HMC5883 Unified`, `Adafruit Unified Sensor`

## Safety Features

- **6 ultrasonic sensors**: Front (center, right, left), sides (right, left), rear
- **Emergency stop threshold**: 0.5m — all front sensors checked while moving forward, rear sensor while reversing
- **3-point turn**: If turn impossible in tight space, reverse and retry (max 5 attempts)
- **Command timeout**: Motors auto-stop if no command received for 2 seconds
- **Manual override**: Ctrl+C in CLI, Spacebar in Web UI, physical emergency stop button

## ESP32 Build Configuration

Uses PlatformIO with:
- Platform: `espressif32`
- Board: `esp32-s3-devkitc-1`
- UART: GPIO 43 (TX) / GPIO 44 (RX)
- LEDC API (Core v3): `ledcAttach(pin, freq, res)` + `ledcWrite(pin, duty)`

See `esp32/car_controller/platformio.ini` for full configuration.
