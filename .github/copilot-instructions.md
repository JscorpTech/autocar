# Copilot Instructions for Autonomous Car Project

This file refines the existing repository guidance with code-verified details
for this current codebase.

## Build, test, and lint commands

### Install dependencies (Raspberry Pi/Python)
```bash
pip3 install pyserial flask flask-socketio
```

### Build ESP32 firmware (same command used in CI)
```bash
cd esp32/car_controller
pio run
```

### Run / test flows
```bash
cd raspberry_pi

# Full navigation in simulation mode (no ESP32)
python3 main.py --simulate ../maps/example_map.json

# Hardware run (set serial port as needed)
python3 main.py ../maps/example_map.json --port /dev/ttyUSB0

# Web UI in simulation mode
python3 web_ui.py --simulate
```

### Run a single test scenario
```bash
cd raspberry_pi
python3 main.py --simulate ../maps/test_straight.json
```
Other focused map scenarios: `maps/test_L_turn.json`, `maps/test_U_turn.json`.

### Lint
No repository lint command is currently configured.

## High-level architecture

- **Two-tier runtime**
  - `raspberry_pi/`: high-level planning, navigation, and web control.
  - `esp32/car_controller/car_controller.ino`: low-level motor/sensor control
    and serial telemetry.

- **CLI control path (`raspberry_pi/main.py`)**
  1. Create `Communicator` (or `SimulatedCommunicator` with `--simulate`).
  2. Load a grid map with `MapManager`.
  3. Compute A* path (`find_path`) and convert to waypoints
     (`generate_waypoints`).
  4. Execute waypoints with `Navigator` (turn PID + straight-line PID).

- **Web control path (`raspberry_pi/web_ui.py`)**
  - Flask + Socket.IO server for map upload/load, pathfinding, manual control,
    and start/stop navigation.
  - Navigation runs in a background thread (`run_navigation`).
  - Telemetry is streamed from communicator to clients in `telemetry_loop`.
  - Uses the same `MapManager` and `Navigator` classes as CLI mode.

- **Pi ↔ ESP32 serial contract**
  - Pi commands: `CMD:speed,steer_angle`, `STOP`, `PING`, `RESET_ENC`.
  - ESP32 telemetry:
    `DATA:encL,encR,heading,rpmL,rpmR,dFront,dFrontRight,dFrontLeft,dRight,dLeft,dRear`.
  - Alerts/acks: `WARN:OBSTACLE`, `WARN:OBSTACLE_REAR`, `WARN:TIMEOUT`,
    `ACK:RESET_ENC`.

## Key conventions in this repository

- Keep telemetry format backward-compatible across layers.
  - `car_controller.ino` sends `DATA` fields in fixed order.
  - `raspberry_pi/communicator.py` parses by fixed index.
  - `raspberry_pi/web_ui.py` maps parsed telemetry to Socket.IO payloads.
  Any field order or naming change must be updated in all three places.

- `RESET_ENC` must **not** reset heading.
  - ESP32 resets encoder counters and emits `ACK:RESET_ENC`.
  - Python navigation relies on heading continuity after encoder resets.

- `Communicator.connect()` handshake is intentional.
  - It sends `PING` and waits up to ~2 seconds for `PONG` while skipping
    interleaved `DATA:` lines that may already be in the serial buffer.

- Encoder reset workflow is part of distance driving.
  - `Navigator._drive_distance()` sends `RESET_ENC`, waits for ACK event, then
    checks that encoder counters have actually become zero before integrating
    traveled distance.

- Waypoint distance uses `config.CELL_SIZE`.
  - `MapManager.generate_waypoints()` computes meters from `CELL_SIZE`.
  - JSON map `cell_size` is currently not consumed by runtime logic.

- Safety checks are duplicated on purpose.
  - ESP32 hard-stops and emits warnings on front/rear obstacles and command
    timeout.
  - Navigator also checks front sensor triplet and aborts/retries in software.

- Map upload hardening in Web UI is required.
  - Keep `secure_filename()` usage.
  - Keep structural validation (`map`, `start`, `end`) and size limit
    (max 1000x1000) before accepting uploaded JSON maps.

- Logging pattern:
  - Call `setup_logging()` once at process startup (`main.py`, `web_ui.py`).
  - Use `get_logger(...)` for module logs.
  - Keep user-facing log prefixes consistent: `[COMM]`, `[MAP]`, `[NAV]`,
    `[WEB]`.
