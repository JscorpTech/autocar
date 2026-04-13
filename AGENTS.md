# AGENTS.md

This document provides essential guidelines for automation agents working in this repository. It includes build/test commands, code style standards, and repository-specific rules.

---

## Build, Lint, and Test Commands

### 🔧 Build Instructions
There is no explicit build process in this repository (Python + Arduino). Ensure all dependencies are installed.

#### Raspberry Pi Dependencies
```bash
pip3 install pyserial flask flask-socketio
```

#### ESP32 Setup
1. Install Arduino IDE or PlatformIO.
2. Add the ESP32 board package (`esp32 by Espressif Systems`).
3. Flash the firmware:
   `esp32/car_controller/car_controller.ino`

### ✅ Test Commands
Testing is primarily through manual simulation or hardware runs:

- **Run the system (simulation):**
  ```bash
  cd raspberry_pi
  python3 main.py --simulate ../maps/example_map.json
  ```

- **Run with ESP32 hardware:**
  ```bash
  cd raspberry_pi
  python3 main.py ../maps/example_map.json --port /dev/ttyUSB0
  ```

- **Web UI test:**
  ```bash
  python3 web_ui.py --simulate
  ```

### 🧪 Running a Single Test
Currently, the repository does not use formal test frameworks. Use simulation mode (`--simulate`) or specific real-world commands for unit tests. Example:
```bash
python3 main.py --simulate ../maps/example_map.json
```

---

## Code Style Guidelines

### 🆗 General Standards
Maintain clean and readable Python and C++ code. Follow these coding conventions:

#### 📥 Imports
- Standard library imports first, third-party libraries next, and local imports last.
- Example:
  ```python
  import os
  import flask
  from communicator import Communicator
  ```
#### 📰 Types
Use type hints for Python functions.
- Example:
  ```python
  def calculate_distance(speed: int, time: float) -> float:
      return speed * time
  ```

#### ✍️ Formatting
- Python: Use PEP-8. Max line length = **88 characters**.
- C++/Arduino: Follow standard formatting; align braces and use consistent indentation.

#### 🛠️ Error Handling
- Python: Catch specific exceptions, avoid generic `except:`.
- Example:
  ```python
  try:
      communicator.connect()
  except SerialException:
      print("Failed to connect")
  ```
- C++: Use `if` checks for hardware connections.

#### 🔤 Naming Conventions
- Python: `snake_case` for variables and functions, `UPPERCASE` for constants.
  - Example: `def navigate_waypoints()`
- C++: `camelCase` for methods, `PascalCase` for classes.
  - Example: `class CarController`.

#### 🚦 Control Flow
- Be explicit with Python `elif`/`else` and C++ `switch` cases.
- Example (Python):
  ```python
  if obstacle_detected:
      stop_motors()
  elif path_clear:
      continue_navigation()
  else:
      retry_pathfinding()
  ```
- Example (C++):
  ```cpp
  switch(sensorState) {
      case CLEAR:
          driveForward();
          break;
      default:
          stopMotors();
  }
  ```

---

## Repository-Specific Rules

1. **Serial Communication Protocol** (Raspberry Pi ↔ ESP32):
   - Commands: `CMD:speed,steer_angle\n`, `STOP\n`, etc.
   - ESP32 sends telemetry data back to Raspberry Pi every **100ms**.
   - Logs must use clear prefixes: `[COMM]`, `[MAP]`, `[NAV]`, etc.

2. **Configuration File (`config.py`)**:
   Key parameters:
   ```python
   SERIAL_BAUD = 115200
   WHEEL_DIAMETER = 0.35
   MAX_STEER_ANGLE = 20.0
   PID_KP, PID_KI, PID_KD = 1.0, 0.05, 0.25
   BASE_SPEED = 150
   ```

3. **Pathfinding (JSON Map)**:
   - `1` = drivable.
   - `0` = obstacle.
   Example:
   ```json
   {
     "start": [0, 0],
     "end": [5, 5],
     "map": [[1, 1, 0, 0], [1, 1, 1, 0]]
   }
   ```

4. **Emergency Stop**:
   - Critical safety feature.
   - Trigger via Web UI, hardware button, or `CTRL+C` in CLI.

5. **Logging**:
   - Use the following prefixes consistently:
     - `[COMM]` for communication logs.
     - `[MAP]` for map-related logs.
     - `[NAV]` for navigation.

---

Adhere to these guidelines to ensure consistent, reliable, and maintainable code. Document updates or deviations explicitly.