#!/usr/bin/env python3

import sys
import signal
import argparse
import time
import math
import threading

from logger import setup_logging
from config import (
    SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT,
    WHEEL_CIRCUMFERENCE, PULSES_PER_REV, WHEEL_BASE,
    MAX_SPEED, BASE_SPEED, MAX_STEER_ANGLE, COMMAND_RATE_HZ,
)
from communicator import Communicator
from map_manager import MapManager
from navigator import Navigator


class AutonomousCar:
    def __init__(self, serial_port, baud, simulate=False):
        self.simulate = simulate
        self.comm = None
        self.navigator = None

        if simulate:
            print("[SIM] simulation mode, running without ESP32")
            self.comm = SimulatedCommunicator()
        else:
            self.comm = Communicator(serial_port, baud, SERIAL_TIMEOUT)

        self.map_mgr = MapManager()
        self.navigator = Navigator(self.comm)

        signal.signal(signal.SIGINT, self._on_exit)

    def run(self, map_file):
        print("=" * 50)
        print("  AUTONOMOUS CAR")
        print("=" * 50)

        if not self.simulate:
            print("\nConnecting to ESP32...")
            if not self.comm.connect():
                print("Failed to connect to ESP32")
                return False
        else:
            print("\nsimulation mode active")

        print(f"\nmap: {map_file}")
        if not self.map_mgr.load_from_file(map_file):
            print("map failed to load!")
            self._cleanup()
            return False

        self.map_mgr.print_map()

        print("searching for path...")
        path = self.map_mgr.find_path()
        if not path:
            print("no path found!")
            self._cleanup()
            return False

        self.map_mgr.print_map()

        waypoints = self.map_mgr.generate_waypoints()
        if not waypoints:
            print("no waypoints generated!")
            self._cleanup()
            return False

        print("\n" + "=" * 50)
        input("press Enter when ready >>> ")

        print("\nnavigation started!")
        success = self.navigator.navigate_waypoints(waypoints)

        if success:
            print("\ndestination reached!")
        else:
            print("\nnavigation failed")

        self._cleanup()
        return success

    def _on_exit(self, sig, frame):
        print("\nCtrl+C, stopping...")
        if self.navigator:
            self.navigator.emergency_stop()
        self._cleanup()
        sys.exit(0)

    def _cleanup(self):
        if self.comm and not self.simulate:
            self.comm.disconnect()


class SimulatedCommunicator:
    """Used for testing when ESP32 is not available.
    Models realistic Ackermann physics:
    - WHEEL_CIRCUMFERENCE and PULSES_PER_REV from config
    - Outer/inner wheel arc difference calculated
    - Encoder reset ACK returned immediately
    """

    def __init__(self):
        self._heading = 0.0
        self._enc_left = 0
        self._enc_right = 0
        self._speed = 0
        self._steer_angle = 0
        self._last_update = time.time()

    def connect(self):
        return True

    def disconnect(self):
        pass

    def send_drive(self, speed, steer_angle):
        self._tick()
        self._speed = max(-255, min(255, int(speed)))
        self._steer_angle = max(-30, min(30, int(steer_angle)))

    def send_stop(self):
        self._tick()
        self._speed = 0
        self._steer_angle = 0

    def reset_encoders(self):
        self._enc_left = 0
        self._enc_right = 0

    def wait_encoder_reset(self, timeout=1.0):
        """Encoders reset immediately in simulation"""
        return True

    def get_telemetry(self):
        from communicator import TelemetryData
        self._tick()
        # RPM estimate: MAX_SPEED PWM ~= 200 RPM
        rpm = abs(self._speed) * 200.0 / MAX_SPEED
        return TelemetryData(
            encoder_left=self._enc_left,
            encoder_right=self._enc_right,
            heading=self._heading,
            rpm_left=rpm,
            rpm_right=rpm,
            distance=999.0,
            dist_front_right=999.0,
            dist_front_left=999.0,
            dist_right=999.0,
            dist_left=999.0,
            dist_rear=999.0,
            timestamp=time.time(),
            obstacle_warning=False,
        )

    def get_warnings(self):
        return []

    def send_raw(self, data):
        pass

    def _tick(self):
        now = time.time()
        dt = now - self._last_update
        self._last_update = now

        if dt <= 0 or self._speed == 0:
            return

        # Speed estimate: MAX_SPEED PWM ~= 1.0 m/s
        speed_ms = (self._speed / MAX_SPEED) * 1.0  # m/s (signed)

        m_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REV
        arc = speed_ms * dt  # distance traveled, metres (signed)

        if abs(self._steer_angle) > 0.5:
            # Ackermann turn: outer/inner wheel arc difference
            rad = math.radians(abs(self._steer_angle))
            turn_radius = WHEEL_BASE / math.tan(rad)  # mid-axle turn radius

            # Heading change (rad -> degrees)
            dTheta = math.degrees(arc / turn_radius)
            if self._steer_angle < 0:
                dTheta = -dTheta  # left turn = negative

            # Outer and inner arcs (half track width = CAR_WIDTH/2 = 0.375m)
            half_width = 0.375  # metres (CAR_WIDTH=0.75m)
            outer_arc = abs(arc) * (turn_radius + half_width) / turn_radius
            inner_arc = abs(arc) * max(0.0, turn_radius - half_width) / turn_radius

            outer_pulses = int(outer_arc / m_per_pulse)
            inner_pulses = int(inner_arc / m_per_pulse)

            if self._steer_angle > 0:  # right turn: right=inner, left=outer
                left_pulses = outer_pulses
                right_pulses = inner_pulses
            else:  # left turn: left=inner, right=outer
                left_pulses = inner_pulses
                right_pulses = outer_pulses

            self._enc_left += left_pulses
            self._enc_right += right_pulses
            self._heading = (self._heading + dTheta) % 360
        else:
            # Straight driving
            pulses = int(abs(arc) / m_per_pulse)
            self._enc_left += pulses
            self._enc_right += pulses


class TerminalManualController:
    """Terminal-based manual drive mode for testing."""
    SPEED_STEP = 50
    STEER_STEP = 5

    def __init__(self, comm):
        self.comm = comm
        self.speed = 0
        self.steer = 0
        self.running = False
        self._state_lock = threading.Lock()
        self._tx_thread = None

    def run(self):
        print("=" * 50)
        print("  TERMINAL MANUAL CONTROL")
        print("=" * 50)
        print("[MANUAL] type 'help' for commands")

        self.running = True
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()
        self._print_help()
        self._print_status(show_telemetry=True)

        try:
            while True:
                cmd = input("manual> ").strip().lower()
                if not cmd:
                    continue

                if cmd in ("q", "quit", "exit"):
                    break
                if cmd in ("h", "help", "?"):
                    self._print_help()
                    continue

                if cmd in ("w", "forward"):
                    self._change_speed(self.SPEED_STEP)
                elif cmd in ("s", "reverse", "back"):
                    self._change_speed(-self.SPEED_STEP)
                elif cmd in ("f",):
                    self._set_speed(BASE_SPEED)
                elif cmd in ("b",):
                    self._set_speed(-BASE_SPEED)
                elif cmd in ("x", "stop", "space"):
                    self._set_speed(0)
                elif cmd in ("a", "left"):
                    self._change_steer(-self.STEER_STEP)
                elif cmd in ("d", "right"):
                    self._change_steer(self.STEER_STEP)
                elif cmd in ("c", "center"):
                    self._set_steer(0)
                elif cmd in ("r", "reset"):
                    self._reset_encoders()
                elif cmd in ("status", "st"):
                    self._print_status(show_telemetry=True)
                    continue
                elif cmd.startswith("speed "):
                    self._set_speed_from_cmd(cmd)
                elif cmd.startswith("steer "):
                    self._set_steer_from_cmd(cmd)
                else:
                    print("[MANUAL] unknown command, type 'help'")
                    continue

                self._print_status(show_telemetry=False)

        except (EOFError, KeyboardInterrupt):
            print("\n[MANUAL] exit requested")
        finally:
            self.running = False
            if self._tx_thread:
                self._tx_thread.join(timeout=1.0)
            self.comm.send_stop()

        return True

    def _tx_loop(self):
        interval = 1.0 / COMMAND_RATE_HZ
        while self.running:
            with self._state_lock:
                speed = int(self.speed)
                steer = int(self.steer)
            self.comm.send_drive(speed, steer)
            time.sleep(interval)

    def _set_speed(self, speed):
        with self._state_lock:
            self.speed = max(-MAX_SPEED, min(MAX_SPEED, int(speed)))

    def _change_speed(self, delta):
        with self._state_lock:
            self.speed = max(-MAX_SPEED, min(MAX_SPEED, int(self.speed + delta)))

    def _set_steer(self, steer):
        max_steer = int(MAX_STEER_ANGLE)
        with self._state_lock:
            self.steer = max(-max_steer, min(max_steer, int(steer)))

    def _change_steer(self, delta):
        max_steer = int(MAX_STEER_ANGLE)
        with self._state_lock:
            self.steer = max(-max_steer, min(max_steer, int(self.steer + delta)))

    def _set_speed_from_cmd(self, cmd):
        parts = cmd.split(maxsplit=1)
        if len(parts) != 2:
            print("[MANUAL] usage: speed <value>")
            return
        try:
            self._set_speed(int(parts[1]))
        except ValueError:
            print("[MANUAL] speed must be integer")

    def _set_steer_from_cmd(self, cmd):
        parts = cmd.split(maxsplit=1)
        if len(parts) != 2:
            print("[MANUAL] usage: steer <value>")
            return
        try:
            self._set_steer(int(parts[1]))
        except ValueError:
            print("[MANUAL] steer must be integer")

    def _reset_encoders(self):
        self.comm.reset_encoders()
        if self.comm.wait_encoder_reset(timeout=1.5):
            print("[MANUAL] encoder reset ACK")
        else:
            print("[MANUAL] encoder reset ACK not received")

    def _print_status(self, show_telemetry=False):
        with self._state_lock:
            speed = int(self.speed)
            steer = int(self.steer)
        print(f"[MANUAL] speed={speed}  steer={steer}")

        for warning in self.comm.get_warnings():
            print(f"[COMM] warning: {warning}")

        if show_telemetry:
            telem = self.comm.get_telemetry()
            front_min = min(telem.distance, telem.dist_front_right, telem.dist_front_left)
            print(f"[MANUAL] heading={telem.heading:.1f}deg  "
                  f"encL={telem.encoder_left}  encR={telem.encoder_right}")
            print(f"[MANUAL] rpmL={telem.rpm_left:.1f}  rpmR={telem.rpm_right:.1f}  "
                  f"front={front_min:.2f}m  rear={telem.dist_rear:.2f}m")

    def _print_help(self):
        print("\n[MANUAL] commands:")
        print("  w / s          speed + / speed -")
        print("  f / b          forward / backward (BASE_SPEED)")
        print("  x              stop")
        print("  a / d          steer left / right")
        print("  c              center steering")
        print("  speed <v>      set speed directly (-220..220)")
        print("  steer <v>      set steer directly (-20..20)")
        print("  r              reset encoders")
        print("  status         show telemetry")
        print("  q              quit\n")


def run_manual_mode(serial_port, baud, simulate=False):
    if simulate:
        print("[SIM] simulation mode, running without ESP32")
        comm = SimulatedCommunicator()
    else:
        print("\nConnecting to ESP32...")
        comm = Communicator(serial_port, baud, SERIAL_TIMEOUT)
        if not comm.connect():
            print("Failed to connect to ESP32")
            return False

    try:
        return TerminalManualController(comm).run()
    finally:
        if not simulate:
            comm.disconnect()


def main():
    setup_logging()
    parser = argparse.ArgumentParser(description="Autonomous Car Robot")
    parser.add_argument("map_file", nargs="?", help="map JSON file")
    parser.add_argument("--port", default=SERIAL_PORT, help="serial port")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--simulate", action="store_true",
                        help="test mode without ESP32")
    parser.add_argument("--manual", action="store_true",
                        help="terminal manual drive mode")
    args = parser.parse_args()

    if args.manual:
        ok = run_manual_mode(args.port, args.baud, args.simulate)
        sys.exit(0 if ok else 1)

    if not args.map_file:
        parser.error("map_file is required unless --manual is used")

    car = AutonomousCar(args.port, args.baud, args.simulate)
    ok = car.run(args.map_file)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
