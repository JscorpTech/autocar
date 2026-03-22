#!/usr/bin/env python3

import sys
import signal
import argparse
import time
import math

from logger import setup_logging
from config import (
    SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT,
    WHEEL_CIRCUMFERENCE, PULSES_PER_REV, WHEEL_BASE,
    MAX_SPEED,
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


def main():
    setup_logging()
    parser = argparse.ArgumentParser(description="Autonomous Car Robot")
    parser.add_argument("map_file", help="map JSON file")
    parser.add_argument("--port", default=SERIAL_PORT, help="serial port")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--simulate", action="store_true",
                        help="test mode without ESP32")
    args = parser.parse_args()

    car = AutonomousCar(args.port, args.baud, args.simulate)
    ok = car.run(args.map_file)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
