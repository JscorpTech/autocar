#!/usr/bin/env python3

import sys
import signal
import argparse
import time

from config import SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT
from communicator import Communicator
from map_manager import MapManager
from navigator import Navigator


class AutonomousCar:
    def __init__(self, serial_port, baud, simulate=False):
        self.simulate = simulate
        self.comm = None
        self.navigator = None

        if simulate:
            print("[SIM] simulyatsiya rejimi, ESP32 siz ishlaymiz")
            self.comm = SimulatedCommunicator()
        else:
            self.comm = Communicator(serial_port, baud, SERIAL_TIMEOUT)

        self.map_mgr = MapManager()
        self.navigator = Navigator(self.comm)

        signal.signal(signal.SIGINT, self._on_exit)

    def run(self, map_file):
        print("=" * 50)
        print("  AVTONOM MASHINA")
        print("=" * 50)

        if not self.simulate:
            print("\nESP32 ga ulanish...")
            if not self.comm.connect():
                print("ESP32 ga ulanib bo'lmadi")
                return False
        else:
            print("\nsimulatsiya rejim faol")

        print(f"\nxarita: {map_file}")
        if not self.map_mgr.load_from_file(map_file):
            print("xarita yuklanmadi!")
            self._cleanup()
            return False

        self.map_mgr.print_map()

        print("yo'l qidirilmoqda...")
        path = self.map_mgr.find_path()
        if not path:
            print("yo'l topilmadi!")
            self._cleanup()
            return False

        self.map_mgr.print_map()

        waypoints = self.map_mgr.generate_waypoints()
        if not waypoints:
            print("waypoint yaratilmadi!")
            self._cleanup()
            return False

        print("\n" + "=" * 50)
        input("tayyor bo'lsa Enter bosing >>> ")

        print("\nnavigatsiya boshlandi!")
        success = self.navigator.navigate_waypoints(waypoints)

        if success:
            print("\nmanzilga yetdik!")
        else:
            print("\nnavigatsiya muvaffaqiyatsiz")

        self._cleanup()
        return success

    def _on_exit(self, sig, frame):
        print("\nCtrl+C, to'xtayapman...")
        if self.navigator:
            self.navigator.emergency_stop()
        self._cleanup()
        sys.exit(0)

    def _cleanup(self):
        if self.comm and not self.simulate:
            self.comm.disconnect()


class SimulatedCommunicator:
    """ESP32 yo'q bo'lganda test qilish uchun"""

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
        self._speed = speed
        self._steer_angle = max(-30, min(30, steer_angle))
        self._tick()

    def send_stop(self):
        self._speed = 0
        self._steer_angle = 0

    def reset_encoders(self):
        self._enc_left = 0
        self._enc_right = 0

    def get_telemetry(self):
        from communicator import TelemetryData
        self._tick()
        return TelemetryData(
            encoder_left=self._enc_left,
            encoder_right=self._enc_right,
            heading=self._heading,
            rpm_left=abs(self._speed) * 0.5,
            rpm_right=abs(self._speed) * 0.5,
            distance=999.0,
            timestamp=time.time(),
            obstacle_warning=False,
        )

    def get_warnings(self):
        return []

    def send_raw(self, data):
        pass

    def _tick(self):
        import math
        now = time.time()
        dt = now - self._last_update
        self._last_update = now

        if dt > 0 and self._speed != 0:
            pulse_rate = 10
            pulses = int(abs(self._speed) * dt * pulse_rate / 255)
            self._enc_left += pulses
            self._enc_right += pulses

            # ackermann burilish simulyatsiyasi
            if abs(self._steer_angle) > 1:
                wheelbase = 1.8
                rad = math.radians(self._steer_angle)
                turn_radius = wheelbase / math.tan(rad)
                speed_ms = self._speed / 255.0
                omega = speed_ms / turn_radius
                self._heading = (self._heading + math.degrees(omega * dt)) % 360


def main():
    parser = argparse.ArgumentParser(description="Avtonom Mashina Robot")
    parser.add_argument("map_file", help="xarita JSON fayli")
    parser.add_argument("--port", default=SERIAL_PORT, help="serial port")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--simulate", action="store_true",
                        help="ESP32 siz sinov rejimi")
    args = parser.parse_args()

    car = AutonomousCar(args.port, args.baud, args.simulate)
    ok = car.run(args.map_file)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
