import serial
import threading
import time
from dataclasses import dataclass

from logger import get_logger

_log = get_logger("comm")


@dataclass
class TelemetryData:
    encoder_left: int = 0
    encoder_right: int = 0
    heading: float = 0.0
    rpm_left: float = 0.0
    rpm_right: float = 0.0
    distance: float = 999.0        # front sensor
    dist_front_right: float = 999.0  # front-right sensor
    dist_front_left: float = 999.0   # front-left sensor
    dist_right: float = 999.0        # right-side sensor
    dist_left: float = 999.0         # left-side sensor
    dist_rear: float = 999.0         # rear sensor
    timestamp: float = 0.0
    obstacle_warning: bool = False


class Communicator:
    def __init__(self, port, baud, timeout=0.1):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.serial_conn = None
        self.telemetry = TelemetryData()
        self._lock = threading.Lock()
        self._running = False
        self._reader_thread = None
        self._warnings = []
        self._rx_count = 0     # DATA packets received
        self._tx_count = 0     # commands sent
        self._enc_reset_event = threading.Event()

    def connect(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout,
                write_timeout=1.0
            )
            _log.info("Serial port opened: %s @ %d baud", self.port, self.baud)
            time.sleep(2)
            self.serial_conn.reset_input_buffer()

            self.send_raw("PING\n")

            # Wait up to 2 seconds for PONG (skip any DATA: lines)
            deadline = time.time() + 2.0
            got_pong = False
            while time.time() < deadline:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line == "PONG":
                        got_pong = True
                        break
                    # Skip DATA: lines that may arrive before PONG
                time.sleep(0.05)

            if got_pong:
                _log.info("ESP32 connected: %s", self.port)
                print(f"[COMM] ESP32 connected: {self.port}")
            else:
                _log.warning("No PONG received, continuing anyway")
                print(f"[COMM] No PONG received, continuing")

            self._start_reader()
            return True
        except serial.SerialException as e:
            _log.error("Connection error: %s", e)
            print(f"[COMM] connection error: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.send_raw("STOP\n")
            except Exception:
                pass
            self.serial_conn.close()
        _log.info("Disconnected from ESP32. RX packets: %d, TX commands: %d",
                  self._rx_count, self._tx_count)

    def send_drive(self, speed, steer_angle):
        """speed: -255..255, steer_angle: -20..+20 degrees"""
        speed = max(-255, min(255, int(speed)))
        steer_angle = max(-20, min(20, int(steer_angle)))
        self.send_raw(f"CMD:{speed},{steer_angle}\n")

    def send_stop(self):
        self.send_raw("STOP\n")

    def reset_encoders(self):
        """Send encoder reset command and clear the ACK event."""
        self._enc_reset_event.clear()
        self.send_raw("RESET_ENC\n")

    def wait_encoder_reset(self, timeout=1.0):
        """Wait for ACK:RESET_ENC from ESP32. Returns True on success."""
        return self._enc_reset_event.wait(timeout)

    def send_raw(self, data):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(data.encode('utf-8'))
                self._tx_count += 1
                _log.debug("TX -> %s", data.strip())
            except serial.SerialException as e:
                _log.error("Write error: %s", e)
                print(f"[COMM] write error: {e}")

    def get_telemetry(self):
        with self._lock:
            return TelemetryData(
                encoder_left=self.telemetry.encoder_left,
                encoder_right=self.telemetry.encoder_right,
                heading=self.telemetry.heading,
                rpm_left=self.telemetry.rpm_left,
                rpm_right=self.telemetry.rpm_right,
                distance=self.telemetry.distance,
                dist_front_right=self.telemetry.dist_front_right,
                dist_front_left=self.telemetry.dist_front_left,
                dist_right=self.telemetry.dist_right,
                dist_left=self.telemetry.dist_left,
                dist_rear=self.telemetry.dist_rear,
                timestamp=self.telemetry.timestamp,
                obstacle_warning=self.telemetry.obstacle_warning,
            )

    def get_warnings(self):
        with self._lock:
            w = self._warnings.copy()
            self._warnings.clear()
            return w

    def _start_reader(self):
        self._running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def _read_loop(self):
        while self._running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        _log.debug("RX <- %s", line)
                        self._parse_line(line)
                else:
                    time.sleep(0.01)
            except serial.SerialException:
                time.sleep(0.1)
            except Exception as e:
                _log.error("Read error: %s", e)
                print(f"[COMM] read error: {e}")
                time.sleep(0.1)

    def _parse_line(self, line):
        if line.startswith("DATA:"):
            try:
                parts = line[5:].split(",")
                if len(parts) >= 6:
                    with self._lock:
                        self.telemetry.encoder_left  = int(parts[0])
                        self.telemetry.encoder_right = int(parts[1])
                        self.telemetry.heading       = float(parts[2])
                        self.telemetry.rpm_left      = float(parts[3])
                        self.telemetry.rpm_right     = float(parts[4])
                        self.telemetry.distance         = float(parts[5])
                        self.telemetry.dist_front_right = float(parts[6])  if len(parts) > 6  else 999.0
                        self.telemetry.dist_front_left  = float(parts[7])  if len(parts) > 7  else 999.0
                        self.telemetry.dist_right       = float(parts[8])  if len(parts) > 8  else 999.0
                        self.telemetry.dist_left        = float(parts[9])  if len(parts) > 9  else 999.0
                        self.telemetry.dist_rear        = float(parts[10]) if len(parts) > 10 else 999.0
                        self.telemetry.timestamp     = time.time()
                        self.telemetry.obstacle_warning = False
                    self._rx_count += 1
                else:
                    print(f"[COMM] short DATA ({len(parts)} parts): '{line}'")
                    _log.warning("Short DATA packet: %s", line)
            except (ValueError, IndexError) as e:
                _log.warning("Malformed DATA packet: %s", line)
                print(f"[COMM] parse error: {e} | '{line}'")

        elif line.startswith("WARN:"):
            warning = line[5:]
            with self._lock:
                self._warnings.append(warning)
                if warning in ("OBSTACLE", "OBSTACLE_REAR"):
                    self.telemetry.obstacle_warning = True
            _log.warning("ESP32 warning: %s", warning)
            print(f"[COMM] warning: {warning}")

        elif line.startswith("ACK:"):
            ack = line[4:]
            if ack == "RESET_ENC":
                self._enc_reset_event.set()
            _log.info("ESP32 ACK: %s", ack)
            print(f"[COMM] ACK: {ack}")
