import time
import math
from communicator import TelemetryData
from config import (
    WHEEL_CIRCUMFERENCE, PULSES_PER_REV,
    BASE_SPEED, TURN_SPEED,
    MAX_STEER_ANGLE,
    PID_KP, PID_KI, PID_KD,
    TURN_PID_KP, TURN_PID_KI, TURN_PID_KD,
    HEADING_TOLERANCE, WAYPOINT_REACHED, OBSTACLE_DISTANCE,
    COMMAND_RATE_HZ, TURN_TIMEOUT_SEC,
    OBSTACLE_WAIT_RETRIES, OBSTACLE_WAIT_SLEEP,
)


class PIDController:
    def __init__(self, kp, ki, kd, out_min=-255, out_max=255):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = max(now - self.last_time, 0.01)
        self.last_time = now

        self.integral += error * dt
        self.integral = max(-100, min(100, self.integral))

        deriv = (error - self.prev_error) / dt
        self.prev_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        return max(self.out_min, min(self.out_max, out))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()


class Navigator:
    def __init__(self, comm):
        self.comm = comm
        # turning PID: output is steering angle (-30..+30)
        self.heading_pid = PIDController(
            TURN_PID_KP, TURN_PID_KI, TURN_PID_KD,
            -MAX_STEER_ANGLE, MAX_STEER_ANGLE
        )
        # straight-line PID: output is steering correction angle
        self.straight_pid = PIDController(
            PID_KP, PID_KI, PID_KD,
            -MAX_STEER_ANGLE, MAX_STEER_ANGLE
        )
        self.running = False
        self.current_waypoint_idx = 0

    def navigate_waypoints(self, waypoints):
        self.running = True
        self.current_waypoint_idx = 0
        total = len(waypoints)

        print(f"\n[NAV] {total} waypoints")

        for idx, wp in enumerate(waypoints):
            if not self.running:
                self.comm.send_stop()
                return False

            self.current_waypoint_idx = idx
            heading = wp["heading"]
            dist = wp["distance"]

            print(f"\n[NAV] waypoint {idx + 1}/{total}: {heading}deg, {dist:.2f}m")

            # turn first (Ackermann arc)
            if not self._turn_to_heading(heading):
                self.comm.send_stop()
                return False

            # then drive straight
            if not self._drive_distance(dist, heading):
                self.comm.send_stop()
                return False

            print(f"[NAV] waypoint {idx + 1} ok")

        self.comm.send_stop()
        self.running = False
        print("\n[NAV] destination reached!")
        return True

    def _turn_to_heading(self, target):
        """
        Ackermann turn: the car drives slowly forward with the wheels
        fully turned. If space is insufficient, it backs up and tries
        again (3-point turn, up to 5 attempts).
        """
        self.heading_pid.reset()
        t0 = time.time()
        reverse_mode = False
        attempts = 0
        dt = 1.0 / COMMAND_RATE_HZ
        last_log = time.time()

        telem0 = self.comm.get_telemetry()
        print(f"  [TURN] start: current={telem0.heading:.1f}deg, target={target}deg, "
              f"err={self._heading_error(telem0.heading, target):.1f}deg")

        while self.running:
            elapsed = time.time() - t0
            if elapsed > TURN_TIMEOUT_SEC:
                print(f"  [TURN] timeout ({TURN_TIMEOUT_SEC}s)")
                self.comm.send_stop()
                return False

            telem = self.comm.get_telemetry()
            err = self._heading_error(telem.heading, target)

            if abs(err) < HEADING_TOLERANCE:
                self.comm.send_drive(0, 0)
                time.sleep(0.15)
                print(f"  [TURN] OK: {telem.heading:.1f}deg (target={target}deg, "
                      f"err={err:.1f}deg, t={elapsed:.1f}s)")
                return True

            # front obstacle check (front, front-right, front-left sensors)
            front_dist = min(telem.distance, telem.dist_front_right, telem.dist_front_left)
            if not reverse_mode and 0 < front_dist < OBSTACLE_DISTANCE:
                attempts += 1
                if attempts > 5:
                    print("  [TURN] not enough space (5 attempts), aborting")
                    self.comm.send_stop()
                    return False
                print(f"  [TURN] obstacle {front_dist:.2f}m, reversing (attempt {attempts})")
                reverse_mode = True

            steer = self.heading_pid.compute(err)
            steer = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer))

            # log every 1 second
            if time.time() - last_log >= 1.0:
                print(f"  [TURN] {elapsed:.1f}s | "
                      f"heading={telem.heading:.1f}deg | "
                      f"err={err:.1f}deg | steer={steer:.0f}")
                last_log = time.time()

            if reverse_mode:
                # reverse with steering turned the opposite way
                print(f"  [TURN] reversing: steer={-steer:.0f}")
                self.comm.send_drive(-TURN_SPEED, int(-steer))
                time.sleep(0.8)
                self.comm.send_drive(0, 0)
                time.sleep(0.2)
                reverse_mode = False
            else:
                # drive forward slowly with wheels turned
                self.comm.send_drive(TURN_SPEED, int(steer))

            time.sleep(dt)

        return False

    def _drive_distance(self, distance, target_heading):
        """
        Drive straight: steering correction based on odometry heading.
        On an Ackermann car, direction is controlled via steering angle,
        not differential motor speed.
        """
        self.straight_pid.reset()
        m_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REV
        print(f"  [DRIVE] start: {distance:.2f}m, heading={target_heading}deg")
        print(f"  [DRIVE] m_per_pulse={m_per_pulse:.5f}m, "
              f"pulses_needed~={distance/m_per_pulse:.0f}")

        # Reset encoders and wait for ACK
        self.comm.reset_encoders()
        if not self.comm.wait_encoder_reset(timeout=1.5):
            print("  [DRIVE] WARNING: encoder reset ACK not received, continuing")

        # Extra check that encoder values are actually zero
        deadline = time.time() + 0.8
        while time.time() < deadline:
            telem = self.comm.get_telemetry()
            if telem.encoder_left == 0 and telem.encoder_right == 0:
                print("  [DRIVE] encoder reset confirmed: OK")
                break
            time.sleep(0.05)
        else:
            telem = self.comm.get_telemetry()
            print(f"  [DRIVE] WARNING: encoders not zero "
                  f"(L={telem.encoder_left}, R={telem.encoder_right})")

        t0 = time.time()
        timeout = max(distance * 5.0 + 10.0, 30.0)
        dt = 1.0 / COMMAND_RATE_HZ
        last_log = time.time()
        front_dist = 999.0

        while self.running:
            elapsed = time.time() - t0
            if elapsed > timeout:
                print(f"  [DRIVE] TIMEOUT ({timeout:.0f}s)")
                self.comm.send_stop()
                return False

            telem = self.comm.get_telemetry()

            # obstacle check: front, front-right, front-left sensors
            front_dist = min(telem.distance, telem.dist_front_right, telem.dist_front_left)
            if 0 < front_dist < OBSTACLE_DISTANCE:
                print(f"  [DRIVE] obstacle! {front_dist:.2f}m")
                self.comm.send_stop()
                time.sleep(1.0)
                cleared = False
                for retry in range(OBSTACLE_WAIT_RETRIES):
                    telem = self.comm.get_telemetry()
                    front_dist = min(telem.distance, telem.dist_front_right, telem.dist_front_left)
                    if front_dist >= OBSTACLE_DISTANCE:
                        print(f"  [DRIVE] obstacle cleared (attempt {retry + 1})")
                        cleared = True
                        break
                    time.sleep(OBSTACLE_WAIT_SLEEP)
                if not cleared:
                    print("  [DRIVE] obstacle not cleared, aborting")
                    return False
                continue

            for w in self.comm.get_warnings():
                if w in ("OBSTACLE", "OBSTACLE_REAR"):
                    print(f"  [DRIVE] ESP32 WARN: {w}")
                    self.comm.send_stop()
                    time.sleep(0.5)

            avg_pulses = (telem.encoder_left + telem.encoder_right) / 2.0
            traveled = avg_pulses * m_per_pulse

            if traveled >= distance - WAYPOINT_REACHED / 2:
                self.comm.send_drive(0, 0)
                print(f"  [DRIVE] REACHED! traveled={traveled:.3f}m "
                      f"(pulses={avg_pulses:.0f}, target={distance:.2f}m)")
                return True

            # steering correction based on odometry heading error
            err = self._heading_error(telem.heading, target_heading)
            steer = self.straight_pid.compute(err)
            steer = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer))

            # log every 1 second
            if time.time() - last_log >= 1.0:
                print(f"  [DRIVE] {elapsed:.1f}s | "
                      f"pulses={avg_pulses:.0f} | "
                      f"traveled={traveled:.3f}m / {distance:.2f}m | "
                      f"heading={telem.heading:.1f}deg err={err:.1f}deg | "
                      f"steer={steer:.0f} | "
                      f"front={front_dist:.2f}m")
                last_log = time.time()

            self.comm.send_drive(BASE_SPEED, int(steer))
            time.sleep(dt)

        self.comm.send_stop()
        return False

    def _heading_error(self, current, target):
        """Returns heading difference in -180..+180 range"""
        diff = target - current
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def stop(self):
        self.running = False
        self.comm.send_stop()

    def emergency_stop(self):
        self.running = False
        self.comm.send_stop()
        print("[NAV] EMERGENCY STOP")
