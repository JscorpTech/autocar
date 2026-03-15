import time
import math
from communicator import Communicator, TelemetryData
from config import (
    WHEEL_CIRCUMFERENCE, PULSES_PER_REV, WHEEL_BASE,
    BASE_SPEED, MIN_SPEED, MAX_SPEED, TURN_SPEED,
    MAX_STEER_ANGLE, MIN_TURN_RADIUS,
    PID_KP, PID_KI, PID_KD,
    TURN_PID_KP, TURN_PID_KI, TURN_PID_KD,
    HEADING_TOLERANCE, WAYPOINT_REACHED, OBSTACLE_DISTANCE,
    COMMAND_RATE_HZ
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
        self.paused = False
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

            print(f"\n[NAV] waypoint {idx + 1}/{total}: "
                  f"{heading}°, {dist}m")

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

        while self.running:
            # overall 20-second timeout
            if time.time() - t0 > 20:
                print("  [TURN] timeout")
                self.comm.send_stop()
                return False

            telem = self.comm.get_telemetry()
            err = self._heading_error(telem.heading, target)

            # yetarlicha burilganmiz
            if abs(err) < HEADING_TOLERANCE:
                self.comm.send_drive(0, 0)
                time.sleep(0.15)
                return True

            # front obstacle check (front, front-right, front-left sensors)
            front_dist = min(telem.distance, telem.dist_front_right, telem.dist_front_left)
            if not reverse_mode and 0 < front_dist < OBSTACLE_DISTANCE:
                reverse_mode = True
                attempts += 1
                if attempts > 5:
                    print("  [TURN] not enough space, aborting")
                    self.comm.send_stop()
                    return False
                print(f"  [TURN] obstacle, reversing (attempt {attempts})")

            # compute steering angle with PID
            steer = self.heading_pid.compute(err)
            steer = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer))

            if reverse_mode:
                # reverse with steering turned the opposite way
                self.comm.send_drive(-TURN_SPEED, int(-steer))
                time.sleep(0.8)
                self.comm.send_drive(0, 0)
                time.sleep(0.2)
                reverse_mode = False
            else:
                # drive forward slowly with wheels turned
                self.comm.send_drive(TURN_SPEED, int(steer))

            time.sleep(1.0 / COMMAND_RATE_HZ)

        return False

    def _drive_distance(self, distance, target_heading):
        """
        Drive straight: steering correction based on odometry heading.
        On an Ackermann car, direction is controlled via steering angle,
        not differential motor speed.
        """
        self.straight_pid.reset()
        self.comm.reset_encoders()
        time.sleep(0.2)

        m_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REV
        t0 = time.time()
        timeout = distance * 5 + 10

        while self.running:
            if time.time() - t0 > timeout:
                print("  [DRIVE] timeout")
                self.comm.send_stop()
                return False

            telem = self.comm.get_telemetry()

            # obstacle check: front, front-right, front-left sensors
            front_dist = min(telem.distance, telem.dist_front_right, telem.dist_front_left)
            if 0 < front_dist < OBSTACLE_DISTANCE:
                print(f"  [DRIVE] obstacle! {front_dist:.2f}m")
                self.comm.send_stop()
                time.sleep(1)
                retries = 0
                while retries < 30:
                    telem = self.comm.get_telemetry()
                    front_dist = min(telem.distance, telem.dist_front_right, telem.dist_front_left)
                    if front_dist >= OBSTACLE_DISTANCE:
                        print("  [DRIVE] obstacle cleared")
                        break
                    retries += 1
                    time.sleep(0.5)
                if retries >= 30:
                    print("  [DRIVE] obstacle not cleared")
                    return False
                continue

            for w in self.comm.get_warnings():
                if w in ("OBSTACLE", "OBSTACLE_REAR"):
                    self.comm.send_stop()
                    time.sleep(0.5)

            avg_pulses = (telem.encoder_left + telem.encoder_right) / 2.0
            traveled = avg_pulses * m_per_pulse

            if traveled >= distance - WAYPOINT_REACHED / 2:
                self.comm.send_drive(0, 0)
                print(f"  [DRIVE] {traveled:.2f}m traveled")
                return True

            # steering correction based on odometry heading error
            err = self._heading_error(telem.heading, target_heading)
            steer = self.straight_pid.compute(err)
            steer = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, steer))

            self.comm.send_drive(BASE_SPEED, int(steer))
            time.sleep(1.0 / COMMAND_RATE_HZ)

        self.comm.send_stop()
        return False

    def _heading_error(self, current, target):
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
