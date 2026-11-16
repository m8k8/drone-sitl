#!/usr/bin/env python3
"""Shared telemetry, commands, and measurements for the B-D prototype tests."""

from __future__ import annotations

import csv
import hashlib
import math
import os
from pathlib import Path
import statistics
import subprocess
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "vendor" / "ardupilot"
HOME = "-35.363261,149.165230,584,353"
EARTH_RADIUS_M = 6378137.0
CSV_FIELDS = [
    "time_boot_ms", "phase", "lat_deg", "lon_deg", "relative_alt_m",
    "north_m", "east_m", "true_lat_deg", "true_lon_deg", "true_alt_m",
    "true_north_m", "true_east_m", "true_altitude_relative_m", "true_sample_age_s",
    "estimated_horizontal_error_m", "truth_horizontal_error_m",
    "roll_deg", "pitch_deg", "tilt_deg", "servo1", "servo2", "servo3", "servo4",
    "servo_sample_age_s", "throttle_percent", "battery_voltage_v",
    "battery_current_a", "battery_consumed_mah", "armed", "mode", "heartbeat_age_s",
    "battery_sample_age_s", "throttle_sample_age_s", "attitude_sample_age_s",
    "landed_state", "landed_state_age_s",
]
CRITERIA = {
    "takeoff_target_m": 10.0,
    "takeoff_acceptance_m": [9.5, 10.5],
    "settle_sim_seconds": 5.0,
    "hover_sim_seconds": 10.0,
    "hover_altitude_bounds_m": [9.0, 11.0],
    "hover_horizontal_error_max_m": 2.0,
    "hover_min_samples": 50,
    "north_waypoint_m": 20.0,
    "waypoint_acceptance_m": 1.0,
    "landing_relative_altitude_abs_max_m": 1.0,
    "landing_truth_relative_altitude_abs_max_m": 1.0,
    "landing_requires_on_ground": True,
    "landing_estimated_home_error_max_m": 2.0,
    "landing_truth_home_error_max_m": 2.0,
    "telemetry_max_age_sim_seconds": 1.0,
    "heartbeat_max_age_sim_seconds": 2.0,
}
PARAMETERS = [
    "ARMING_SKIPCHK", "AHRS_EKF_TYPE", "FRAME_CLASS", "FRAME_TYPE",
    "MOT_THST_HOVER", "MOT_HOVER_LEARN", "MOT_THST_EXPO", "MOT_SPIN_MIN",
    "MOT_SPIN_MAX", "MOT_PWM_MIN", "MOT_PWM_MAX", "MOT_BAT_VOLT_MIN",
    "MOT_BAT_VOLT_MAX", "SIM_BATT_VOLTAGE", "SIM_BATT_CAP_AH",
    "BATT_CAPACITY", "BATT_MONITOR", "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT",
    "SIM_WIND_SPD", "SIM_WIND_TURB", "SIM_GPS1_ENABLE", "SIM_GPS1_NOISE",
    "SIM_GPS1_LAG_MS", "RNGFND1_TYPE", "SPRAY_ENABLE", "SIM_SPR_ENABLE",
    "SERIAL0_PROTOCOL", "ATC_RAT_RLL_P", "ATC_RAT_PIT_P", "ATC_RAT_YAW_P",
]


def sha256(path):
    with Path(path).open("rb") as file:
        return hashlib.file_digest(file, "sha256").hexdigest()


def git_head(path):
    return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()


def local_ne(lat, lon, origin):
    return (
        math.radians(lat - origin[0]) * EARTH_RADIUS_M,
        math.radians(lon - origin[1]) * EARTH_RADIUS_M * math.cos(math.radians(origin[0])),
    )


def distribution(values):
    values = [float(value) for value in values if value is not None]
    if not values:
        return {"mean": None, "min": None, "max": None}
    return {"mean": statistics.fmean(values), "min": min(values), "max": max(values)}


class FlightSupport:
    def __init__(self, link, process, csv_file, summary, wall_timeout, log_path, model):
        self.link, self.process, self.summary = link, process, summary
        self.writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.wall_start, self.wall_timeout = time.monotonic(), wall_timeout
        self.last_send = 0.0
        self.target_system = self.target_component = None
        self.latest, self.seen, self.parameters = {}, {}, {}
        self.sim_time = 0.0
        self.phase = "startup"
        self.home = self.true_home = None
        self.target_ne = (0.0, 0.0)
        self.throttle = 1000
        self.rows = []
        self.first_arm_sim_s = None
        self.first_disarm_sim_s = None
        self.log_path, self.model = log_path, model

    def step(self, label):
        self.phase = label
        self.summary["events"].append({"phase": label, "time_boot_s": self.sim_time})
        print(f"[{self.sim_time:.1f}s] {label}", flush=True)

    def receive(self):
        if self.process.poll() is not None:
            raise RuntimeError(f"SITL exited: {self.process.returncode}")
        if time.monotonic() - self.wall_start > self.wall_timeout:
            raise TimeoutError(f"Wall-clock deadline exceeded: {self.wall_timeout}s")
        now = time.monotonic()
        if self.target_system is not None and now - self.last_send >= 0.1:
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            self.link.mav.rc_channels_override_send(self.target_system, self.target_component,
                1500, 1500, self.throttle, 1500, 65535, 65535, 65535, 65535)
            self.last_send = now
        message = self.link.recv_match(blocking=True, timeout=0.05)
        if message is None or message.get_type() == "BAD_DATA":
            return None
        if self.target_system is not None and message.get_srcSystem() != self.target_system:
            return None
        kind = message.get_type()
        if kind == "GLOBAL_POSITION_INT":
            self.sim_time = message.time_boot_ms / 1000.0
        self.latest[kind] = message
        self.seen[kind] = self.sim_time
        if kind == "PARAM_VALUE":
            self.parameters[str(message.param_id).rstrip("\x00")] = message.param_value
        elif kind == "HEARTBEAT":
            armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed and self.first_arm_sim_s is None:
                self.first_arm_sim_s = self.sim_time
            elif not armed and self.first_arm_sim_s is not None and self.first_disarm_sim_s is None:
                self.first_disarm_sim_s = self.sim_time
        elif kind == "STATUSTEXT":
            text = str(message.text).rstrip("\x00")
            self.summary["recent_status_text"].append(text)
            self.summary["recent_status_text"] = self.summary["recent_status_text"][-60:]
            print(f"  SITL: {text}", flush=True)
            lower = text.lower()
            if "disarm" in lower and ("crash" in lower or "emergency" in lower):
                raise RuntimeError(f"Abnormal disarm: {text}")
        elif kind == "GLOBAL_POSITION_INT":
            self.record(message)
        return message

    def wait(self, label, predicate, sim_timeout=60, wall_timeout=60):
        start_sim, start_wall = self.sim_time, time.monotonic()
        while time.monotonic() - start_wall < wall_timeout:
            message = self.receive()
            if message is not None and predicate(message):
                self.summary["completed_steps"].append(label)
                return message
            if self.sim_time - start_sim > sim_timeout:
                raise TimeoutError(f"Simulation timeout: {label} ({sim_timeout}s)")
        raise TimeoutError(f"Wall-clock timeout: {label} ({wall_timeout}s)")

    def command(self, command, *params):
        self.link.mav.command_long_send(self.target_system, self.target_component,
            command, 0, *(list(params) + [0] * (7 - len(params))))
        ack = self.wait(f"command {command}",
            lambda m: m.get_type() == "COMMAND_ACK" and m.command == command,
            sim_timeout=20, wall_timeout=15)
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"Command {command} rejected: MAV_RESULT={ack.result}")

    def set_mode(self, mode):
        modes = self.link.mode_mapping()
        if not modes or mode not in modes:
            raise RuntimeError(f"Unavailable mode: {mode}")
        self.link.mav.set_mode_send(self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, modes[mode])
        self.wait(f"mode {mode}", lambda m: m.get_type() == "HEARTBEAT" and m.custom_mode == modes[mode])

    def fresh(self, kind):
        return kind in self.latest and self.sim_time - self.seen[kind] <= CRITERIA["telemetry_max_age_sim_seconds"]

    def record(self, position):
        row = {field: None for field in CSV_FIELDS}
        row.update(time_boot_ms=position.time_boot_ms, phase=self.phase,
                   lat_deg=position.lat / 1e7, lon_deg=position.lon / 1e7,
                   relative_alt_m=position.relative_alt / 1000)
        heartbeat = self.latest.get("HEARTBEAT")
        if heartbeat is not None:
            row["armed"] = int(bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
            row["mode"] = heartbeat.custom_mode
            row["heartbeat_age_s"] = self.sim_time - self.seen["HEARTBEAT"]
        extended = self.latest.get("EXTENDED_SYS_STATE")
        if extended is not None:
            row["landed_state"] = extended.landed_state
            row["landed_state_age_s"] = self.sim_time - self.seen["EXTENDED_SYS_STATE"]
        if self.home:
            row["north_m"], row["east_m"] = local_ne(row["lat_deg"], row["lon_deg"], self.home)
            row["estimated_horizontal_error_m"] = math.hypot(row["north_m"] - self.target_ne[0], row["east_m"] - self.target_ne[1])
        truth = self.latest.get("SIMSTATE")
        state = self.latest.get("SIM_STATE")
        if truth is not None and state is not None:
            row.update(true_lat_deg=truth.lat / 1e7, true_lon_deg=truth.lng / 1e7,
                       true_alt_m=state.alt, true_sample_age_s=max(self.sim_time - self.seen["SIMSTATE"], self.sim_time - self.seen["SIM_STATE"]))
            if self.true_home:
                row["true_north_m"], row["true_east_m"] = local_ne(row["true_lat_deg"], row["true_lon_deg"], self.true_home)
                row["true_altitude_relative_m"] = row["true_alt_m"] - self.true_home[2]
            if self.home:
                true_target_n, true_target_e = local_ne(row["true_lat_deg"], row["true_lon_deg"], self.home)
                row["truth_horizontal_error_m"] = math.hypot(true_target_n - self.target_ne[0], true_target_e - self.target_ne[1])
        attitude = self.latest.get("ATTITUDE")
        if attitude is not None:
            row["roll_deg"], row["pitch_deg"] = math.degrees(attitude.roll), math.degrees(attitude.pitch)
            row["tilt_deg"] = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(attitude.roll) * math.cos(attitude.pitch)))))
            row["attitude_sample_age_s"] = self.sim_time - self.seen["ATTITUDE"]
        servos = self.latest.get("SERVO_OUTPUT_RAW")
        if servos is not None:
            for motor in range(1, 5):
                row[f"servo{motor}"] = getattr(servos, f"servo{motor}_raw")
            row["servo_sample_age_s"] = self.sim_time - self.seen["SERVO_OUTPUT_RAW"]
        hud = self.latest.get("VFR_HUD")
        if hud is not None:
            row["throttle_percent"] = hud.throttle
            row["throttle_sample_age_s"] = self.sim_time - self.seen["VFR_HUD"]
        battery = self.latest.get("SYS_STATUS")
        if battery is not None:
            row["battery_voltage_v"] = battery.voltage_battery / 1000 if battery.voltage_battery != 65535 else None
            row["battery_current_a"] = battery.current_battery / 100 if battery.current_battery != -1 else None
            row["battery_sample_age_s"] = self.sim_time - self.seen["SYS_STATUS"]
        consumed = self.latest.get("BATTERY_STATUS")
        if consumed is not None and consumed.current_consumed >= 0:
            row["battery_consumed_mah"] = consumed.current_consumed
        self.writer.writerow(row)
        self.rows.append(row)

    def ready(self, _message):
        gps, ekf, status = (self.latest.get(name) for name in ("GPS_RAW_INT", "EKF_STATUS_REPORT", "SYS_STATUS"))
        required = (mavutil.mavlink.EKF_ATTITUDE | mavutil.mavlink.EKF_VELOCITY_HORIZ |
                    mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_POS_VERT_ABS)
        return (gps is not None and gps.fix_type >= 3 and ekf is not None and
                ekf.flags & required == required and not ekf.flags & mavutil.mavlink.EKF_CONST_POS_MODE and
                status is not None and status.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)

    def duration(self, seconds):
        start = self.sim_time
        self.wait(f"{self.phase} {seconds:g}s", lambda _m: self.sim_time - start >= seconds,
                  sim_timeout=seconds + 5, wall_timeout=max(30, seconds * 2))

    def hover(self, name):
        self.step(name)
        self.duration(CRITERIA["hover_sim_seconds"])
        rows = [row for row in self.rows if row["phase"] == name]
        if len(rows) < CRITERIA["hover_min_samples"]:
            raise RuntimeError(f"{name}: only {len(rows)} position samples")
        for row in rows:
            if row["heartbeat_age_s"] is None or row["heartbeat_age_s"] > CRITERIA["heartbeat_max_age_sim_seconds"]:
                raise RuntimeError(f"{name}: stale heartbeat")
            if not row["armed"] or row["mode"] != self.link.mode_mapping()["LOITER"]:
                raise RuntimeError(f"{name}: vehicle left armed LOITER mode")
            required_fields = ["relative_alt_m", "true_altitude_relative_m", "estimated_horizontal_error_m",
                "truth_horizontal_error_m", "battery_current_a", "battery_voltage_v", "throttle_percent", "tilt_deg",
                "servo1", "servo2", "servo3", "servo4"]
            if any(row[field] is None or not math.isfinite(row[field]) for field in required_fields):
                raise RuntimeError(f"{name}: missing or nonfinite measurement")
            if row["true_sample_age_s"] is None or row["true_sample_age_s"] > 1:
                raise RuntimeError(f"{name}: missing/stale SIMSTATE or SIM_STATE telemetry")
            if row["servo_sample_age_s"] is None or row["servo_sample_age_s"] > 1 or any(not row[f"servo{i}"] for i in range(1, 5)):
                raise RuntimeError(f"{name}: missing/stale servo telemetry")
            if any(row[field] is None or row[field] > 1 for field in
                   ("battery_sample_age_s", "throttle_sample_age_s", "attitude_sample_age_s")):
                raise RuntimeError(f"{name}: stale current, voltage, throttle or attitude telemetry")
        metrics = {"samples": len(rows), "duration_s": (rows[-1]["time_boot_ms"] - rows[0]["time_boot_ms"]) / 1000}
        for field, prefix in [("relative_alt_m", "altitude"), ("throttle_percent", "throttle_percent"),
                              ("battery_voltage_v", "battery_voltage"), ("battery_current_a", "battery_current"),
                              ("tilt_deg", "tilt_deg")]:
            unit = {"altitude": "_m", "battery_voltage": "_v", "battery_current": "_a"}.get(prefix, "")
            for stat, value in distribution(row[field] for row in rows).items():
                metrics[f"{prefix}_{stat}{unit}"] = value
        metrics["estimated_horizontal_error_max_m"] = max(row["estimated_horizontal_error_m"] for row in rows)
        metrics["truth_horizontal_error_max_m"] = max(row["truth_horizontal_error_m"] for row in rows)
        pwm = [row[f"servo{i}"] for row in rows for i in range(1, 5)]
        metrics["servo_pwm_mean"], metrics["servo_pwm_max"] = statistics.fmean(pwm), max(pwm)
        self.summary["metrics"][name] = metrics
        low, high = CRITERIA["hover_altitude_bounds_m"]
        if metrics["altitude_min_m"] < low or metrics["altitude_max_m"] > high:
            raise RuntimeError(f"{name}: altitude outside {low}..{high} m")
        if metrics["estimated_horizontal_error_max_m"] > CRITERIA["hover_horizontal_error_max_m"] or metrics["truth_horizontal_error_max_m"] > CRITERIA["hover_horizontal_error_max_m"]:
            raise RuntimeError(f"{name}: horizontal error exceeds {CRITERIA['hover_horizontal_error_max_m']} m")
