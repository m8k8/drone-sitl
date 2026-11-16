#!/usr/bin/env python3
"""Measure one isolated G06 payload scenario; never attach to an existing vehicle."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import tempfile
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


class PayloadFlight:
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

    def fly(self):
        self.step("connect")
        heartbeat = self.wait("ArduPilot heartbeat", lambda m: m.get_type() == "HEARTBEAT" and
            m.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, wall_timeout=30)
        self.target_system, self.target_component = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
        self.link.target_system, self.link.target_component = self.target_system, self.target_component
        self.link.mav.request_data_stream_send(self.target_system, self.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        for name in ("GLOBAL_POSITION_INT", "SIMSTATE", "SIM_STATE", "SERVO_OUTPUT_RAW", "VFR_HUD", "SYS_STATUS", "BATTERY_STATUS", "ATTITUDE", "EXTENDED_SYS_STATE"):
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                         getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}"), 100000)
        self.step("prearm")
        self.wait("GPS/EKF/normal pre-arm health", self.ready, sim_timeout=120, wall_timeout=60)
        self.wait("all measurement streams", lambda _m: all(self.fresh(name) for name in
            ("GLOBAL_POSITION_INT", "SIMSTATE", "SIM_STATE", "SERVO_OUTPUT_RAW", "VFR_HUD", "SYS_STATUS", "BATTERY_STATUS")))
        marker = re.search(r"G06 frame mass=([0-9.]+) referenceMass=([0-9.]+)", self.log_path.read_text())
        if marker is None:
            raise RuntimeError("Patched model marker missing: prepare/build the G06 SITL binary first")
        observed_mass, observed_reference = map(float, marker.groups())
        if not math.isclose(observed_mass, float(self.model["mass"]), abs_tol=1e-5) or not math.isclose(observed_reference, float(self.model["referenceMass"]), abs_tol=1e-5):
            raise RuntimeError("Runtime model mass/referenceMass does not match the selected JSON")
        self.summary["runtime_model"] = {"mass_kg": observed_mass, "reference_mass_kg": observed_reference}
        for name in PARAMETERS:
            self.link.mav.param_request_read_send(self.target_system, self.target_component, name.encode(), -1)
            self.wait(f"parameter {name}", lambda m, n=name: m.get_type() == "PARAM_VALUE" and str(m.param_id).rstrip("\x00") == n, sim_timeout=15, wall_timeout=10)
        if int(self.parameters["ARMING_SKIPCHK"]) != 0:
            raise RuntimeError("Experiment requires ARMING_SKIPCHK=0 (no normal checks skipped)")
        if int(self.parameters["FRAME_CLASS"]) != 1 or int(self.parameters["FRAME_TYPE"]) != 1:
            raise RuntimeError("Experiment requires quad X (FRAME_CLASS=1, FRAME_TYPE=1)")
        if self.parameters["SIM_BATT_CAP_AH"] <= 0:
            raise RuntimeError("Experiment requires a finite simulated battery capacity")
        truth, state = self.latest["SIMSTATE"], self.latest["SIM_STATE"]
        self.true_home = (truth.lat / 1e7, truth.lng / 1e7, state.alt)
        if not all(math.isfinite(value) for value in self.true_home) or abs(self.true_home[0]) > 90 or abs(self.true_home[1]) > 180:
            raise RuntimeError("Invalid simulator truth telemetry")
        self.summary["truth_home"] = {"lat_deg": self.true_home[0], "lon_deg": self.true_home[1], "alt_m": self.true_home[2]}
        self.set_mode("GUIDED")
        self.step("arm")
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        self.wait("normal arming", lambda m: m.get_type() == "HEARTBEAT" and m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION)
        self.wait("home position", lambda _m: "HOME_POSITION" in self.latest)
        home = self.latest["HOME_POSITION"]
        self.home = (home.latitude / 1e7, home.longitude / 1e7, home.altitude / 1000)
        self.summary["navigation_home"] = {"lat_deg": self.home[0], "lon_deg": self.home[1], "alt_m": self.home[2]}
        self.throttle = 1500
        self.step("takeoff")
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, CRITERIA["takeoff_target_m"])
        self.wait("10 m takeoff", lambda m: m.get_type() == "GLOBAL_POSITION_INT" and
            9500 <= m.relative_alt <= 10500, sim_timeout=60)
        self.set_mode("LOITER")
        self.step("settle_home")
        self.duration(CRITERIA["settle_sim_seconds"])
        self.hover("hover_home")
        self.step("north_transit")
        self.set_mode("GUIDED")
        self.target_ne = (CRITERIA["north_waypoint_m"], 0.0)
        target_lat = self.home[0] + math.degrees(self.target_ne[0] / EARTH_RADIUS_M)
        self.link.mav.set_position_target_global_int_send(int(self.sim_time * 1000),
            self.target_system, self.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            3576, round(target_lat * 1e7), round(self.home[1] * 1e7), 10, 0, 0, 0, 0, 0, 0, 0, 0)
        self.wait("20 m north waypoint", lambda m: m.get_type() == "GLOBAL_POSITION_INT" and
            self.rows[-1]["estimated_horizontal_error_m"] <= CRITERIA["waypoint_acceptance_m"] and
            9000 <= m.relative_alt <= 11000, sim_timeout=90)
        self.set_mode("LOITER")
        self.step("settle_north")
        self.duration(CRITERIA["settle_sim_seconds"])
        self.hover("hover_north")
        self.step("rtl")
        self.target_ne = (0.0, 0.0)
        self.set_mode("RTL")
        self.wait("RTL landing/disarm", lambda m: m.get_type() == "HEARTBEAT" and
            not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED and
            abs(self.latest["GLOBAL_POSITION_INT"].relative_alt) < 1000 and
            self.fresh("EXTENDED_SYS_STATE") and
            self.latest["EXTENDED_SYS_STATE"].landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
            sim_timeout=180, wall_timeout=90)
        self.step("landed")
        self.duration(1)
        row = self.rows[-1]
        landing_fields = ("north_m", "east_m", "true_north_m", "true_east_m", "relative_alt_m", "true_altitude_relative_m")
        if any(row[field] is None or not math.isfinite(row[field]) for field in landing_fields):
            raise RuntimeError("Landing has missing/nonfinite position measurements")
        if row["heartbeat_age_s"] is None or row["heartbeat_age_s"] > CRITERIA["heartbeat_max_age_sim_seconds"] or row["armed"]:
            raise RuntimeError("Landing needs a fresh disarmed heartbeat")
        landing = {"estimated_home_error_m": math.hypot(row["north_m"], row["east_m"]),
                   "truth_home_error_m": math.hypot(row["true_north_m"], row["true_east_m"]),
                   "estimated_relative_alt_m": row["relative_alt_m"],
                   "truth_relative_alt_m": row["true_altitude_relative_m"], "disarmed": not bool(row["armed"]),
                   "landed_state": row["landed_state"]}
        self.summary["metrics"]["landing"] = landing
        if not all(self.fresh(name) for name in ("SIMSTATE", "SIM_STATE", "GLOBAL_POSITION_INT", "EXTENDED_SYS_STATE")):
            raise RuntimeError("Landing position telemetry is stale")
        if row["landed_state"] != mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND or abs(landing["truth_relative_alt_m"]) >= CRITERIA["landing_truth_relative_altitude_abs_max_m"]:
            raise RuntimeError("Landing requires on-ground state and simulator true altitude within 1 m of starting ground")
        if landing["estimated_home_error_m"] > CRITERIA["landing_estimated_home_error_max_m"] or landing["truth_home_error_m"] > CRITERIA["landing_truth_home_error_max_m"]:
            raise RuntimeError("Landing error exceeds 2 m")
        self.summary["metrics"]["flight"] = {
            "armed_sim_seconds": self.first_disarm_sim_s - self.first_arm_sim_s,
            "armed_time_note": "Observed heartbeat transitions using latest position time; heartbeat quantization applies.",
            "battery_consumed_mah": row["battery_consumed_mah"],
            "battery_voltage_final_v": row["battery_voltage_v"],
            "telemetry_samples": len(self.rows),
        }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, default=ROOT / "config/g06/manifest.json")
    parser.add_argument("--label", default="g06-payload")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--instance", type=int, default=30)
    parser.add_argument("--speedup", type=float, default=5)
    parser.add_argument("--home", default=HOME)
    parser.add_argument("--wall-timeout", type=float, default=480)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.instance < 30 or args.instance > 100:
        raise SystemExit("Use an isolated instance between 30 and 100")
    if not 0 < args.speedup <= 20 or args.wall_timeout <= 0:
        raise SystemExit("Invalid speedup/deadline")
    model_path, params_path = args.model.resolve(strict=True), args.params.resolve(strict=True)
    model = json.loads(model_path.read_text())
    if any(not math.isfinite(float(model.get(key, 0))) or float(model.get(key, 0)) <= 0 for key in ("mass", "referenceMass")):
        raise SystemExit("Model requires positive mass and referenceMass")
    manifest_path = args.manifest.resolve(strict=True)
    manifest = json.loads(manifest_path.read_text())
    cases = [case for case in manifest["cases"] if (manifest_path.parent / case["model"]).resolve() == model_path]
    if len(cases) != 1:
        raise SystemExit("Selected model must match exactly one manifest case")
    case = cases[0]
    payload_kg = float(case["water_l"]) * float(manifest["water_density_kg_l"])
    if not math.isclose(float(case["gross_mass_kg"]), float(model["mass"]), abs_tol=1e-6) or not math.isclose(float(manifest["empty_mass_kg"]) + payload_kg, float(model["mass"]), abs_tol=1e-6):
        raise SystemExit("Model mass does not match the manifest's empty mass plus declared payload")
    binary = SOURCE / "build/sitl/bin/arducopter"
    if not binary.is_file():
        raise SystemExit("Build SITL first")
    if args.output:
        run_dir = args.output.resolve()
        run_dir.mkdir(parents=True, exist_ok=False)
    else:
        (ROOT / "runs").mkdir(exist_ok=True)
        run_dir = Path(tempfile.mkdtemp(prefix="payload-", dir=ROOT / "runs"))
    shutil.copy2(model_path, run_dir / "model.json")
    shutil.copy2(params_path, run_dir / "experiment.parm")
    base = SOURCE / "Tools/autotest/default_params/copter.parm"
    shutil.copy2(base, run_dir / "base.parm")
    shutil.copy2(__file__, run_dir / "runner.py")
    shutil.copy2(manifest_path, run_dir / "manifest.json")
    patch_path = manifest_path.parent / "reference-mass.patch"
    if patch_path.is_file():
        shutil.copy2(patch_path, run_dir / "reference-mass.patch")
    metadata = {
        "label": args.label, "started_utc": datetime.now(timezone.utc).isoformat(),
        "model_file": model_path.name, "model_sha256": sha256(model_path),
        "actual_mass_kg": model["mass"], "reference_mass_kg": model["referenceMass"],
        "payload_kg": payload_kg, "water_l": case["water_l"], "empty_mass_kg": manifest["empty_mass_kg"],
        "manifest_sha256": sha256(manifest_path),
        "model_parameters": model, "experiment_params_sha256": sha256(params_path),
        "base_params_sha256": sha256(base), "runner_sha256": sha256(__file__),
        "ardupilot_commit": git_head(SOURCE), "project_commit": git_head(ROOT),
        "binary_sha256": sha256(binary), "platform": platform.platform(), "python": platform.python_version(),
        "project_worktree_status": subprocess.check_output(["git", "-C", str(ROOT), "status", "--porcelain"], text=True).splitlines(),
        "project_tracked_diff_sha256": hashlib.sha256(subprocess.check_output(["git", "-C", str(ROOT), "diff", "HEAD", "--binary"])).hexdigest(),
        "ardupilot_worktree_status": subprocess.check_output(["git", "-C", str(SOURCE), "status", "--porcelain"], text=True).splitlines(),
        "ardupilot_tracked_diff_sha256": hashlib.sha256(subprocess.check_output(["git", "-C", str(SOURCE), "diff", "HEAD", "--binary"])).hexdigest(),
        "instance": args.instance, "speedup": args.speedup, "home": args.home,
        "measurement_note": "SIMSTATE integer latitude/longitude; SIM_STATE altitude. Truth fields are asynchronous latest values with recorded age. Hover truth error is relative to commanded navigation coordinates; landing truth error is relative to physical takeoff position.",
    }
    summary = {"label": args.label, "result": "failed", "criteria": CRITERIA,
               "completed_steps": [], "events": [], "recent_status_text": [], "metrics": {}}
    link = process = flight = None
    started = time.monotonic()
    (run_dir / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
    try:
        reservations = []
        try:
            for offset in (0, 2, 3):
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                reservations.append(sock)
                sock.bind(("0.0.0.0", 5760 + 10 * args.instance + offset))
        finally:
            for sock in reservations:
                sock.close()
        link = mavutil.mavlink_connection("udpin:127.0.0.1:0", source_system=255, source_component=190)
        port = link.port.getsockname()[1]
        # AP_Filesystem maps absolute paths into the SITL working directory.
        # Use the input snapshots beside eeprom/logs, relative to this run's cwd.
        command = [str(binary), "-I", str(args.instance), "--model", "X:model.json",
                   "--speedup", str(args.speedup), "--home", args.home,
                   "--defaults", "base.parm,experiment.parm",
                   "--serial0", f"udpclient:127.0.0.1:{port}"]
        metadata["command"] = command
        (run_dir / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(f"Run: {run_dir}", flush=True)
        with (run_dir / "sitl.log").open("w") as log, (run_dir / "telemetry.csv").open("w", newline="") as csv_file:
            process = subprocess.Popen(command, cwd=run_dir, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            flight = PayloadFlight(link, process, csv_file, summary, args.wall_timeout, run_dir / "sitl.log", model)
            flight.fly()
        summary["result"] = "passed"
        print("PASS: normal arm, 10 m hover, 20 m north flight, RTL landing", flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        summary["error"] = str(error) or type(error).__name__
        print(f"FAIL: {summary['error']}", file=sys.stderr, flush=True)
        return 1
    finally:
        if link is not None:
            link.close()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        if flight is not None:
            (run_dir / "parameters.json").write_text(json.dumps(flight.parameters, indent=2, sort_keys=True) + "\n")
            summary["sim_elapsed_seconds"] = flight.sim_time
        summary["elapsed_wall_seconds"] = round(time.monotonic() - started, 3)
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        print(f"Result: {run_dir / 'summary.json'}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
