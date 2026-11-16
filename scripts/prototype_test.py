#!/usr/bin/env python3
"""Run one isolated B-D prototype flight, AUTO mission, or rangefinder experiment."""

from __future__ import annotations

import argparse
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

import flight_support as base
from pymavlink import mavutil

ROOT, SOURCE = base.ROOT, base.SOURCE
CONFIG = ROOT / "config/prototype-01"
SENSOR_FIELDS = ["distance_m", "distance_min_m", "distance_max_m", "distance_age_s",
                 "distance_signal_quality", "laser_health_bit", "sonar_position_z_m",
                 "mission_seq"]
base.CSV_FIELDS.extend(SENSOR_FIELDS)
FLIGHT_CRITERIA = {**base.CRITERIA, "settle_sim_seconds": 8.0}
RANGE_CRITERIA = {
    "baseline_measure_s": 8, "response_settle_s": 12, "response_measure_s": 6,
    "synthetic_position_step_m": 1.0, "step_altitude_change_m": [0.6, 1.4],
    "return_altitude_error_max_m": 0.5, "out_of_range_position_z_m": -50.0,
    "fallback_measure_s": 10, "fallback_altitude_change_max_m": 0.75,
    "range_truth_error_max_m": 0.5, "range_max_age_s": 1.0,
    "measurement_min_samples": 30, "horizontal_error_max_m": 2.0,
    "log_min_samples_per_stream": 5,
}
MISSION_CRITERIA = {"side_m": 20, "altitude_m": 10, "waypoint_truth_error_max_m": 2,
                    "waypoint_altitude_bounds_m": [9, 11], "mission_timeout_sim_s": 240}
EXTRA_PARAMETERS = ["SERIAL5_PROTOCOL", "SERIAL5_BAUD", "RNGFND1_MIN", "RNGFND1_MAX",
                    "RNGFND1_ORIENT", "RNGFND1_GNDCLR", "SURFTRAK_MODE",
                    "SIM_SONAR_POS_Z", "SIM_SONAR_RND", "LOG_BITMASK"]


class NullWriter:
    def writerow(self, _row):
        pass


class PrototypeFlight(base.FlightSupport):
    def __init__(self, *args, scenario, expected_params, **kwargs):
        super().__init__(*args, **kwargs)
        self.scenario, self.expected_params = scenario, expected_params
        self.sonar_position_z = 0.0
        self.mission_reached = []
        self.waypoints = []
        self.waypoint_closest = {}

    def receive(self):
        message = super().receive()
        if message is not None and message.get_type() == "MISSION_ITEM_REACHED":
            self.mission_reached.append({"seq": message.seq, "time_boot_s": self.sim_time})
        return message

    def record(self, position):
        writer, self.writer = self.writer, NullWriter()
        try:
            super().record(position)
        finally:
            self.writer = writer
        row = self.rows[-1]
        distance = self.latest.get("DISTANCE_SENSOR")
        if distance is not None:
            row.update(distance_m=distance.current_distance / 100.0,
                       distance_min_m=distance.min_distance / 100.0,
                       distance_max_m=distance.max_distance / 100.0,
                       distance_age_s=self.sim_time - self.seen["DISTANCE_SENSOR"],
                       distance_signal_quality=getattr(distance, "signal_quality", None))
        status = self.latest.get("SYS_STATUS")
        if status is not None:
            row["laser_health_bit"] = int(bool(status.onboard_control_sensors_health &
                mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION))
        row["sonar_position_z_m"] = self.sonar_position_z
        current = self.latest.get("MISSION_CURRENT")
        if current is not None:
            row["mission_seq"] = current.seq
        if (self.phase == "auto_mission" and row["true_lat_deg"] is not None and
                row["mode"] == self.link.mode_mapping().get("AUTO") and current is not None):
            for waypoint in self.waypoints:
                if current.seq != waypoint["seq"]:
                    continue
                n, e = base.local_ne(row["true_lat_deg"], row["true_lon_deg"], self.home)
                error = math.hypot(n - waypoint["north_m"], e - waypoint["east_m"])
                previous = self.waypoint_closest.get(waypoint["seq"])
                if previous is None or error < previous["truth_error_m"]:
                    self.waypoint_closest[waypoint["seq"]] = {
                        "truth_error_m": error, "relative_alt_m": row["relative_alt_m"],
                        "true_altitude_relative_m": row["true_altitude_relative_m"],
                        "time_boot_s": self.sim_time, "mode": row["mode"],
                        "true_sample_age_s": row["true_sample_age_s"],
                        "heartbeat_age_s": row["heartbeat_age_s"], "armed": row["armed"],
                    }
        writer.writerow(row)

    def parameter(self, name):
        self.link.mav.param_request_read_send(self.target_system, self.target_component, name.encode(), -1)
        message = self.wait(f"parameter {name}", lambda m: m.get_type() == "PARAM_VALUE" and
            str(m.param_id).rstrip("\x00") == name, sim_timeout=15, wall_timeout=10)
        return message.param_value

    def set_parameter(self, name, value):
        self.link.mav.param_set_send(self.target_system, self.target_component, name.encode(),
                                    value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.wait(f"set parameter {name}={value}", lambda m: m.get_type() == "PARAM_VALUE" and
            str(m.param_id).rstrip("\x00") == name and math.isclose(m.param_value, value, abs_tol=1e-5),
            sim_timeout=15, wall_timeout=10)
        if name == "SIM_SONAR_POS_Z":
            self.sonar_position_z = value
        self.summary.setdefault("parameter_changes", []).append(
            {"time_boot_s": self.sim_time, "parameter": name, "value": value})

    def prepare(self):
        self.step("connect")
        heartbeat = self.wait("ArduPilot heartbeat", lambda m: m.get_type() == "HEARTBEAT" and
            m.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, wall_timeout=30)
        self.target_system, self.target_component = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
        self.link.target_system, self.link.target_component = self.target_system, self.target_component
        self.link.mav.request_data_stream_send(self.target_system, self.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        for name in ("GLOBAL_POSITION_INT", "SIMSTATE", "SIM_STATE", "SERVO_OUTPUT_RAW", "VFR_HUD",
                     "SYS_STATUS", "BATTERY_STATUS", "ATTITUDE", "EXTENDED_SYS_STATE", "DISTANCE_SENSOR", "MISSION_CURRENT"):
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                         getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}"), 100000)
        self.step("prearm")
        self.wait("GPS/EKF/normal pre-arm health", self.ready, sim_timeout=120, wall_timeout=60)
        self.wait("measurement streams", lambda _m: all(self.fresh(name) for name in
            ("GLOBAL_POSITION_INT", "SIMSTATE", "SIM_STATE", "SERVO_OUTPUT_RAW", "VFR_HUD", "SYS_STATUS")))
        marker = re.search(r"G06 frame mass=([0-9.]+) referenceMass=([0-9.]+)", self.log_path.read_text())
        if marker is None:
            raise RuntimeError("Patched fixed-propulsion model marker missing")
        mass, reference = map(float, marker.groups())
        if not math.isclose(mass, self.model["mass"], abs_tol=1e-5) or not math.isclose(reference, self.model["referenceMass"], abs_tol=1e-5):
            raise RuntimeError("Runtime model differs from input snapshot")
        self.summary["runtime_model"] = {"mass_kg": mass, "reference_mass_kg": reference}
        for name in dict.fromkeys(base.PARAMETERS + EXTRA_PARAMETERS):
            self.parameter(name)
        for name in ("ARMING_SKIPCHK", "FRAME_CLASS", "FRAME_TYPE", "SERIAL5_PROTOCOL", "SERIAL5_BAUD",
                     "RNGFND1_TYPE", "RNGFND1_MIN", "RNGFND1_MAX", "RNGFND1_ORIENT", "SURFTRAK_MODE"):
            expected = self.expected_params[name]
            if not math.isclose(self.parameters[name], expected, abs_tol=1e-5):
                raise RuntimeError(f"Parameter {name}={self.parameters[name]} != expected {expected}")
        if self.parameters["ARMING_SKIPCHK"] != 0 or self.parameters["SIM_BATT_CAP_AH"] <= 0:
            raise RuntimeError("Normal arm checks and finite battery required")
        if not int(self.parameters["LOG_BITMASK"]) & (1 << 4):
            raise RuntimeError("CTUN/RFND log bit must be enabled")
        self.summary["initial_parameters"] = dict(self.parameters)
        truth, state = self.latest["SIMSTATE"], self.latest["SIM_STATE"]
        self.true_home = (truth.lat / 1e7, truth.lng / 1e7, state.alt)
        if not all(math.isfinite(v) for v in self.true_home):
            raise RuntimeError("Invalid simulator truth origin")
        self.summary["truth_home"] = dict(zip(("lat_deg", "lon_deg", "alt_m"), self.true_home))
        self.set_mode("GUIDED")
        self.step("arm")
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        self.wait("normal arming", lambda m: m.get_type() == "HEARTBEAT" and
            m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION)
        self.wait("home position", lambda _m: "HOME_POSITION" in self.latest)
        home = self.latest["HOME_POSITION"]
        self.home = (home.latitude / 1e7, home.longitude / 1e7, home.altitude / 1000)
        self.summary["navigation_home"] = dict(zip(("lat_deg", "lon_deg", "alt_m"), self.home))
        self.throttle = 1500
        self.step("takeoff")
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10)
        self.wait("10 m takeoff", lambda m: m.get_type() == "GLOBAL_POSITION_INT" and
            9500 <= m.relative_alt <= 10500, sim_timeout=60)
        self.set_mode("LOITER")
        self.step("settle_home")
        self.duration(FLIGHT_CRITERIA["settle_sim_seconds"])

    def hover(self, name):
        super().hover(name)
        rows = [r for r in self.rows if r["phase"] == name]
        truth = base.distribution(r["true_altitude_relative_m"] for r in rows)
        self.summary["metrics"][name].update({f"true_altitude_{key}_m": value for key, value in truth.items()})
        if truth["min"] < 9 or truth["max"] > 11:
            raise RuntimeError(f"{name}: physical model altitude outside 9..11 m")
        self.range_measurements(name, require_in_range=True)

    def range_measurements(self, name, require_in_range, duration=None):
        if duration is not None:
            self.step(name)
            self.duration(duration)
        rows = [r for r in self.rows if r["phase"] == name]
        if len(rows) < RANGE_CRITERIA["measurement_min_samples"]:
            raise RuntimeError(f"{name}: insufficient measurement samples")
        for row in rows:
            needed = ("relative_alt_m", "true_altitude_relative_m", "truth_horizontal_error_m",
                      "distance_m", "distance_min_m", "distance_max_m", "distance_age_s", "tilt_deg")
            if any(row[k] is None or not math.isfinite(row[k]) for k in needed):
                raise RuntimeError(f"{name}: missing/nonfinite range or navigation data")
            if row["distance_age_s"] > 1 or row["true_sample_age_s"] is None or row["true_sample_age_s"] > 1:
                raise RuntimeError(f"{name}: stale sensor/truth data")
            if row["heartbeat_age_s"] is None or row["heartbeat_age_s"] > 2 or not row["armed"] or row["mode"] != self.link.mode_mapping()["LOITER"]:
                raise RuntimeError(f"{name}: needs fresh armed LOITER heartbeat")
            if row["truth_horizontal_error_m"] > RANGE_CRITERIA["horizontal_error_max_m"]:
                raise RuntimeError(f"{name}: horizontal position error > 2 m")
            in_range = row["distance_min_m"] <= row["distance_m"] <= row["distance_max_m"]
            if require_in_range != in_range:
                raise RuntimeError(f"{name}: unexpected range validity")
        result = {
            "samples": len(rows), "start_s": rows[0]["time_boot_ms"] / 1000,
            "end_s": rows[-1]["time_boot_ms"] / 1000,
            "altitude_mean_m": statistics.fmean(r["relative_alt_m"] for r in rows),
            "altitude_min_m": min(r["relative_alt_m"] for r in rows),
            "altitude_max_m": max(r["relative_alt_m"] for r in rows),
            "true_altitude_mean_m": statistics.fmean(r["true_altitude_relative_m"] for r in rows),
            "true_altitude_min_m": min(r["true_altitude_relative_m"] for r in rows),
            "true_altitude_max_m": max(r["true_altitude_relative_m"] for r in rows),
            "distance_mean_m": statistics.fmean(r["distance_m"] for r in rows),
            "distance_min_m": min(r["distance_m"] for r in rows),
            "distance_max_m": max(r["distance_m"] for r in rows),
            "truth_horizontal_error_max_m": max(r["truth_horizontal_error_m"] for r in rows),
            "laser_health_bit_values": sorted(set(r["laser_health_bit"] for r in rows)),
            "sensor_position_z_m": self.sonar_position_z,
        }
        self.summary["metrics"].setdefault("rangefinder", {})[name] = result
        return result

    def flight_scenario(self):
        self.hover("hover_home")
        self.step("north_transit")
        self.set_mode("GUIDED")
        self.target_ne = (20.0, 0.0)
        lat = self.home[0] + math.degrees(20 / base.EARTH_RADIUS_M)
        self.link.mav.set_position_target_global_int_send(int(self.sim_time * 1000), self.target_system,
            self.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 3576,
            round(lat * 1e7), round(self.home[1] * 1e7), 10, 0, 0, 0, 0, 0, 0, 0, 0)
        self.wait("20 m north waypoint", lambda m: m.get_type() == "GLOBAL_POSITION_INT" and
            self.rows[-1]["estimated_horizontal_error_m"] <= 1 and 9000 <= m.relative_alt <= 11000,
            sim_timeout=90)
        self.set_mode("LOITER")
        self.step("settle_north")
        self.duration(FLIGHT_CRITERIA["settle_sim_seconds"])
        self.hover("hover_north")

    def upload_mission(self):
        points = [(20, 0), (20, 20), (0, 20), (0, 0)]
        self.waypoints = [{"seq": i + 1, "north_m": n, "east_m": e} for i, (n, e) in enumerate(points)]
        items = [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0)]
        items += [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, n, e, 10, 2) for n, e in points]
        items += [(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0)]
        self.link.mav.mission_count_send(self.target_system, self.target_component, len(items), 0)
        requested = set()
        start = time.monotonic()
        while time.monotonic() - start < 30:
            message = self.receive()
            if message is None:
                continue
            if message.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                if not 0 <= message.seq < len(items):
                    raise RuntimeError(f"Invalid mission request {message.seq}")
                cmd, north, east, alt, hold = items[message.seq]
                lat = self.home[0] + math.degrees(north / base.EARTH_RADIUS_M)
                lon = self.home[1] + math.degrees(east / (base.EARTH_RADIUS_M * math.cos(math.radians(self.home[0]))))
                self.link.mav.mission_item_int_send(self.target_system, self.target_component,
                    message.seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd,
                    0, 1, hold, 1 if cmd == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT else 0,
                    0, math.nan if cmd == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT else 0,
                    round(lat * 1e7), round(lon * 1e7), alt, 0)
                requested.add(message.seq)
            elif message.get_type() == "MISSION_ACK":
                if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED or requested != set(range(len(items))):
                    raise RuntimeError(f"Mission upload rejected/incomplete: {message.type}, requested={requested}")
                self.summary["mission"] = {"items": [dict(seq=i, command=x[0], north_m=x[1], east_m=x[2], altitude_m=x[3], hold_s=x[4]) for i, x in enumerate(items)],
                                           "upload_ack": message.type}
                return
        raise TimeoutError("Mission upload did not complete")

    def mission_scenario(self):
        self.hover("hover_home")
        self.step("mission_upload")
        self.upload_mission()
        self.step("auto_mission")
        # Copter 4.7.2 rejects nonzero first/last-item parameters. This command
        # performs the AUTO transition and starts/resumes the uploaded mission.
        self.command(mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)
        self.wait("AUTO mission mode", lambda m: m.get_type() == "HEARTBEAT" and
            m.custom_mode == self.link.mode_mapping()["AUTO"])
        self.wait("AUTO rectangle and mission RTL item", lambda m: m.get_type() == "MISSION_CURRENT" and
            m.seq == 5 and {1, 2, 3, 4}.issubset({x["seq"] for x in self.mission_reached}),
            sim_timeout=MISSION_CRITERIA["mission_timeout_sim_s"], wall_timeout=120)
        self.summary["mission"]["reached"] = self.mission_reached
        self.summary["mission"]["rtl_execution_note"] = "Mission item 5 is NAV_RETURN_TO_LAUNCH. Copter runs the RTL submode while its flight mode remains AUTO."
        self.summary["metrics"]["mission"] = {"waypoint_closest": self.waypoint_closest}
        reached = {x["seq"] for x in self.mission_reached}
        for waypoint in self.waypoints:
            seq = waypoint["seq"]
            closest = self.waypoint_closest.get(seq)
            if seq not in reached or closest is None:
                raise RuntimeError(f"AUTO waypoint {seq} was not confirmed reached")
            if (closest["truth_error_m"] > 2 or not 9 <= closest["relative_alt_m"] <= 11 or
                    not 9 <= closest["true_altitude_relative_m"] <= 11 or closest["true_sample_age_s"] > 1 or
                    closest["heartbeat_age_s"] is None or closest["heartbeat_age_s"] > 2 or
                    not closest["armed"] or closest["mode"] != self.link.mode_mapping()["AUTO"]):
                raise RuntimeError(f"AUTO waypoint {seq} missed truth/altitude/mode criterion: {closest}")

    def rangefinder_scenario(self):
        baseline = self.range_measurements("range_baseline", True, 8)
        rows = [r for r in self.rows if r["phase"] == "range_baseline"]
        errors = [abs(r["distance_m"] * math.cos(math.radians(r["tilt_deg"])) - r["true_altitude_relative_m"]) for r in rows]
        baseline["range_truth_error_max_m"] = max(errors)
        if max(errors) > RANGE_CRITERIA["range_truth_error_max_m"] or not 9 <= baseline["altitude_mean_m"] <= 11:
            raise RuntimeError("Baseline range/true height disagreement")
        self.step("range_bias_injection")
        self.set_parameter("SIM_SONAR_POS_Z", 1)
        self.duration(12)
        shifted = self.range_measurements("range_bias_response", True, 6)
        delta = shifted["altitude_mean_m"] - baseline["altitude_mean_m"]
        shifted["altitude_change_m"] = delta
        truth_delta = shifted["true_altitude_mean_m"] - baseline["true_altitude_mean_m"]
        shifted["true_altitude_change_m"] = truth_delta
        if not 0.6 <= delta <= 1.4 or not 0.6 <= truth_delta <= 1.4:
            raise RuntimeError(f"Surface tracking did not respond to 1 m synthetic range step: estimated={delta}, truth={truth_delta}")
        self.step("range_bias_restore")
        self.set_parameter("SIM_SONAR_POS_Z", 0)
        self.duration(12)
        restored = self.range_measurements("range_bias_restored", True, 6)
        restored["altitude_change_m"] = restored["altitude_mean_m"] - baseline["altitude_mean_m"]
        restored["true_altitude_change_m"] = restored["true_altitude_mean_m"] - baseline["true_altitude_mean_m"]
        if abs(restored["altitude_change_m"]) > 0.5 or abs(restored["true_altitude_change_m"]) > 0.5:
            raise RuntimeError("Surface tracking did not return after restoring range offset")
        self.step("range_out_of_range_injection")
        self.set_parameter("SIM_SONAR_POS_Z", -50)
        self.wait("distance beyond configured maximum", lambda m: m.get_type() == "DISTANCE_SENSOR" and
            m.current_distance > m.max_distance, sim_timeout=10)
        self.duration(5)
        fallback = self.range_measurements("range_out_of_range_hold", False, 10)
        fallback["altitude_change_m"] = fallback["altitude_mean_m"] - restored["altitude_mean_m"]
        fallback["true_altitude_change_m"] = fallback["true_altitude_mean_m"] - restored["true_altitude_mean_m"]
        if max(abs(fallback["altitude_min_m"] - restored["altitude_mean_m"]),
               abs(fallback["altitude_max_m"] - restored["altitude_mean_m"]),
               abs(fallback["true_altitude_min_m"] - restored["true_altitude_mean_m"]),
               abs(fallback["true_altitude_max_m"] - restored["true_altitude_mean_m"])) > 0.75:
            raise RuntimeError("Altitude was not maintained after out-of-range readings")
        self.step("range_recovery")
        self.set_parameter("SIM_SONAR_POS_Z", 0)
        self.wait("distance recovered within limits", lambda m: m.get_type() == "DISTANCE_SENSOR" and
            m.min_distance <= m.current_distance <= m.max_distance, sim_timeout=10)
        self.duration(8)
        recovered = self.range_measurements("range_recovered_hold", True, 10)
        recovered["altitude_change_m"] = recovered["altitude_mean_m"] - restored["altitude_mean_m"]
        recovered["true_altitude_change_m"] = recovered["true_altitude_mean_m"] - restored["true_altitude_mean_m"]
        if max(abs(recovered["altitude_min_m"] - restored["altitude_mean_m"]),
               abs(recovered["altitude_max_m"] - restored["altitude_mean_m"]),
               abs(recovered["true_altitude_min_m"] - restored["true_altitude_mean_m"]),
               abs(recovered["true_altitude_max_m"] - restored["true_altitude_mean_m"])) > 0.75:
            raise RuntimeError("Altitude jumped after range recovery")

    def land(self, already_rtl=False):
        self.target_ne = (0.0, 0.0)
        if not already_rtl:
            self.step("rtl_transition")
            self.set_mode("RTL")
        self.step("rtl")
        self.wait("RTL landing/disarm", lambda m: m.get_type() == "HEARTBEAT" and
            not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED and
            abs(self.latest["GLOBAL_POSITION_INT"].relative_alt) < 1000 and self.fresh("EXTENDED_SYS_STATE") and
            self.latest["EXTENDED_SYS_STATE"].landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
            sim_timeout=180, wall_timeout=90)
        rtl_rows = [r for r in self.rows if r["phase"] == "rtl"]
        expected_mode_name = "AUTO" if already_rtl else "RTL"
        expected_mode = self.link.mode_mapping()[expected_mode_name]
        if not rtl_rows:
            raise RuntimeError("RTL phase has no position telemetry")
        for row in rtl_rows:
            if row["mode"] != expected_mode or row["heartbeat_age_s"] is None or row["heartbeat_age_s"] > 2:
                raise RuntimeError(f"Return phase left {expected_mode_name} or has stale heartbeat")
            required = ("relative_alt_m", "true_lat_deg", "true_lon_deg", "true_altitude_relative_m", "true_sample_age_s")
            if any(row[k] is None or not math.isfinite(row[k]) for k in required) or row["true_sample_age_s"] > 1:
                raise RuntimeError("Return phase has missing/nonfinite/stale simulator truth")
        self.summary["metrics"]["rtl"] = {
            "samples": len(rtl_rows), "expected_mode": expected_mode_name,
            "observed_modes": sorted(set(r["mode"] for r in rtl_rows)),
            "heartbeat_max_age_s": max(r["heartbeat_age_s"] for r in rtl_rows),
            "truth_max_age_s": max(r["true_sample_age_s"] for r in rtl_rows),
        }
        self.step("landed")
        self.duration(1)
        row = self.rows[-1]
        required = ("north_m", "east_m", "true_north_m", "true_east_m", "relative_alt_m", "true_altitude_relative_m")
        if any(row[k] is None or not math.isfinite(row[k]) for k in required):
            raise RuntimeError("Missing landing truth/navigation measurements")
        if not all(self.fresh(name) for name in ("SIMSTATE", "SIM_STATE", "GLOBAL_POSITION_INT", "EXTENDED_SYS_STATE")):
            raise RuntimeError("Stale landing truth/navigation measurements")
        landing = {"estimated_home_error_m": math.hypot(row["north_m"], row["east_m"]),
                   "truth_home_error_m": math.hypot(row["true_north_m"], row["true_east_m"]),
                   "estimated_relative_alt_m": row["relative_alt_m"], "truth_relative_alt_m": row["true_altitude_relative_m"],
                   "disarmed": not bool(row["armed"]), "landed_state": row["landed_state"]}
        self.summary["metrics"]["landing"] = landing
        if row["heartbeat_age_s"] is None or row["heartbeat_age_s"] > 2 or row["armed"] or row["landed_state"] != mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            raise RuntimeError("Landing requires fresh normal disarm and on-ground state")
        if abs(landing["truth_relative_alt_m"]) >= 1 or landing["estimated_home_error_m"] > 2 or landing["truth_home_error_m"] > 2:
            raise RuntimeError("Landing position/height outside criteria")
        self.summary["metrics"]["flight"] = {"armed_sim_seconds": self.first_disarm_sim_s - self.first_arm_sim_s,
            "battery_consumed_mah": row["battery_consumed_mah"], "battery_voltage_final_v": row["battery_voltage_v"],
            "telemetry_samples": len(self.rows)}

    def fly(self):
        self.prepare()
        {"flight": self.flight_scenario, "mission": self.mission_scenario,
         "rangefinder": self.rangefinder_scenario}[self.scenario]()
        self.land(already_rtl=self.scenario == "mission")


def verify_range_log(run_dir, summary):
    """Gate success on flight-controller states, not just GCS health-bit telemetry."""
    files = sorted((run_dir / "logs").glob("*.BIN"))
    if not files:
        raise RuntimeError("Rangefinder test requires a DataFlash BIN log")
    windows = summary["metrics"]["rangefinder"]
    phases = ("range_baseline", "range_bias_response", "range_bias_restored", "range_out_of_range_hold", "range_recovered_hold")
    evidence = {phase: {"RFND": [], "SURF": [], "CTUN": []} for phase in phases}
    for file in files:
        log = mavutil.mavlink_connection(str(file))
        try:
            while True:
                message = log.recv_match(type=["RFND", "SURF", "CTUN"])
                if message is None:
                    break
                timestamp = message.TimeUS / 1e6
                for phase in phases:
                    window = windows[phase]
                    if window["start_s"] + 0.5 <= timestamp <= window["end_s"]:
                        kind = message.get_type()
                        if kind == "RFND" and message.Instance == 0:
                            evidence[phase][kind].append(int(message.Stat))
                        elif kind == "SURF" and message.I == 0:
                            evidence[phase][kind].append(int(message.St))
                        elif kind == "CTUN":
                            evidence[phase][kind].append(math.isfinite(message.DSAlt))
        finally:
            log.close()
    report = {}
    for phase, values in evidence.items():
        if any(len(v) < 5 for v in values.values()):
            raise RuntimeError(f"{phase}: missing RFND/SURF/CTUN log evidence: { {k:len(v) for k,v in values.items()} }")
        fault = phase == "range_out_of_range_hold"
        report[phase] = {"sample_counts": {k: len(v) for k, v in values.items()},
                         "RFND_Stat_values": sorted(set(values["RFND"])),
                         "SURF_St_values": sorted(set(values["SURF"])),
                         "CTUN_DSAlt_finite_fraction": sum(values["CTUN"]) / len(values["CTUN"])}
        if fault:
            good = all(v == 3 for v in values["RFND"]) and all(v & 2 for v in values["SURF"]) and not any(values["CTUN"])
        else:
            good = all(v == 4 for v in values["RFND"]) and all(v & 1 and not v & 2 for v in values["SURF"]) and all(values["CTUN"])
        if not good:
            summary["range_log_evidence"] = report
            raise RuntimeError(f"{phase}: controller range status evidence did not match expected state: {report[phase]}")
    summary["range_log_evidence"] = report


def read_params(path):
    values = {}
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            name, value = line.split()
            values[name] = float(value)
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("flight", "mission", "rangefinder"), default="flight")
    parser.add_argument("--model", type=Path, default=CONFIG / "models/nominal.json")
    parser.add_argument("--params", type=Path, default=CONFIG / "defaults.parm")
    parser.add_argument("--manifest", type=Path, default=CONFIG / "manifest.json")
    parser.add_argument("--label", default="prototype-01")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--instance", type=int, default=40)
    parser.add_argument("--speedup", type=float, default=5)
    parser.add_argument("--home", default=base.HOME)
    parser.add_argument("--wall-timeout", type=float, default=600)
    args = parser.parse_args()
    if not 40 <= args.instance <= 100 or not 0 < args.speedup <= 20 or args.wall_timeout <= 0:
        parser.error("Use instance 40..100, speedup >0..20, and a positive deadline")
    model_path, params_path, manifest_path = (p.resolve(strict=True) for p in (args.model, args.params, args.manifest))
    model, manifest = json.loads(model_path.read_text()), json.loads(manifest_path.read_text())
    cases = [case for case in manifest["cases"] if (manifest_path.parent / case["model"]).resolve() == model_path]
    if len(cases) != 1 or not math.isclose(model["mass"], cases[0]["gross_mass_kg"], abs_tol=1e-6):
        raise SystemExit("Model must match exactly one manifest case and declared gross mass")
    if any(not math.isfinite(model[k]) or model[k] <= 0 for k in ("mass", "referenceMass")):
        raise SystemExit("Model mass and calibration referenceMass must be positive")
    if not math.isclose(model["referenceMass"], manifest["reference_mass_kg"], abs_tol=1e-6):
        raise SystemExit("Model referenceMass does not match the fixed propulsion calibration")
    if base.git_head(SOURCE) != manifest["upstream_commit"]:
        raise SystemExit("ArduPilot source differs from the pinned manifest commit")
    binary = SOURCE / "build/sitl/bin/arducopter"
    if not binary.is_file():
        raise SystemExit("Run prepare-prototype.sh first")
    if args.output:
        run_dir = args.output.resolve()
        run_dir.mkdir(parents=True, exist_ok=False)
    else:
        (ROOT / "runs").mkdir(exist_ok=True)
        run_dir = Path(tempfile.mkdtemp(prefix="prototype-", dir=ROOT / "runs"))
    source_base = SOURCE / "Tools/autotest/default_params/copter.parm"
    snapshots = {"model.json": model_path, "experiment.parm": params_path, "base.parm": source_base,
                 "manifest.json": manifest_path, "runner.py": Path(__file__), "flight_support.py": Path(base.__file__)}
    for key in ("components", "mass_properties", "propulsion_parameters", "calibration", "reference_mass_patch"):
        source = (manifest_path.parent / manifest[key]).resolve(strict=True)
        snapshots[source.name] = source
    propulsion_name = Path(manifest["propulsion_parameters"]).name
    if base.sha256(snapshots[propulsion_name]) != manifest["propulsion_parameters_sha256"]:
        raise SystemExit("Propulsion calibration checksum differs from the manifest")
    if base.sha256(snapshots["reference-mass.patch"]) != manifest["patch_sha256"]:
        raise SystemExit("Propulsion patch checksum differs from the manifest")
    for name, source in snapshots.items():
        shutil.copy2(source, run_dir / name)
    metadata = {"label": args.label, "scenario": args.scenario, "case": cases[0]["label"],
                "started_utc": datetime.now(timezone.utc).isoformat(), "actual_mass_kg": model["mass"],
                "reference_mass_kg": model["referenceMass"], "model_parameters": model,
                "input_sha256": {name: base.sha256(path) for name, path in snapshots.items()},
                "binary_sha256": base.sha256(binary), "project_commit": base.git_head(ROOT),
                "ardupilot_commit": base.git_head(SOURCE), "platform": platform.platform(),
                "python": platform.python_version(), "instance": args.instance, "speedup": args.speedup, "home": args.home,
                "measurement_note": "Latest asynchronous SIMSTATE horizontal position and SIM_STATE altitude, with ages. Range backend data is not an optical TF02-Pro digital twin. Synthetic sensor-position shifts inject measurement bias/out-of-range without changing geometry or terrain."}
    for prefix, path in (("project", ROOT), ("ardupilot", SOURCE)):
        metadata[f"{prefix}_worktree_status"] = subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True).splitlines()
        metadata[f"{prefix}_tracked_diff_sha256"] = hashlib.sha256(subprocess.check_output(["git", "-C", str(path), "diff", "HEAD", "--binary"])).hexdigest()
    summary = {"label": args.label, "scenario": args.scenario, "case": cases[0]["label"], "result": "failed",
               "criteria": {"flight": FLIGHT_CRITERIA, "rangefinder": RANGE_CRITERIA, "mission": MISSION_CRITERIA},
               "completed_steps": [], "events": [], "recent_status_text": [], "metrics": {}}
    summary["criteria"]["return_mode"] = "AUTO" if args.scenario == "mission" else "RTL"
    summary["criteria"]["requires_nonempty_dataflash_log"] = True
    link = process = flight = None
    completed = False
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
        command = [str(binary), "-I", str(args.instance), "--model", "X:model.json", "--speedup", str(args.speedup),
                   "--home", args.home, "--defaults", "base.parm,experiment.parm", "--serial0", f"udpclient:127.0.0.1:{port}"]
        launch_args = manifest.get("launch_args", ["--serial5=sim:benewake_tf03"])
        if launch_args != ["--serial5=sim:benewake_tf03"]:
            raise RuntimeError(f"Unexpected rangefinder launch arguments: {launch_args}")
        command.extend(launch_args)
        metadata["command"] = command
        (run_dir / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(f"Run: {run_dir}", flush=True)
        with (run_dir / "sitl.log").open("w") as log, (run_dir / "telemetry.csv").open("w", newline="") as csv_file:
            process = subprocess.Popen(command, cwd=run_dir, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            flight = PrototypeFlight(link, process, csv_file, summary, args.wall_timeout, run_dir / "sitl.log", model,
                                     scenario=args.scenario, expected_params=read_params(params_path))
            flight.fly()
        completed = True
    except (Exception, KeyboardInterrupt) as error:
        summary["error"] = str(error) or type(error).__name__
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
        if completed:
            try:
                bin_logs = sorted((run_dir / "logs").glob("*.BIN"))
                if not bin_logs or any(p.stat().st_size == 0 for p in bin_logs):
                    raise RuntimeError("Completed flight requires nonempty DataFlash BIN logs")
                summary["log_validation"] = {"files": len(bin_logs), "total_bytes": sum(p.stat().st_size for p in bin_logs),
                                             "all_nonempty": True}
                if args.scenario == "rangefinder":
                    verify_range_log(run_dir, summary)
                summary["result"] = "passed"
            except Exception as error:
                summary["error"] = str(error) or type(error).__name__
        summary["elapsed_wall_seconds"] = round(time.monotonic() - started, 3)
        summary["finished_utc"] = datetime.now(timezone.utc).isoformat()
        summary["log_sha256"] = {str(p.relative_to(run_dir)): base.sha256(p) for p in (run_dir / "logs").glob("*.BIN")}
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        print(f"{summary['result'].upper()}: {summary.get('error', args.scenario)}; {run_dir / 'summary.json'}", flush=True)
    return 0 if summary["result"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
