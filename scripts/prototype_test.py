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
import subprocess
import sys
import tempfile
import time

import flight_support as base
from pymavlink import mavutil
from scenarios import SCENARIOS, auto, terrain_following
from scenarios.field import scenario as field
from scenarios.flight import scenario as flight_scenario
from scenarios.mission import scenario as mission
from scenarios.planned_step_crossing import scenario as planned_step_crossing
from scenarios.rangefinder import scenario as rangefinder
from scenarios.sloped_field import scenario as sloped_field
from scenarios.stepped_field import scenario as stepped_field

ROOT, SOURCE = base.ROOT, base.SOURCE
CONFIG = ROOT / "config/prototype-01"
SENSOR_FIELDS = ["distance_m", "distance_min_m", "distance_max_m", "distance_age_s",
                 "distance_signal_quality", "laser_health_bit", "sonar_position_z_m",
                 "mission_seq"]
base.CSV_FIELDS.extend(SENSOR_FIELDS)
TERRAIN_SCENARIOS = {
    "sloped_field": sloped_field,
    "stepped_field": stepped_field,
    "planned_step_crossing": planned_step_crossing,
}
AUTO_SCENARIOS = {"mission", "field", *TERRAIN_SCENARIOS}
SCENARIO_RUNNERS = {
    "flight": flight_scenario.run,
    "mission": mission.run,
    "rangefinder": rangefinder.run,
    "field": field.run,
    "sloped_field": terrain_following.run,
    "stepped_field": terrain_following.run,
    "planned_step_crossing": planned_step_crossing.run,
}
EXTRA_PARAMETERS = ["SERIAL5_PROTOCOL", "SERIAL5_BAUD", "RNGFND1_MIN", "RNGFND1_MAX",
                    "RNGFND1_ORIENT", "RNGFND1_GNDCLR", "SURFTRAK_MODE",
                    "SIM_SONAR_POS_Z", "SIM_SONAR_RND", "LOG_BITMASK"]


class NullWriter:
    def writerow(self, _row):
        pass


class PrototypeFlight(base.FlightSupport):
    def __init__(self, *args, scenario, expected_params, gazebo=None, terrain=None, terrain_settings=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.scenario, self.expected_params = scenario, expected_params
        self.gazebo = gazebo
        self.terrain = terrain
        self.terrain_settings = terrain_settings
        self.sonar_position_z = 0.0
        self.mission_reached = []
        self.waypoints = []
        self.waypoint_closest = {}

    def receive(self):
        message = super().receive()
        if message is None:
            return None
        if message.get_type() == "MISSION_ITEM_REACHED":
            self.mission_reached.append({"seq": message.seq, "time_boot_s": self.sim_time})
        if self.terrain is not None and message.get_type() == "TERRAIN_REQUEST":
            self.terrain.reply(self.link.mav, message)
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

    def wait_for_gazebo(self):
        print("Waiting for the Gazebo 3D view before arming…", flush=True)
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if failure := self.gazebo.failure():
                raise RuntimeError(failure)
            if self.gazebo.is_ready(require_gui=True):
                self.summary["gazebo"]["ready_before_arm"] = True
                print("Gazebo 3D view ready", flush=True)
                return
            self.receive()
        raise TimeoutError(f"Gazebo 3D view did not become ready. See {self.log_path.parent / 'gazebo-bridge.log'} and gazebo-gui.log.")

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
        if self.terrain is not None:
            for message_id in (mavutil.mavlink.MAVLINK_MSG_ID_TERRAIN_REQUEST, mavutil.mavlink.MAVLINK_MSG_ID_TERRAIN_REPORT):
                self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, 500000)
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
        if self.terrain is not None:
            for name, expected in self.terrain_settings.items():
                if not math.isclose(self.parameter(name), expected, abs_tol=1e-5):
                    raise RuntimeError(f"Unexpected terrain parameter {name}")
            terrain_following.wait_for_terrain(self, self.terrain_settings["TERRAIN_SPACING"])
        self.summary["initial_parameters"] = dict(self.parameters)
        truth, state = self.latest["SIMSTATE"], self.latest["SIM_STATE"]
        self.true_home = (truth.lat / 1e7, truth.lng / 1e7, state.alt)
        if not all(math.isfinite(v) for v in self.true_home):
            raise RuntimeError("Invalid simulator truth origin")
        self.summary["truth_home"] = dict(zip(("lat_deg", "lon_deg", "alt_m"), self.true_home))
        if self.gazebo is not None:
            self.wait_for_gazebo()
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
        self.duration(flight_scenario.FLIGHT_CRITERIA["settle_sim_seconds"])

    def hover(self, name):
        super().hover(name)
        rows = [r for r in self.rows if r["phase"] == name]
        truth = base.distribution(r["true_altitude_relative_m"] for r in rows)
        self.summary["metrics"][name].update({f"true_altitude_{key}_m": value for key, value in truth.items()})
        if truth["min"] < 9 or truth["max"] > 11:
            raise RuntimeError(f"{name}: physical model altitude outside 9..11 m")
        rangefinder.measurements(self, name, require_in_range=True)

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
        SCENARIO_RUNNERS[self.scenario](self)
        self.land(already_rtl=self.scenario in AUTO_SCENARIOS)


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
    parser.add_argument("--scenario", choices=SCENARIOS, default="flight")
    parser.add_argument("--model", type=Path, default=CONFIG / "models/nominal.json")
    parser.add_argument("--params", type=Path, default=CONFIG / "defaults.parm")
    parser.add_argument("--manifest", type=Path, default=CONFIG / "manifest.json")
    parser.add_argument("--label", default="prototype-01")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--instance", type=int, default=40)
    parser.add_argument("--speedup", type=float, help="Simulation speed multiplier (default: 1 with --gazebo, 5 otherwise)")
    parser.add_argument("--home", default=base.HOME)
    parser.add_argument("--wall-timeout", type=float, default=600)
    parser.add_argument("--gazebo", action="store_true", help="Show this automated test in Gazebo Jetty")
    args = parser.parse_args()
    if args.speedup is None:
        args.speedup = 1 if args.gazebo else 5
    if not 40 <= args.instance <= 100 or not 0 < args.speedup <= 20 or args.wall_timeout <= 0:
        parser.error("Use instance 40..100, speedup >0..20, and a positive deadline")
    terrain_case = TERRAIN_SCENARIOS.get(args.scenario)
    terrain = terrain_case.create_terrain(args.home) if terrain_case is not None else None
    launch_home = terrain.home if terrain is not None else args.home
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
    if args.gazebo:
        from gazebo_view import GazeboView

        try:
            GazeboView.require_environment()
        except RuntimeError as error:
            raise SystemExit(str(error)) from error
    if args.output:
        run_dir = args.output.resolve()
        run_dir.mkdir(parents=True, exist_ok=False)
    else:
        (ROOT / "runs").mkdir(exist_ok=True)
        run_dir = Path(tempfile.mkdtemp(prefix="prototype-", dir=ROOT / "runs"))
    source_base = SOURCE / "Tools/autotest/default_params/copter.parm"
    snapshots = {"model.json": model_path, "experiment.parm": params_path, "base.parm": source_base,
                 "manifest.json": manifest_path, "runner.py": Path(__file__), "flight_support.py": Path(base.__file__)}
    scripts = ROOT / "scripts"
    for source in sorted((scripts / "scenarios").rglob("*.py")):
        snapshots[source.relative_to(scripts).as_posix()] = source
    defaults = "base.parm,experiment.parm"
    terrain_settings = None
    if terrain_case is not None:
        terrain_params = terrain_case.PARAMS.relative_to(scripts).as_posix()
        snapshots[terrain_params] = terrain_case.PARAMS
        defaults += f",{terrain_params}"
        terrain_settings = read_params(terrain_case.PARAMS)
    if args.gazebo:
        for name in ("gazebo_view.py", "gazebo_bridge.py", "gazebo_world.py", "gazebo_pose_system.py"):
            snapshots[name] = scripts / name
    for key in ("components", "mass_properties", "propulsion_parameters", "calibration", "reference_mass_patch"):
        source = (manifest_path.parent / manifest[key]).resolve(strict=True)
        snapshots[source.name] = source
    propulsion_name = Path(manifest["propulsion_parameters"]).name
    if base.sha256(snapshots[propulsion_name]) != manifest["propulsion_parameters_sha256"]:
        raise SystemExit("Propulsion calibration checksum differs from the manifest")
    if base.sha256(snapshots["reference-mass.patch"]) != manifest["patch_sha256"]:
        raise SystemExit("Propulsion patch checksum differs from the manifest")
    for name, source in snapshots.items():
        destination = run_dir / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    metadata = {"label": args.label, "scenario": args.scenario, "case": cases[0]["label"],
                "started_utc": datetime.now(timezone.utc).isoformat(), "actual_mass_kg": model["mass"],
                "reference_mass_kg": model["referenceMass"], "model_parameters": model,
                "input_sha256": {name: base.sha256(path) for name, path in snapshots.items()},
                "binary_sha256": base.sha256(binary), "project_commit": base.git_head(ROOT),
                "ardupilot_commit": base.git_head(SOURCE), "platform": platform.platform(),
                "python": platform.python_version(), "instance": args.instance, "speedup": args.speedup, "home": launch_home,
                "measurement_note": "Latest asynchronous SIMSTATE horizontal position and SIM_STATE altitude, with ages. Range backend data is not an optical TF02-Pro digital twin. Synthetic sensor-position shifts inject measurement bias/out-of-range without changing geometry or terrain."}
    if terrain is not None:
        metadata["requested_home"] = args.home
        metadata["terrain"] = {**terrain_case.terrain_details(terrain),
                               "terrain_height_at_launch_amsl_m": terrain.home_alt_m}
    for prefix, path in (("project", ROOT), ("ardupilot", SOURCE)):
        metadata[f"{prefix}_worktree_status"] = subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True).splitlines()
        metadata[f"{prefix}_tracked_diff_sha256"] = hashlib.sha256(subprocess.check_output(["git", "-C", str(path), "diff", "HEAD", "--binary"])).hexdigest()
    mission_criteria = dict(auto.MISSION_CRITERIA)
    if args.scenario not in {"field", *TERRAIN_SCENARIOS}:
        mission_criteria["side_m"] = mission.SIDE_M
    if terrain_case is planned_step_crossing:
        mission_criteria["waypoint_altitude_tolerance_m"] = (
            mission_criteria.pop("waypoint_altitude_bounds_m")[1] - mission_criteria.pop("altitude_m"))
    summary = {"label": args.label, "scenario": args.scenario, "case": cases[0]["label"], "result": "failed",
               "criteria": {"flight": flight_scenario.FLIGHT_CRITERIA, "rangefinder": rangefinder.RANGE_CRITERIA,
                            "mission": mission_criteria},
               "completed_steps": [], "events": [], "recent_status_text": [], "metrics": {}}
    summary["criteria"]["return_mode"] = "AUTO" if args.scenario in AUTO_SCENARIOS else "RTL"
    summary["criteria"]["requires_nonempty_dataflash_log"] = True
    if terrain_case is not None:
        criteria_key = "planned_step_crossing" if terrain_case is planned_step_crossing else "terrain_following"
        summary["criteria"][criteria_key] = terrain_case.CRITERIA
    link = process = flight = gazebo = None
    completed = False
    started = time.monotonic()
    if args.gazebo:
        metadata["gazebo"] = {"enabled": True}
        summary["gazebo"] = {"ready_before_arm": False}
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
                   "--home", launch_home, "--defaults", defaults, "--serial0", f"udpclient:127.0.0.1:{port}"]
        launch_args = manifest.get("launch_args", ["--serial5=sim:benewake_tf03"])
        if launch_args != ["--serial5=sim:benewake_tf03"]:
            raise RuntimeError(f"Unexpected rangefinder launch arguments: {launch_args}")
        command.extend(launch_args)
        if args.gazebo:
            terrain_step = None
            step_height_m = 1
            if terrain_case in {stepped_field, planned_step_crossing}:
                terrain_step = (terrain.step_start_north_m, terrain.step_end_north_m)
                step_height_m = terrain.step_height_m
            gazebo = GazeboView(run_dir, model, instance=args.instance, home=launch_home,
                                field=args.scenario == "field" or terrain is not None,
                                sloped=terrain_case is sloped_field, terrain_step=terrain_step, step_height_m=step_height_m)
            command.append("--enable-fgview")
            metadata["gazebo"].update(port=gazebo.port, partition=gazebo.partition)
        metadata["command"] = command
        (run_dir / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(f"Run: {run_dir}", flush=True)
        if gazebo is not None:
            gazebo.start()
        with (run_dir / "sitl.log").open("w") as log, (run_dir / "telemetry.csv").open("w", newline="") as csv_file:
            process = subprocess.Popen(command, cwd=run_dir, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            flight = PrototypeFlight(link, process, csv_file, summary, args.wall_timeout, run_dir / "sitl.log", model,
                                     scenario=args.scenario, expected_params=read_params(params_path), gazebo=gazebo,
                                     terrain=terrain, terrain_settings=terrain_settings)
            flight.fly()
            if terrain_case is not None:
                terrain_case.verify(flight, run_dir / "terrain-following.csv")
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
        if gazebo is not None:
            if failure := gazebo.failure():
                summary["gazebo"]["exit_before_test_finished"] = failure
                if summary["gazebo"]["ready_before_arm"]:
                    print(f"Gazebo: {failure}", file=sys.stderr, flush=True)
            gazebo.close()
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
                    rangefinder.verify_range_log(run_dir, summary)
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
