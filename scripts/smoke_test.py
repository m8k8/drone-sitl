#!/usr/bin/env python3
"""Run an isolated local SITL and verify one basic flight, without hardware."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

from pymavlink import mavutil


ROOT = Path(__file__).resolve().parents[1]
ARDUPILOT = ROOT / "vendor" / "ardupilot"
INSTANCE = 20
HOME_LOCATION = "-35.363261,149.165230,584,353"


class FlightCheck:
    def __init__(self, connection, process, summary):
        self.link = connection
        self.process = process
        self.summary = summary
        self.messages = {}
        self.started = time.monotonic()
        self.last_send = 0.0
        self.throttle = 1000
        self.target_system = None
        self.target_component = None

    def receive(self):
        if self.process.poll() is not None:
            raise RuntimeError(f"SITL exited with code {self.process.returncode}")
        if time.monotonic() - self.started > 240:
            raise TimeoutError("Flight check exceeded its 240 second deadline")
        now = time.monotonic()
        if self.target_system is not None and now - self.last_send >= 0.2:
            self.link.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.link.mav.rc_channels_override_send(
                self.target_system, self.target_component,
                1500, 1500, self.throttle, 1500,
                65535, 65535, 65535, 65535,
            )
            self.last_send = now
        message = self.link.recv_match(blocking=True, timeout=0.1)
        if message is None or message.get_type() == "BAD_DATA":
            return None
        if self.target_system is not None and message.get_srcSystem() != self.target_system:
            return None
        self.messages[message.get_type()] = message
        if message.get_type() == "STATUSTEXT":
            text = str(message.text).rstrip("\x00")
            self.summary["recent_status_text"].append(text)
            self.summary["recent_status_text"] = self.summary["recent_status_text"][-20:]
            print(f"  SITL: {text}", flush=True)
        return message

    def wait(self, description, predicate, timeout=40):
        print(description, flush=True)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.receive()
            if message is not None and predicate(message):
                self.summary["completed_steps"].append(description)
                return message
        raise TimeoutError(f"Timed out: {description}")

    def command(self, command, *params):
        self.link.mav.command_long_send(
            self.target_system, self.target_component,
            command, 0, *(list(params) + [0] * (7 - len(params))),
        )
        ack = self.wait(
            f"  Command acknowledgement: {command}",
            lambda m: m.get_type() == "COMMAND_ACK" and m.command == command,
            timeout=15,
        )
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"Command {command} rejected, MAV_RESULT={ack.result}")

    def mode(self, name):
        modes = self.link.mode_mapping()
        if not modes or name not in modes:
            raise RuntimeError(f"Mode unavailable: {name}")
        wanted = modes[name]
        self.link.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            wanted,
        )
        self.wait(
            f"Mode: {name}",
            lambda m: m.get_type() == "HEARTBEAT" and m.custom_mode == wanted,
        )

    def ready(self, message):
        gps = self.messages.get("GPS_RAW_INT")
        ekf = self.messages.get("EKF_STATUS_REPORT")
        status = self.messages.get("SYS_STATUS")
        required = (mavutil.mavlink.EKF_ATTITUDE |
                    mavutil.mavlink.EKF_VELOCITY_HORIZ |
                    mavutil.mavlink.EKF_POS_HORIZ_ABS |
                    mavutil.mavlink.EKF_POS_VERT_ABS)
        prearm = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        return (
            gps is not None and gps.fix_type >= 3 and
            ekf is not None and ekf.flags & required == required and
            not ekf.flags & mavutil.mavlink.EKF_CONST_POS_MODE and
            status is not None and status.onboard_control_sensors_health & prearm
        )

    def fly(self):
        heartbeat = self.wait(
            "Waiting for local SITL heartbeat",
            lambda m: m.get_type() == "HEARTBEAT" and
            m.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            timeout=30,
        )
        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        self.link.target_system = self.target_system
        self.link.target_component = self.target_component
        self.link.mav.request_data_stream_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
        )
        self.wait("Waiting for GPS, EKF and normal pre-arm checks", self.ready, timeout=60)
        self.mode("GUIDED")
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        self.wait(
            "Motors armed without forcing or disabling checks",
            lambda m: m.get_type() == "HEARTBEAT" and
            m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        )
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10)
        position = self.wait(
            "Taking off to 10 m",
            lambda m: m.get_type() == "GLOBAL_POSITION_INT" and
            9000 <= m.relative_alt <= 11000,
            timeout=45,
        )
        self.summary["takeoff_altitude_m"] = position.relative_alt / 1000
        self.throttle = 1500
        self.mode("LOITER")
        print("Checking LOITER altitude and position for 3 wall-clock seconds", flush=True)
        hold_start = time.monotonic()
        samples = 0
        max_drift = 0.0
        while time.monotonic() - hold_start < 3:
            message = self.receive()
            if message is None or message.get_type() != "GLOBAL_POSITION_INT":
                continue
            samples += 1
            altitude = message.relative_alt / 1000
            north = (message.lat - position.lat) / 1e7 * 111320
            east = (message.lon - position.lon) / 1e7 * 111320 * math.cos(math.radians(position.lat / 1e7))
            max_drift = max(max_drift, math.hypot(north, east))
            if not 8.5 <= altitude <= 11.5 or max_drift > 3:
                raise RuntimeError(f"LOITER unstable: altitude={altitude:.2f} m, drift={max_drift:.2f} m")
        if samples < 3:
            raise RuntimeError("Not enough position samples to verify LOITER")
        self.summary["completed_steps"].append("LOITER altitude and position maintained")
        self.summary["loiter_max_drift_m"] = round(max_drift, 3)
        self.mode("RTL")
        self.wait(
            "Waiting for RTL landing and automatic disarm",
            lambda m: m.get_type() == "HEARTBEAT" and
            not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED and
            "GLOBAL_POSITION_INT" in self.messages and
            abs(self.messages["GLOBAL_POSITION_INT"].relative_alt) < 1000,
            timeout=70,
        )


def main():
    binary = ARDUPILOT / "build" / "sitl" / "bin" / "arducopter"
    defaults = ARDUPILOT / "Tools" / "autotest" / "default_params" / "copter.parm"
    if not binary.is_file():
        print("SITL binary not found. Run ./scripts/build.sh first.", file=sys.stderr)
        return 1
    runs = ROOT / "runs"
    runs.mkdir(exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix="smoke-", dir=runs))
    summary = {"result": "failed", "completed_steps": [], "recent_status_text": []}
    process = None
    connection = None
    start = time.monotonic()
    try:
        # Allocate our own loopback endpoint before starting SITL. No existing
        # TCP simulator or external aircraft is used by this test.
        connection = mavutil.mavlink_connection(
            "udpin:127.0.0.1:0", source_system=255, source_component=190,
        )
        port = connection.port.getsockname()[1]
        summary["udp_port"] = port
        # Refuse a collision on the independent instance's default TCP ports.
        reservations = []
        try:
            for tcp_port in (5960, 5962, 5963):
                reservation = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                reservations.append(reservation)
                reservation.bind(("127.0.0.1", tcp_port))
        finally:
            for reservation in reservations:
                reservation.close()
        command = [
            str(binary), "-I", str(INSTANCE), "--model", "+", "--speedup", "5",
            "--home", HOME_LOCATION, "--defaults", str(defaults),
            "--serial0", f"udpclient:127.0.0.1:{port}",
        ]
        print(f"Logs: {run_dir}", flush=True)
        with (run_dir / "sitl.log").open("w") as log:
            process = subprocess.Popen(
                command, cwd=run_dir, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            FlightCheck(connection, process, summary).fly()
        summary["result"] = "passed"
        print("PASS: heartbeat, pre-arm checks, 10 m takeoff, LOITER, RTL and disarm", flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        summary["error"] = str(error) or type(error).__name__
        print(f"FAIL: {summary['error']}", file=sys.stderr, flush=True)
        return 1
    finally:
        if connection is not None:
            connection.close()
        if process is not None and process.poll() is None:
            # Only the process group this test created is stopped.
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        summary["elapsed_seconds"] = round(time.monotonic() - start, 2)
        (run_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
        print(f"Result: {run_dir / 'summary.json'}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
