"""Exercise the native battery failsafe on the field AUTO route."""

import math
from pathlib import Path
import re

from pymavlink import mavutil

from scenarios import auto
from scenarios.field.scenario import WAYPOINTS_NE_M


PARAMS = Path(__file__).with_name("battery.parm")
CRITERIA = {
    "low_threshold_v": 42.0,
    "critical_threshold_v": 40.8,
    "voltage_hold_s": 2.0,
    "low_injected_voltage_v": 43.2,
    "critical_injected_voltage_v": 41.2,
    "low_injection_waypoint": 2,
    "low_injection_min_home_distance_m": 24.0,
    "critical_injection_min_home_distance_m": 15.0,
    "critical_injection_min_homeward_speed_mps": 0.5,
}


def _armed_in(flight, mode):
    heartbeat = flight.latest.get("HEARTBEAT")
    return (
        heartbeat is not None
        and flight.sim_time - flight.seen["HEARTBEAT"] <= 2
        and heartbeat.custom_mode == flight.link.mode_mapping()[mode]
        and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def _position(flight):
    if not flight.rows:
        return None
    row = flight.rows[-1]
    values = (row["true_north_m"], row["true_east_m"], row["true_altitude_relative_m"], row["true_sample_age_s"])
    if any(value is None or not math.isfinite(value) for value in values) or values[-1] > 1:
        return None
    north, east, altitude, _age = values
    return {
        "time_boot_s": flight.sim_time,
        "north_m": north,
        "east_m": east,
        "altitude_m": altitude,
        "home_distance_m": math.hypot(north, east),
    }


def _battery_voltage(flight):
    if not flight.fresh("SYS_STATUS"):
        return None
    mv = flight.latest["SYS_STATUS"].voltage_battery
    return mv / 1000 if 0 < mv < 65535 else None


def _ready_for_low(flight, message):
    if message.get_type() != "GLOBAL_POSITION_INT" or not _armed_in(flight, "AUTO"):
        return False
    mission = flight.latest.get("MISSION_CURRENT")
    position, voltage = _position(flight), _battery_voltage(flight)
    return (
        mission is not None and 1 <= mission.seq <= len(WAYPOINTS_NE_M)
        and position is not None and position["home_distance_m"] >= CRITERIA["low_injection_min_home_distance_m"]
        and voltage is not None and voltage > CRITERIA["low_threshold_v"]
        and any(event["seq"] == CRITERIA["low_injection_waypoint"] for event in flight.mission_reached)
    )


def _homeward_speed(message, position):
    return -(message.vx * position["north_m"] + message.vy * position["east_m"]) / (100 * position["home_distance_m"])


def _ready_for_critical(flight, message):
    if message.get_type() != "GLOBAL_POSITION_INT" or not _armed_in(flight, "RTL"):
        return False
    position = _position(flight)
    return (
        position is not None and position["home_distance_m"] >= CRITERIA["critical_injection_min_home_distance_m"]
        and _homeward_speed(message, position) >= CRITERIA["critical_injection_min_homeward_speed_mps"]
    )


def _exercise_threshold(flight, result, level, source_mode, expected_mode):
    warning = re.compile(rf"Battery 1 is {level} ([\d.]+)V used (\d+) mAh")

    def latest_warning():
        return next((match for text in reversed(flight.summary["recent_status_text"])
                     if (match := warning.fullmatch(text))), None)

    if latest_warning() is not None:
        raise RuntimeError(f"The {level} battery warning arrived before its voltage injection")
    injected_voltage = CRITERIA[f"{level}_injected_voltage_v"]
    injection = _position(flight)
    voltage = _battery_voltage(flight)
    if injection is None or voltage is None or not _armed_in(flight, source_mode):
        raise RuntimeError(f"Missing armed {source_mode} position or voltage before the {level} injection")
    result["injection"] = dict(injection, from_mode=source_mode, loaded_voltage_v=voltage,
                               requested_open_circuit_voltage_v=injected_voltage)
    flight.step(f"battery_{level}_voltage")
    flight.set_parameter("SIM_BATT_VOLTAGE", injected_voltage)

    flight.wait(
        f"{level} battery warning and armed {expected_mode}",
        lambda message: message.get_type() == "HEARTBEAT" and _armed_in(flight, expected_mode)
        and latest_warning() is not None,
        sim_timeout=15,
    )
    transition = _position(flight)
    if transition is None:
        raise RuntimeError(f"Missing position at the {expected_mode} heartbeat")
    match = latest_warning()
    warning_voltage, consumed = float(match[1]), int(match[2])
    remaining = flight.parameters["BATT_CAPACITY"] - consumed
    result["transition"] = dict(
        transition, mode=expected_mode, warning_voltage_v=warning_voltage,
        consumed_mah=consumed, remaining_mah=remaining, status_text=match[0],
    )
    if warning_voltage >= CRITERIA[f"{level}_threshold_v"] or remaining <= flight.parameters["BATT_LOW_MAH"]:
        raise RuntimeError(f"The {level} battery action was not isolated to low voltage")
    if level == "low" and warning_voltage < CRITERIA["critical_threshold_v"]:
        raise RuntimeError("The first voltage injection also crossed the critical threshold")
    flight.step(f"battery_failsafe_{expected_mode.lower()}")
    return transition


def run(flight, *, critical):
    expected_parameters = {
        "BATT_FS_LOW_ACT": 2, "BATT_FS_CRT_ACT": 1, "BATT_LOW_TIMER": CRITERIA["voltage_hold_s"], "BATT_FS_VOLTSRC": 0,
        "BATT_LOW_VOLT": CRITERIA["low_threshold_v"], "BATT_CRT_VOLT": CRITERIA["critical_threshold_v"],
    }
    for name, expected in expected_parameters.items():
        if not math.isclose(flight.parameter(name), expected, abs_tol=1e-5):
            raise RuntimeError(f"Unexpected battery failsafe parameter {name}")
    flight.parameter("BATT_LOW_MAH")

    report = {"critical_case": critical, "low": {}}
    flight.summary["metrics"]["battery_failsafe"] = report
    auto.start(flight, WAYPOINTS_NE_M)
    flight.wait("field waypoint 2 reached in armed AUTO with healthy voltage",
                lambda message: _ready_for_low(flight, message), sim_timeout=90)
    flight.summary["mission"]["reached"] = flight.mission_reached
    _exercise_threshold(flight, report["low"], "low", "AUTO", "RTL")
    if not critical:
        return None

    message = flight.wait("moving homeward in armed RTL while clear of home",
                          lambda message: _ready_for_critical(flight, message), sim_timeout=20)
    report["critical"] = {"homeward_speed_mps": _homeward_speed(message, _position(flight))}
    transition = _exercise_threshold(flight, report["critical"], "critical", "RTL", "LAND")
    return transition["north_m"], transition["east_m"]
