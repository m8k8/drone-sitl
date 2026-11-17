"""Upload, run and verify an AUTO mission ending in RTL."""

import math
import time

import flight_support as base
from pymavlink import mavutil

MISSION_CRITERIA = {"altitude_m": 10, "waypoint_truth_error_max_m": 2,
                    "waypoint_altitude_bounds_m": [9, 11], "mission_timeout_sim_s": 240}


HOME_RELATIVE = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT


def _upload_mission(flight, points, altitude_frame, altitudes_m):
    flight.waypoints = [
        {"seq": i + 1, "north_m": n, "east_m": e, "altitude_m": altitude}
        for i, ((n, e), altitude) in enumerate(zip(points, altitudes_m, strict=True))
    ]
    items = [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0)]
    items += [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, n, e, altitude, 2)
              for (n, e), altitude in zip(points, altitudes_m, strict=True)]
    items += [(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0)]
    flight.link.mav.mission_count_send(flight.target_system, flight.target_component, len(items), 0)
    requested = set()
    start = time.monotonic()
    while time.monotonic() - start < 30:
        message = flight.receive()
        if message is None:
            continue
        if message.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            if not 0 <= message.seq < len(items):
                raise RuntimeError(f"Invalid mission request {message.seq}")
            cmd, north, east, alt, hold = items[message.seq]
            lat = flight.home[0] + math.degrees(north / base.EARTH_RADIUS_M)
            lon = flight.home[1] + math.degrees(east / (base.EARTH_RADIUS_M * math.cos(math.radians(flight.home[0]))))
            frame = altitude_frame if 1 <= message.seq <= len(points) else HOME_RELATIVE
            flight.link.mav.mission_item_int_send(flight.target_system, flight.target_component,
                message.seq, frame, cmd,
                0, 1, hold, 1 if cmd == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT else 0,
                0, math.nan if cmd == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT else 0,
                round(lat * 1e7), round(lon * 1e7), alt, 0)
            requested.add(message.seq)
        elif message.get_type() == "MISSION_ACK":
            if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED or requested != set(range(len(items))):
                raise RuntimeError(f"Mission upload rejected/incomplete: {message.type}, requested={requested}")
            flight.summary["mission"] = {"items": [dict(seq=i, command=x[0], north_m=x[1], east_m=x[2], altitude_m=x[3], hold_s=x[4]) for i, x in enumerate(items)],
                                       "upload_ack": message.type}
            return
    raise TimeoutError("Mission upload did not complete")


def start(flight, points, *, altitude_frame=HOME_RELATIVE, altitudes_m=None):
    if altitudes_m is None:
        altitudes_m = (MISSION_CRITERIA["altitude_m"],) * len(points)
    flight.hover("hover_home")
    flight.step("mission_upload")
    _upload_mission(flight, points, altitude_frame, altitudes_m)
    flight.summary["mission"]["waypoint_altitude_frame"] = altitude_frame
    flight.step("auto_mission")
    # Copter 4.7.2 rejects nonzero first/last-item parameters. This command
    # performs the AUTO transition and starts/resumes the uploaded mission.
    flight.command(mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)
    flight.wait("AUTO mission mode", lambda m: m.get_type() == "HEARTBEAT" and
        m.custom_mode == flight.link.mode_mapping()["AUTO"] and
        m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def run(flight, points, *, altitude_frame=HOME_RELATIVE, altitudes_m=None):
    start(flight, points, altitude_frame=altitude_frame, altitudes_m=altitudes_m)
    waypoint_seqs = {item["seq"] for item in flight.waypoints}
    rtl_seq = len(flight.waypoints) + 1
    flight.wait("AUTO waypoints and mission RTL item", lambda m: m.get_type() == "MISSION_CURRENT" and
        m.seq == rtl_seq and waypoint_seqs.issubset({x["seq"] for x in flight.mission_reached}),
        sim_timeout=MISSION_CRITERIA["mission_timeout_sim_s"], wall_timeout=120)
    flight.summary["mission"]["reached"] = flight.mission_reached
    flight.summary["mission"]["rtl_execution_note"] = (
        f"Mission item {rtl_seq} is NAV_RETURN_TO_LAUNCH. "
        "Copter runs the RTL submode while its flight mode remains AUTO."
    )
    flight.summary["metrics"]["mission"] = {"waypoint_closest": flight.waypoint_closest}
    alt_min, alt_max = MISSION_CRITERIA["waypoint_altitude_bounds_m"]
    for waypoint in flight.waypoints:
        seq = waypoint["seq"]
        closest = flight.waypoint_closest.get(seq)
        if closest is None:
            raise RuntimeError(f"AUTO waypoint {seq} has no true position sample")
        altitude_offset = waypoint["altitude_m"] - MISSION_CRITERIA["altitude_m"]
        minimum, maximum = alt_min + altitude_offset, alt_max + altitude_offset
        home_altitude_ok = (minimum <= closest["relative_alt_m"] <= maximum and
                            minimum <= closest["true_altitude_relative_m"] <= maximum)
        if (closest["truth_error_m"] > MISSION_CRITERIA["waypoint_truth_error_max_m"] or
                (altitude_frame == HOME_RELATIVE and not home_altitude_ok) or closest["true_sample_age_s"] > 1 or
                closest["heartbeat_age_s"] is None or closest["heartbeat_age_s"] > 2 or
                not closest["armed"] or closest["mode"] != flight.link.mode_mapping()["AUTO"]):
            raise RuntimeError(f"AUTO waypoint {seq} missed truth/altitude/mode criterion: {closest}")
