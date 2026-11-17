"""Run terrain-relative field waypoints and measure AGL from simulator truth."""

import csv
import math
import statistics

from pymavlink import mavutil
from scenarios import auto
from scenarios.field.scenario import WAYPOINTS_NE_M

TARGET_AGL_M = auto.MISSION_CRITERIA["altitude_m"]


def wait_for_terrain(flight, grid_spacing_m):
    flight.step("terrain_ready")
    flight.wait(
        "synthetic terrain height at the launch pad",
        lambda message: message.get_type() == "TERRAIN_REPORT"
        and message.spacing == grid_spacing_m
        and math.isclose(message.terrain_height, flight.terrain.home_alt_m, abs_tol=0.15),
        sim_timeout=120,
    )


def run(flight):
    auto.run(flight, WAYPOINTS_NE_M, altitude_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT)


def record_agl(flight, path, max_sample_age_s, *, require_range=True, target_agl_m=TARGET_AGL_M):
    mode = flight.link.mode_mapping()["AUTO"]
    waypoint_count = len(flight.waypoints)
    truth_keys = ("true_north_m", "true_east_m", "true_alt_m", "true_sample_age_s")
    range_keys = ("distance_m", "distance_min_m", "distance_max_m", "distance_age_s", "tilt_deg", "attitude_sample_age_s")
    samples = []
    for row in flight.rows:
        seq = row["mission_seq"]
        if row["phase"] != "auto_mission" or seq is None or not 1 <= seq <= waypoint_count:
            continue
        if not samples and row["mode"] != mode:
            continue  # The first AUTO heartbeat may arrive just after the first mission item.
        if not row["armed"] or row["mode"] != mode:
            raise RuntimeError("Terrain-following flight left armed AUTO before completing its waypoints")
        if any(row[key] is None or not math.isfinite(row[key]) for key in truth_keys):
            raise RuntimeError("Missing terrain-following simulator position")
        if row["true_sample_age_s"] > max_sample_age_s:
            raise RuntimeError("Stale terrain-following simulator position")
        range_valid = (all(row[key] is not None and math.isfinite(row[key]) for key in range_keys)
                       and max(row["distance_age_s"], row["attitude_sample_age_s"]) <= max_sample_age_s
                       and row["distance_min_m"] <= row["distance_m"] <= row["distance_max_m"])
        if require_range and not range_valid:
            raise RuntimeError("Missing, stale or out-of-range measurement during terrain-following flight")

        n = row["true_north_m"]
        ground = flight.terrain.home_alt_m + flight.terrain.ground_height_m(n)
        actual = row["true_alt_m"] - ground
        range_agl = row["distance_m"] * math.cos(math.radians(row["tilt_deg"])) if range_valid else None
        samples.append({"time_s": row["time_boot_ms"] / 1000, "waypoint": seq, "north_m": n,
                        "east_m": row["true_east_m"], "ground_amsl_m": ground,
                        "aircraft_amsl_m": row["true_alt_m"], "true_agl_m": actual, "range_agl_m": range_agl,
                        "target_agl_m": target_agl_m,
                        "agl_error_m": actual - target_agl_m if target_agl_m is not None else None})

    if {sample["waypoint"] for sample in samples} != set(range(1, waypoint_count + 1)):
        raise RuntimeError("Terrain-following AGL samples did not cover every AUTO waypoint leg")
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)
    return samples


def summarize_agl(samples):
    target = TARGET_AGL_M
    agl = [sample["true_agl_m"] for sample in samples]
    ground = [sample["ground_amsl_m"] for sample in samples]
    return {
        "samples": len(samples), "target_agl_m": target, "min_agl_m": min(agl),
        "max_agl_m": max(agl), "mean_agl_m": statistics.fmean(agl),
        "max_absolute_error_m": max(abs(value - target) for value in agl),
        "ground_height_span_m": max(ground) - min(ground),
        "max_range_agreement_error_m": max(abs(sample["range_agl_m"] - sample["true_agl_m"]) for sample in samples),
    }
