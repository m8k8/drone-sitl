"""Measure AUTO terrain-following transients across a one-metre terrace."""

import math
from pathlib import Path

from scenarios import terrain_following
from scenarios.stepped_field.terrain import GRID_SPACING_M, STEP_HEIGHT_M, Terrain as create_terrain

PARAMS = Path(__file__).with_name("terrain.parm")
CRITERIA = {
    "target_agl_m": terrain_following.TARGET_AGL_M,
    "recovery_tolerance_m": 0.25,
    "recovery_hold_s": 1.0,
    "max_recovery_sample_gap_s": 0.25,
    "max_plateau_range_agreement_error_m": 0.5,
    "max_sample_age_s": 1.0,
}
CROSSING_LEGS = ((2, 1), (4, -1), (6, 1))


def terrain_details(terrain):
    return {"grid_spacing_m": GRID_SPACING_M, "step_height_m": terrain.step_height_m,
            "step_start_north_m": terrain.step_start_north_m,
            "step_end_north_m": terrain.step_end_north_m}


def _crossing_time(leg, boundary, direction):
    for previous, current in zip(leg, leg[1:]):
        before = direction * (previous["north_m"] - boundary)
        after = direction * (current["north_m"] - boundary)
        if before < 0 <= after:
            fraction = -before / (after - before)
            return previous["time_s"] + fraction * (current["time_s"] - previous["time_s"])
    raise RuntimeError(f"No measured terrain crossing at {boundary:.2f} m north")


def _recovery(window, exit_time):
    start = previous = None
    for sample in window:
        timestamp = sample["time_s"]
        if timestamp < exit_time:
            continue
        if abs(sample["agl_error_m"]) > CRITERIA["recovery_tolerance_m"]:
            start = previous = None
            continue
        if previous is None or timestamp - previous > CRITERIA["max_recovery_sample_gap_s"]:
            start = timestamp
        if timestamp - start >= CRITERIA["recovery_hold_s"]:
            return start, timestamp
        previous = timestamp
    return None, None


def verify(flight, path):
    samples = terrain_following.record_agl(flight, path, CRITERIA["max_sample_age_s"])
    report = terrain_following.summarize_agl(samples)
    flight.summary["metrics"]["terrain_following"] = report
    lower = flight.terrain.step_start_north_m
    upper = flight.terrain.step_end_north_m
    plateau = [sample for sample in samples
               if sample["north_m"] < lower - GRID_SPACING_M or sample["north_m"] > upper + GRID_SPACING_M]
    if not plateau or report["ground_height_span_m"] < STEP_HEIGHT_M:
        raise RuntimeError("Stepped field samples did not cover both terrain levels")
    report["max_plateau_range_agreement_error_m"] = max(
        abs(sample["range_agl_m"] - sample["true_agl_m"]) for sample in plateau)

    crossings = []
    for waypoint, direction in CROSSING_LEGS:
        leg = [sample for sample in samples if sample["waypoint"] == waypoint]
        entry, exit = (lower, upper) if direction > 0 else (upper, lower)
        crossings.append({"waypoint": waypoint, "direction": "uphill" if direction > 0 else "downhill",
                          "entry_time_s": _crossing_time(leg, entry, direction),
                          "exit_time_s": _crossing_time(leg, exit, direction)})

    report["step_crossings"] = crossings
    for index, crossing in enumerate(crossings):
        entry_time = crossing["entry_time_s"]
        next_entry_time = crossings[index + 1]["entry_time_s"] if index + 1 < len(crossings) else math.inf
        window = [sample for sample in samples if entry_time <= sample["time_s"] < next_entry_time]
        recovered_at, confirmed_at = _recovery(window, crossing["exit_time_s"])
        response = window if confirmed_at is None else [sample for sample in window if sample["time_s"] <= confirmed_at]
        crossing.update(min_agl_m=min(sample["true_agl_m"] for sample in response),
                        recovery_time_s=None if recovered_at is None else recovered_at - entry_time,
                        confirmed_at_s=confirmed_at)

    if report["max_plateau_range_agreement_error_m"] > CRITERIA["max_plateau_range_agreement_error_m"]:
        raise RuntimeError("Rangefinder readings did not agree with the two ground levels")
    if any(crossing["recovery_time_s"] is None for crossing in crossings):
        raise RuntimeError("AGL failed to recover before the next crossing or the end of the mission")
