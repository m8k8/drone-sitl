"""Clear a known ten-metre terrace with preplanned waypoint altitudes."""

from scenarios import auto, terrain_following
from scenarios.stepped_field.scenario import PARAMS, terrain_details
from scenarios.stepped_field.terrain import Terrain

STEP_HEIGHT_M = 10
CROSSING_ALTITUDE_M = 22
INSPECTION_AGL_M = 10
WAYPOINTS_NE_ALT_M = (
    (8, -8, CROSSING_ALTITUDE_M),   # Finish the climb on the lower side.
    (25, -8, CROSSING_ALTITUDE_M),  # Cross uphill with clearance.
    (30, 0, STEP_HEIGHT_M + INSPECTION_AGL_M),
    (25, 8, CROSSING_ALTITUDE_M),   # Climb before crossing downhill.
    (10, 8, CROSSING_ALTITUDE_M),   # Stay high until clear of the edge.
    (6, 8, 10),
)
CRITERIA = {
    "min_crossing_agl_m": 11.0,
    "crossing_margin_m": 2.0,
    "min_mission_agl_m": 9.0,
    "inspection_target_agl_m": INSPECTION_AGL_M,
    "inspection_max_error_m": 0.5,
    "max_sample_age_s": 1.0,
}


def create_terrain(home):
    return Terrain(home, step_height_m=STEP_HEIGHT_M)


def run(flight):
    points = tuple((north, east) for north, east, _altitude in WAYPOINTS_NE_ALT_M)
    altitudes = tuple(altitude for _north, _east, altitude in WAYPOINTS_NE_ALT_M)
    auto.run(flight, points, altitudes_m=altitudes)


def verify(flight, path):
    samples = terrain_following.record_agl(flight, path, CRITERIA["max_sample_age_s"],
                                           require_range=False, target_agl_m=None)
    minimum = min(sample["true_agl_m"] for sample in samples)
    ground_span = max(sample["ground_amsl_m"] for sample in samples) - min(sample["ground_amsl_m"] for sample in samples)
    report = {"samples": len(samples), "min_mission_agl_m": minimum, "ground_height_span_m": ground_span,
              "height_before_first_crossing_m": flight.waypoint_closest[1]["true_altitude_relative_m"],
              "crossings": []}
    flight.summary["metrics"]["planned_step_crossing"] = report
    lower = flight.terrain.step_start_north_m
    upper = flight.terrain.step_end_north_m
    margin = CRITERIA["crossing_margin_m"]
    for waypoint, direction in ((2, "uphill"), (5, "downhill")):
        window = [sample for sample in samples if sample["waypoint"] == waypoint
                  and lower - margin <= sample["north_m"] <= upper + margin]
        if (not window or min(sample["north_m"] for sample in window) >= lower
                or max(sample["north_m"] for sample in window) <= upper):
            raise RuntimeError(f"Incomplete {direction} crossing coverage")
        report["crossings"].append({"waypoint": waypoint, "direction": direction, "samples": len(window),
                                    "min_agl_m": min(sample["true_agl_m"] for sample in window)})

    inspection_time = flight.waypoint_closest[3]["time_boot_s"]
    inspection = min((sample for sample in samples if sample["waypoint"] == 3),
                     key=lambda sample: abs(sample["time_s"] - inspection_time))
    report["inspection_waypoint_agl_m"] = inspection["true_agl_m"]
    if ground_span < STEP_HEIGHT_M or minimum < CRITERIA["min_mission_agl_m"]:
        raise RuntimeError("Planned mission did not clear both ground levels")
    if any(crossing["min_agl_m"] < CRITERIA["min_crossing_agl_m"] for crossing in report["crossings"]):
        raise RuntimeError("Missed the minimum clearance at the step")
    if abs(report["inspection_waypoint_agl_m"] - INSPECTION_AGL_M) > CRITERIA["inspection_max_error_m"]:
        raise RuntimeError("Missed the planned inspection height above the upper field")
