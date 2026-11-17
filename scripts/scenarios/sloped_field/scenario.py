"""Verify the field route 10 metres above a synthetic 5% slope."""

from pathlib import Path

from scenarios import terrain_following
from scenarios.sloped_field.terrain import GRID_SPACING_M, Terrain as create_terrain

PARAMS = Path(__file__).with_name("terrain.parm")
CRITERIA = {
    "target_agl_m": terrain_following.TARGET_AGL_M,
    "max_agl_error_m": 0.5,
    "max_range_agreement_error_m": 0.5,
    "min_ground_height_span_m": 1.0,
    "max_sample_age_s": 1.0,
}


def terrain_details(_terrain):
    return {"grid_spacing_m": GRID_SPACING_M, "slope_percent": 5.0, "uphill_direction": "north"}


def verify(flight, path):
    samples = terrain_following.record_agl(flight, path, CRITERIA["max_sample_age_s"])
    report = terrain_following.summarize_agl(samples)
    flight.summary["metrics"]["terrain_following"] = report
    if report["ground_height_span_m"] < CRITERIA["min_ground_height_span_m"]:
        raise RuntimeError("Sloped field flight did not cover enough terrain height variation")
    if report["max_range_agreement_error_m"] > CRITERIA["max_range_agreement_error_m"]:
        raise RuntimeError("Rangefinder readings did not agree with the synthetic terrain height")
    if report["max_absolute_error_m"] > CRITERIA["max_agl_error_m"]:
        raise RuntimeError(f"Terrain following exceeded AGL error tolerance: {report['max_absolute_error_m']:.3f} m")
