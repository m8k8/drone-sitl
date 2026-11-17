"""Synthetic terrace served as ArduPilot terrain tiles."""

import math


GRID_SPACING_M = 1
STEP_HEIGHT_M = 1
_TARGET_STEP_NORTH_M = 18
_DEGREE_E7 = 10_000_000
_LAT_E7_M = 0.011131884502145034  # ArduPilot LATLON_TO_M
_TERRAIN_TILE_SIZE = 4
_TERRAIN_TILE_COLUMNS = 8
_TERRAIN_TILE_COUNT = 56


def ground_height_m(north_m, step_start_north_m, step_end_north_m, *, step_height_m=STEP_HEIGHT_M):
    """Ground height above the pad, including the grid's linear transition."""
    fraction = (north_m - step_start_north_m) / (step_end_north_m - step_start_north_m)
    return step_height_m * min(1.0, max(0.0, fraction))


class Terrain:
    def __init__(self, home, *, step_height_m=STEP_HEIGHT_M):
        if not math.isfinite(step_height_m) or step_height_m != round(step_height_m):
            raise ValueError("ArduPilot terrain tiles require step heights in whole metres")
        latitude, longitude, altitude, heading = home.split(",")
        self.latitude_int = int(float(latitude) * _DEGREE_E7)
        self.longitude_int = int(float(longitude) * _DEGREE_E7)
        self.home_alt_m = float(round(float(altitude)))
        self.home = f"{latitude},{longitude},{self.home_alt_m:.0f},{heading}"
        self.step_height_m = step_height_m

        # Place the transition between two adjacent nodes of a single degree grid.
        target_latitude_e7 = self.latitude_int + _TARGET_STEP_NORTH_M / _LAT_E7_M
        grid_degree_e7 = math.floor(target_latitude_e7 / _DEGREE_E7) * _DEGREE_E7
        target_offset_m = (target_latitude_e7 - grid_degree_e7) * _LAT_E7_M
        last_ramp_cell = int(_DEGREE_E7 * _LAT_E7_M / GRID_SPACING_M) - 1
        ramp_cell = min(round(target_offset_m / GRID_SPACING_M), last_ramp_cell)
        self.step_start_north_m = (grid_degree_e7 - self.latitude_int) * _LAT_E7_M + ramp_cell * GRID_SPACING_M
        self.step_end_north_m = self.step_start_north_m + GRID_SPACING_M

    def ground_height_m(self, north_m):
        return ground_height_m(
            north_m, self.step_start_north_m, self.step_end_north_m, step_height_m=self.step_height_m,
        )

    def reply(self, mav, message):
        """Reply to a TERRAIN_REQUEST and return the number of 4x4 tiles sent."""
        if message.get_type() != "TERRAIN_REQUEST":
            return 0
        if message.grid_spacing != GRID_SPACING_M:
            raise RuntimeError(f"Unexpected SITL terrain grid spacing {message.grid_spacing} m")

        # Snap the request's E7 latitude to the original grid node.
        south_degree_e7 = (message.lat // _DEGREE_E7) * _DEGREE_E7
        south_cells = round((message.lat - south_degree_e7) * _LAT_E7_M / GRID_SPACING_M)
        south_north_m = (south_degree_e7 - self.latitude_int) * _LAT_E7_M + south_cells * GRID_SPACING_M

        sent = 0
        for bit in range(_TERRAIN_TILE_COUNT):
            if not message.mask & (1 << bit):
                continue
            tile_south_m = south_north_m + (bit // _TERRAIN_TILE_COLUMNS) * _TERRAIN_TILE_SIZE * GRID_SPACING_M
            heights = [
                round(self.home_alt_m + self.ground_height_m(tile_south_m + row * GRID_SPACING_M))
                for row in range(_TERRAIN_TILE_SIZE)
                for _ in range(_TERRAIN_TILE_SIZE)
            ]
            mav.terrain_data_send(message.lat, message.lon, message.grid_spacing, bit, heights)
            sent += 1
        return sent
