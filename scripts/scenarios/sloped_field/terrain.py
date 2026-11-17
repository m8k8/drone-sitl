"""Synthetic 5% northward slope served as ArduPilot terrain tiles."""

GRID_SPACING_M = 20
_DEGREE_E7 = 10_000_000
_LAT_E7_M = 0.011131884502145034  # ArduPilot LATLON_TO_M
_TERRAIN_TILE_SIZE = 4
_TERRAIN_TILE_COLUMNS = 8
_TERRAIN_TILE_COUNT = 56


def ground_height_m(north_m):
    """Ground height relative to the pad in metres."""
    return north_m / GRID_SPACING_M


def _latitude_within_degree_m(latitude_e7):
    # ArduPilot's terrain grids restart at each integer degree of latitude.
    return (latitude_e7 % _DEGREE_E7) * _LAT_E7_M


class Terrain:
    ground_height_m = staticmethod(ground_height_m)

    def __init__(self, home):
        latitude, longitude, altitude, heading = home.split(",")
        self.latitude_int = int(float(latitude) * _DEGREE_E7)
        self.longitude_int = int(float(longitude) * _DEGREE_E7)
        home_grid_height = ground_height_m(_latitude_within_degree_m(self.latitude_int))
        # Keep the grid nodes on whole metres, with the launch datum at the
        # interpolated height. This adjusts only the launch altitude by <=0.5 m.
        self.home_alt_m = round(float(altitude) - home_grid_height) + home_grid_height
        self.home = f"{latitude},{longitude},{self.home_alt_m:.3f},{heading}"

    def reply(self, mav, message):
        """Reply to a TERRAIN_REQUEST and return the number of 4x4 tiles sent."""
        if message.get_type() != "TERRAIN_REQUEST":
            return 0
        if message.grid_spacing != GRID_SPACING_M:
            raise RuntimeError(f"Unexpected SITL terrain grid spacing {message.grid_spacing} m")

        # Requested block origins are grid nodes, rounded to an integer E7
        # latitude. Snap the coordinate back to the nearest grid node.
        south_cells = round(_latitude_within_degree_m(message.lat) / GRID_SPACING_M)
        south_degree_e7 = (message.lat // _DEGREE_E7) * _DEGREE_E7
        south_m = (south_degree_e7 - self.latitude_int) * _LAT_E7_M + south_cells * GRID_SPACING_M
        south_height_m = round(self.home_alt_m + ground_height_m(south_m))

        sent = 0
        for bit in range(_TERRAIN_TILE_COUNT):
            if not message.mask & (1 << bit):
                continue
            tile_south_m = south_height_m + (bit // _TERRAIN_TILE_COLUMNS) * _TERRAIN_TILE_SIZE
            heights = [
                tile_south_m + row
                for row in range(_TERRAIN_TILE_SIZE)
                for _ in range(_TERRAIN_TILE_SIZE)
            ]
            mav.terrain_data_send(message.lat, message.lon, message.grid_spacing, bit, heights)
            sent += 1
        return sent
