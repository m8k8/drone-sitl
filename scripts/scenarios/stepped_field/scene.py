"""Display the lower field, interpolated terrace edge and raised field."""

import math

from scenarios.field import scene as field_scene
from scenarios.stepped_field.terrain import STEP_HEIGHT_M, ground_height_m


TALL_TERRACE_CAMERA_POSE = "-37 -33 38 0 0.405 0.914"


def _strip(link, add_visual, name, *, east, width, north_bounds, elevations, thickness, color):
    south, north = north_bounds
    low, high = elevations
    run = north - south
    rise = high - low
    roll = math.atan2(rise, run)
    # Box poses refer to the centre, so offset the solid down from its top face.
    pose = (east, (south + north) / 2 + math.sin(roll) * thickness / 2,
            (low + high) / 2 - math.cos(roll) * thickness / 2, roll, 0, 0)
    add_visual(link, name, pose, "box", {"size": f"{width} {math.hypot(run, rise):.9g} {thickness}"}, color)


def add_terraced_field(link, add_visual, terrain_step, *, step_height_m=STEP_HEIGHT_M):
    step_start, step_end = terrain_step
    _strip(link, add_visual, "terrace_edge", east=5, width=120,
           north_bounds=terrain_step, elevations=(0, step_height_m),
           thickness=step_height_m + 0.08, color="0.36 0.28 0.20 1")
    _strip(link, add_visual, "upper_field", east=5, width=120,
           north_bounds=(step_end, 65), elevations=(step_height_m, step_height_m),
           thickness=step_height_m + 0.08, color="0.32 0.38 0.35 1")

    def height_at(north):
        return ground_height_m(north, *terrain_step, step_height_m=step_height_m)

    def add_crop_visual(link, name, pose, shape, dimensions, color):
        east, north, z, roll, pitch, yaw = pose
        if shape == "ellipsoid":
            lifted = (east, north, z + height_at(north), roll, pitch, yaw)
            add_visual(link, name, lifted, shape, dimensions, color)
            return

        width, length, thickness = map(float, dimensions["size"].split())
        south_edge, north_edge = north - length / 2, north + length / 2
        sections = (("lower", south_edge, min(north_edge, step_start)),
                    ("edge", max(south_edge, step_start), min(north_edge, step_end)),
                    ("upper", max(south_edge, step_end), north_edge))
        top = z + thickness / 2
        for section, south, north in sections:
            if south < north:
                elevations = (top + height_at(south), top + height_at(north))
                _strip(link, add_visual, f"{name}_{section}", east=east, width=width,
                       north_bounds=(south, north), elevations=elevations, thickness=thickness, color=color)

    field_scene.add_crop_rows(link, add_crop_visual)
