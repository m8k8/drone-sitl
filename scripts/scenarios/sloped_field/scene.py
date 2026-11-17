"""Display geometry for the sloped field scenario."""

import math

from scenarios.sloped_field.terrain import ground_height_m


# Gazebo uses north for +Y, so rolling the ground link around +X pitches its
# northern edge up. Its origin stays at ground level at the launch point.
GROUND_POSE = (0, 0, ground_height_m(0), math.atan2(ground_height_m(1) - ground_height_m(0), 1), 0, 0)
