"""Fly parallel passes over the flat field and return to the launch pad."""

from scenarios import auto


WAYPOINTS_NE_M = (
    (6, -8), (30, -8),
    (30, 0), (6, 0),
    (6, 8), (30, 8),
)


def run(flight):
    auto.run(flight, WAYPOINTS_NE_M)
