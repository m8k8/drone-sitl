"""Fly the 20 m square mission and return to the launch pad."""

from scenarios import auto


SIDE_M = 20
WAYPOINTS_NE_M = ((SIDE_M, 0), (SIDE_M, SIDE_M), (0, SIDE_M), (0, 0))


def run(flight):
    auto.run(flight, WAYPOINTS_NE_M)
