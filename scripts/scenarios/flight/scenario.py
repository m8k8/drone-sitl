"""Basic flight with a hover at home and 20 metres north."""

import math

import flight_support as base
from pymavlink import mavutil

FLIGHT_CRITERIA = {**base.CRITERIA, "settle_sim_seconds": 8.0}


def run(flight):
    flight.hover("hover_home")
    flight.step("north_transit")
    flight.set_mode("GUIDED")
    flight.target_ne = (20.0, 0.0)
    lat = flight.home[0] + math.degrees(20 / base.EARTH_RADIUS_M)
    flight.link.mav.set_position_target_global_int_send(int(flight.sim_time * 1000), flight.target_system,
        flight.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 3576,
        round(lat * 1e7), round(flight.home[1] * 1e7), 10, 0, 0, 0, 0, 0, 0, 0, 0)
    flight.wait("20 m north waypoint", lambda m: m.get_type() == "GLOBAL_POSITION_INT" and
        flight.rows[-1]["estimated_horizontal_error_m"] <= 1 and 9000 <= m.relative_alt <= 11000,
        sim_timeout=90)
    flight.set_mode("LOITER")
    flight.step("settle_north")
    flight.duration(FLIGHT_CRITERIA["settle_sim_seconds"])
    flight.hover("hover_north")
