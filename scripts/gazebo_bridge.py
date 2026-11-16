#!/usr/bin/env python3
"""Send native SITL ground-truth poses to the Gazebo visualization."""

import argparse
import math
import select
import socket
import struct
import sys
from threading import Event
import time

from gazebo_world import MODEL_NAME, WORLD_NAME

LOOPBACK = "127.0.0.1"
POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"
FDM_SIZE = 408
FDM_VERSION = 24
FDM_HEADER = struct.Struct(">I4x3d4f")
EARTH_RADIUS_M = 6378137.0
POSE_INTERVAL_S = 1 / 30
RETRY_INTERVAL_S = 0.25
REQUEST_TIMEOUT_MS = 200


def parse_home(text):
    """Read the launcher home format (lat,lon,alt,heading)."""
    try:
        latitude, longitude, altitude, heading = map(float, text.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected home as lat,lon,alt,heading") from error
    if (not all(map(math.isfinite, (latitude, longitude, altitude, heading)))
            or abs(latitude) > 90 or abs(longitude) > 180):
        raise argparse.ArgumentTypeError("invalid home coordinates")
    return math.radians(longitude), math.radians(latitude), altitude


def decode_fdm(packet):
    """Read the position and Euler attitude from a FlightGear v24 datagram."""
    if len(packet) != FDM_SIZE:
        return None
    version, longitude, latitude, altitude, _, roll, pitch, heading = FDM_HEADER.unpack_from(packet)
    state = longitude, latitude, altitude, roll, pitch, heading
    if version != FDM_VERSION or not all(map(math.isfinite, state)):
        return None
    if abs(longitude) > math.pi or abs(latitude) > math.pi / 2:
        return None
    return state


def gazebo_pose(home, state):
    """Transform NED / forward-right-down attitude to ENU / forward-left-up."""
    home_longitude, home_latitude, home_altitude = home
    longitude, latitude, altitude, roll, pitch, heading = state
    east = EARTH_RADIUS_M * math.cos(home_latitude) * math.remainder(longitude - home_longitude, math.tau)
    north = EARTH_RADIUS_M * (latitude - home_latitude)
    up = altitude - home_altitude

    half_roll, half_pitch, half_yaw = roll / 2, -pitch / 2, (math.pi / 2 - heading) / 2
    sr, cr = math.sin(half_roll), math.cos(half_roll)
    sp, cp = math.sin(half_pitch), math.cos(half_pitch)
    sy, cy = math.sin(half_yaw), math.cos(half_yaw)
    orientation = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    return (east, north, up), orientation


def latest_state(source):
    latest = None
    # Read one extra byte to reject oversized datagrams as well as short ones.
    for _ in range(256):
        try:
            packet, sender = source.recvfrom(FDM_SIZE + 1)
        except BlockingIOError:
            break
        if sender[0] == LOOPBACK:
            state = decode_fdm(packet)
            if state is not None:
                latest = state
    return latest


def stream_poses(source, home):
    from gz.msgs.boolean_pb2 import Boolean
    from gz.msgs.pose_pb2 import Pose
    from gz.transport import Node

    node = Node()
    rendered = Event()
    if not node.subscribe(Pose, "/gui/camera/pose", lambda _pose: rendered.set()):
        raise RuntimeError("could not subscribe to the Gazebo camera pose")
    reported_render = False
    pose = Pose(name=MODEL_NAME)
    source.setblocking(False)
    next_attempt = 0.0
    server_ready = None
    while True:
        delay = next_attempt - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        select.select([source], [], [])
        state = latest_state(source)
        if state is None:
            continue

        if not reported_render and rendered.is_set():
            print("Gazebo GUI render ready", flush=True)
            reported_render = True
        position, orientation = gazebo_pose(home, state)
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
        attempted = time.monotonic()
        success, response = node.request(POSE_SERVICE, pose, Pose, Boolean, REQUEST_TIMEOUT_MS)
        ready = bool(success and response.data)
        if ready != server_ready:
            if ready:
                print(f"Gazebo pose stream ready: {WORLD_NAME}/{MODEL_NAME}", flush=True)
            else:
                print(f"Gazebo pose service unavailable: {POSE_SERVICE}. Retrying.", file=sys.stderr, flush=True)
            server_ready = ready
        next_attempt = attempted + POSE_INTERVAL_S if ready else time.monotonic() + RETRY_INTERVAL_S


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fd", type=int, required=True, help="inherited loopback UDP socket descriptor")
    parser.add_argument("--home", type=parse_home, required=True, help="reference home as lat,lon,alt,heading")
    args = parser.parse_args()
    try:
        with socket.socket(fileno=args.fd) as source:
            if (source.family != socket.AF_INET or source.type != socket.SOCK_DGRAM
                    or source.getsockname()[0] != LOOPBACK or not source.getsockname()[1]):
                raise ValueError("expected a UDP socket bound to IPv4 loopback")
            stream_poses(source, args.home)
    except KeyboardInterrupt:
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(f"Gazebo pose bridge: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
