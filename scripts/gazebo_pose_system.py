"""Apply the latest SITL display pose on Gazebo's simulation thread."""

from threading import Lock

from gz.math import Pose3d, Quaterniond, Vector3d
from gz.msgs.pose_pb2 import Pose
from gz.sim import Model
from gz.transport import Node

from gazebo_world import POSE_TOPIC


class SitlPoseSystem:
    def __init__(self):
        self._lock = Lock()
        self._pending_pose = None

    def configure(self, entity, _sdf, _ecm, _event_manager):
        self._model = Model(entity)
        self._node = Node()
        if not self._node.subscribe(Pose, POSE_TOPIC, self._receive_pose):
            raise RuntimeError(f"Could not subscribe to the SITL pose topic: {POSE_TOPIC}")

    def _receive_pose(self, pose):
        with self._lock:
            self._pending_pose = pose

    def pre_update(self, info, ecm):
        if info.paused:
            return
        with self._lock:
            pose = self._pending_pose
            self._pending_pose = None
        if pose is None:
            return

        position = pose.position
        orientation = pose.orientation
        self._model.set_world_pose_cmd(
            ecm,
            Pose3d(
                Vector3d(position.x, position.y, position.z),
                Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z),
            ),
        )


def get_system():
    return SitlPoseSystem()
