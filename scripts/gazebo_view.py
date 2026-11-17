"""Start and stop the optional Gazebo view for one native SITL instance."""

import os
from pathlib import Path
import signal
import socket
import subprocess
import uuid

from gazebo_world import write_world


ROOT = Path(__file__).resolve().parents[1]
GAZEBO_BIN = ROOT / ".cache/gazebo/env/bin"
POSE_READY = "Gazebo pose stream ready:"
GUI_READY = "Gazebo GUI render ready"


def stop_child(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3)
    except ProcessLookupError:
        process.wait()
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=3)


class GazeboView:
    @staticmethod
    def require_environment():
        if not all(os.access(GAZEBO_BIN / name, os.X_OK) for name in ("gz", "python")):
            raise RuntimeError("先に ./scripts/setup-gazebo.sh を実行してください。")

    def __init__(self, run_dir, model, *, instance, home, field=False):
        self.run_dir = run_dir.resolve()
        self.port = 5503 + 10 * instance
        self.partition = f"bd_{uuid.uuid4().hex}"
        self._model = model
        self._home = home
        self._field = field
        self._children = []

    def start(self):
        if self._children:
            raise RuntimeError("このセッションのGazeboは既に起動しています。")
        self.require_environment()
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as pose_socket:
                try:
                    pose_socket.bind(("127.0.0.1", self.port))
                except OSError as error:
                    raise RuntimeError(f"Gazebo用のローカルUDP {self.port} を確保できません: {error}") from error
                self._start_children(pose_socket.fileno())
        except BaseException:
            self.close()
            raise

    def _start_children(self, pose_fd):
        world = self.run_dir / "gazebo.sdf"
        write_world(world, self._model, field=self._field)
        prefix = GAZEBO_BIN.parent
        environment = os.environ.copy()
        environment["PATH"] = f"{GAZEBO_BIN}{os.pathsep}{environment.get('PATH', '')}"
        scripts = str(ROOT / "scripts")
        environment["PYTHONPATH"] = os.pathsep.join(filter(None, (scripts, environment.get("PYTHONPATH"))))
        environment["GEM_HOME"] = str(prefix / "share/rubygems")
        environment["GZ_CONFIG_PATH"] = str(prefix / "share/gz")
        environment["OGRE2_RESOURCE_PATH"] = str(prefix / "lib/OGRE-Next")
        environment["GZ_PARTITION"] = self.partition
        environment["GZ_IP"] = "127.0.0.1"
        gz = str(GAZEBO_BIN / "gz")
        commands = (
            ("server", [gz, "sim", "-s", "-r", str(world)], ()),
            ("gui", [gz, "sim", "-g", "--render-engine-gui", "ogre2", "--render-engine-gui-api-backend", "metal"], ()),
            ("bridge", [str(GAZEBO_BIN / "python"), "-u", str(ROOT / "scripts/gazebo_bridge.py"),
                        "--fd", str(pose_fd), f"--home={self._home}"], (pose_fd,)),
        )
        for name, command, descriptors in commands:
            with (self.run_dir / f"gazebo-{name}.log").open("w") as output:
                child = subprocess.Popen(command, cwd=self.run_dir, env=environment, stdin=subprocess.DEVNULL,
                                         stdout=output, stderr=subprocess.STDOUT, pass_fds=descriptors,
                                         start_new_session=True)
                self._children.append((name, child))

    def is_ready(self, require_gui=False):
        log_path = self.run_dir / "gazebo-bridge.log"
        if not log_path.is_file():
            return False
        log = log_path.read_text(errors="replace")
        return POSE_READY in log and (not require_gui or GUI_READY in log)

    def failure(self):
        for name, child in self._children:
            code = child.poll()
            if code is not None:
                log_path = self.run_dir / f"gazebo-{name}.log"
                return f"Gazeboの{name}が終了しました（{code}）。{log_path} を確認してください。"
        return None

    def close(self):
        while self._children:
            _, child = self._children.pop()
            stop_child(child)
