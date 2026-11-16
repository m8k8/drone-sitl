#!/usr/bin/env python3
"""Start nominal B-D SITL and interactive MAVProxy without arming automatically."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
SOURCE, CONFIG = ROOT / "vendor/ardupilot", ROOT / "config/prototype-01"
RUN = ROOT / "runs/prototype-manual"
INSTANCE = 41
HOME_POSITION = "-35.363261,149.165230,584,353"
TCP_PORTS, RC_PORT = (6170, 6172, 6173), 5911
MODULES = "log,wp,fence,param,arm,mode,rc,cmdlong,battery,output"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")


def preflight():
    binary, proxy = SOURCE / "build/sitl/bin/arducopter", ROOT / ".venv/bin/mavproxy.py"
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise RuntimeError("先に ./scripts/prepare-prototype.sh を実行してください。")
    if not proxy.is_file():
        raise RuntimeError("先に ./scripts/setup.sh を実行してください。")
    lsof = shutil.which("lsof")
    if not lsof:
        raise RuntimeError("接続先SITLの所有者を確認するため、lsofが必要です。")
    # MAVProxy automatically executes these scripts, possibly adding commands or links.
    for path in (Path.home() / ".mavinit.scr", Path.home() / ".mavproxy/mavinit.scr"):
        if path.exists():
            raise RuntimeError(f"MAVProxyの自動起動スクリプトがあります: {path}。自動コマンドを避けるため停止しました。このファイルは変更していません。")
    subprocess.run([sys.executable, str(CONFIG / "generate.py"), "--check"], check=True)
    manifest = json.loads((CONFIG / "manifest.json").read_text())
    upstream = subprocess.check_output(["git", "-C", str(SOURCE), "rev-parse", "HEAD"], text=True).strip()
    if upstream != manifest["upstream_commit"]:
        raise RuntimeError("ArduPilotが固定版と異なります。setup.shの環境を確認してください。")
    for name, hashes in manifest["patch_files"].items():
        if digest(SOURCE / name) != hashes["patched_sha256"]:
            raise RuntimeError("モデル拡張が未準備です。./scripts/prepare-prototype.sh を実行してください。")
    model_path = CONFIG / "models/nominal.json"
    model = json.loads(model_path.read_text())
    if not math.isclose(model["mass"], 12.3387, abs_tol=1e-7):
        raise RuntimeError("手動起動は第１弾B〜Dのnominal 12.3387kgモデル専用です。")
    if manifest["launch_args"] != ["--serial5=sim:benewake_tf03"]:
        raise RuntimeError("想定と異なる距離センサーの起動設定です。")
    inputs = {"model.json": model_path, "base.parm": SOURCE / "Tools/autotest/default_params/copter.parm",
              "experiment.parm": CONFIG / "defaults.parm"}
    return binary, proxy, lsof, model, manifest, inputs


def reserve_ports():
    reservations = []
    try:
        for port in TCP_PORTS:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            reservations.append(sock)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", port))
            sock.listen(1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        reservations.append(sock)
        sock.bind(("0.0.0.0", RC_PORT))
    except OSError as error:
        for sock in reservations:
            sock.close()
        raise RuntimeError(f"専用ポートTCP6170/6172/6173またはUDP5911が使用中・使用不可です。既存プロセスには接続せず停止します: {error}") from error
    return reservations


def check_runtime_model(log_path, expected):
    marker = re.search(r"G06 frame mass=([0-9.]+) referenceMass=([0-9.]+)", log_path.read_text(errors="replace"))
    if marker is None:
        return False
    mass, reference = map(float, marker.groups())
    if not math.isclose(mass, expected["mass"], abs_tol=1e-5) or not math.isclose(reference, expected["referenceMass"], abs_tol=1e-5):
        raise RuntimeError("実行中の質量・推進校正値が入力モデルと一致しません。")
    return True


def wait_for_own_listener(process, lsof, log_path, model):
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"ArduCopterが接続前に終了しました（{process.returncode}）。{log_path}を確認してください。")
        check_runtime_model(log_path, model)
        result = subprocess.run([lsof, "-nP", "-a", "-p", str(process.pid), "-iTCP:6170", "-sTCP:LISTEN", "-t"],
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3)
        if result.returncode == 0 and str(process.pid) in result.stdout.split():
            return
        time.sleep(0.1)
    raise RuntimeError("起動したArduCopter自身のTCP6170待受を確認できません。MAVProxyは接続しません。")


def stop_child(process):
    if process is None or process.poll() is not None:
        return
    # Both children own separate sessions/groups; no other launcher is in these groups.
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


def interrupted(_signum, _frame):
    raise KeyboardInterrupt


def launch(args):
    binary, proxy_path, lsof, model, manifest, inputs = preflight()
    reservations = reserve_ports()
    arducopter = proxy = None
    metadata = record_path = log_stream = lock_stream = None
    exit_code = 1
    try:
        RUN.mkdir(parents=True, exist_ok=True)
        lock_stream = (RUN / ".launcher.lock").open("a")
        try:
            fcntl.flock(lock_stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("第１弾の手動起動が既に動いています。") from error
        fingerprint = {"input_sha256": {name: digest(path) for name, path in inputs.items()},
                       "upstream_commit": manifest["upstream_commit"], "instance": INSTANCE,
                       "home": HOME_POSITION, "serial5": "sim:benewake_tf03"}
        eeprom, saved = RUN / "eeprom.bin", RUN / "configuration.json"
        eeprom_existed = eeprom.exists()
        if eeprom_existed and not args.reset:
            previous = json.loads(saved.read_text()) if saved.exists() else None
            if previous != fingerprint:
                raise RuntimeError("保存パラメータとモデル・既定設定の版が一致しません。./scripts/run.sh --reset で保存パラメータを初期化してください。ログは保持されます。")
        if args.reset:
            eeprom.unlink(missing_ok=True)
        for name, path in inputs.items():
            shutil.copy2(path, RUN / name)
        write_json(saved, fingerprint)
        sessions = RUN / "sessions"
        sessions.mkdir(exist_ok=True)
        session = Path(tempfile.mkdtemp(prefix=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-"), dir=sessions))
        for name in inputs:
            shutil.copy2(RUN / name, session / name)
        record_path, log_path = session / "launch.json", session / "sitl.log"
        command = [str(binary), "-I", str(INSTANCE), "--model", "X:model.json", "--speedup", "1",
                   "--home", HOME_POSITION, "--defaults", "base.parm,experiment.parm", "--serial5=sim:benewake_tf03"]
        proxy_command = [sys.executable, str(proxy_path), "--retries", "5", "--master", "tcp:127.0.0.1:6170",
                         "--sitl", "127.0.0.1:5911", "--out", "127.0.0.1:14550", "--default-modules", MODULES,
                         "--logfile", str(session.relative_to(RUN) / "mav.tlog")]
        metadata = {"started_utc": datetime.now(timezone.utc).isoformat(), "model": "prototype-01/nominal",
                    "mass_kg": model["mass"], "reference_mass_kg": model["referenceMass"], **fingerprint,
                    "reset_requested": args.reset, "eeprom_existed": eeprom_existed,
                    "binary_sha256": digest(binary), "manifest_sha256": digest(CONFIG / "manifest.json"),
                    "project_commit": subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
                    "arducopter_command": command, "mavproxy_command": proxy_command,
                    "automatic_arm": False, "state": "starting"}
        write_json(record_path, metadata)
        print("第１弾B〜D / 12.3387kg / CMACで手動起動します。自動アームはしません。", flush=True)
        print("終了: Ctrl+C。保存状態: runs/prototype-manual、機体BIN: runs/prototype-manual/logs", flush=True)
        print(f"今回の通信・起動ログ: {session.relative_to(ROOT)}", flush=True)
        for sock in reservations:
            sock.close()
        reservations = []
        log_stream = log_path.open("w")
        arducopter = subprocess.Popen(command, cwd=RUN, stdin=subprocess.DEVNULL, stdout=log_stream,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        metadata["arducopter_pid"] = arducopter.pid
        write_json(record_path, metadata)
        wait_for_own_listener(arducopter, lsof, log_path, model)
        proxy = subprocess.Popen(proxy_command, cwd=RUN, start_new_session=True)
        metadata.update(mavproxy_pid=proxy.pid, state="running")
        write_json(record_path, metadata)
        model_checked, model_deadline = False, time.monotonic() + 20
        while True:
            if arducopter.poll() is not None:
                raise RuntimeError(f"ArduCopterが終了しました（{arducopter.returncode}）。{log_path}を確認してください。")
            if proxy.poll() is not None:
                exit_code = proxy.returncode
                break
            if not model_checked:
                model_checked = check_runtime_model(log_path, model)
                if not model_checked and time.monotonic() > model_deadline:
                    raise RuntimeError("モデル拡張の実行確認ができません。./scripts/prepare-prototype.sh を実行してください。")
                if model_checked:
                    metadata["runtime_model_verified"] = True
                    write_json(record_path, metadata)
            time.sleep(0.1)
    except KeyboardInterrupt:
        exit_code = 130
        if metadata is not None:
            metadata["stop_reason"] = "interrupted"
    except Exception as error:
        print(f"起動を終了しました: {error}", file=sys.stderr, flush=True)
        if metadata is not None:
            metadata["error"] = str(error)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stop_child(proxy)
        stop_child(arducopter)
        for sock in reservations:
            sock.close()
        if log_stream is not None:
            log_stream.close()
        if metadata is not None:
            metadata.update(state="stopped", finished_utc=datetime.now(timezone.utc).isoformat(), exit_code=exit_code,
                            arducopter_exit_code=arducopter.returncode if arducopter else None,
                            mavproxy_exit_code=proxy.returncode if proxy else None)
            write_json(record_path, metadata)
        if lock_stream is not None:
            lock_stream.close()
    return exit_code


def main():
    parser = argparse.ArgumentParser(description="第１弾B〜D（12.3387kg）の手動SITL起動。保存先: runs/prototype-manual。自動アームはしません。")
    parser.add_argument("--reset", action="store_true", help="ポート確認後にeeprom.binだけを削除して既定設定へ戻す。過去ログは保持")
    args = parser.parse_args()
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        return launch(args)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f"起動できません: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
