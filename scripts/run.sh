#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"

sitl_reset=false
case "${1:-}" in
  '') ;;
  --reset) sitl_reset=true; shift ;;
  -h|--help)
    echo '使い方: ./scripts/run.sh [--reset]'
    echo 'MAVProxy CLIで起動。--resetはruns/defaultの保存パラメータを初期化します。'
    exit 0 ;;
  *) echo "未対応の引数: $1" >&2; exit 2 ;;
esac
if [[ $# -ne 0 ]]; then
  echo '使い方: ./scripts/run.sh [--reset]' >&2
  exit 2
fi
require_environment
if [[ ! -x "$SITL_SOURCE/build/sitl/bin/arducopter" ]]; then
  echo '先に ./scripts/build.sh を実行してください。' >&2
  exit 1
fi
# Refuse to attach MAVProxy to an already-running process on the default port.
"$SITL_VENV_PYTHON" -c '
import socket, sys
with socket.socket() as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", 5760))
    except OSError as exc:
        sys.exit(f"TCP 5760を使用できません。既存のSITLを終了してください: {exc}")
'
mkdir -p "$SITL_ROOT/runs/default"
cd "$SITL_ROOT/runs/default"
echo '標準quad / CMACで起動します。操作手順はREADME.mdを参照してください。'
echo '終了: Ctrl+C。機体ログ: runs/default/logs、通信ログ: runs/default/mav.tlog'
echo 'SITLの起動メッセージ: /tmp/ArduCopter.log'
# The upstream headless launcher keeps this terminal for MAVProxy.
unset DISPLAY TMUX STY ZELLIJ SITL_RITW_TERMINAL
sitl_launch_args=(
  -v ArduCopter -f quad -L CMAC --no-rebuild --no-configure \
  --use-dir "$SITL_ROOT/runs/default" \
  --mavproxy-args='--default-modules=log,wp,fence,param,arm,mode,rc,cmdlong,battery,output'
)
if "$sitl_reset"; then
  sitl_launch_args+=(-w)
fi
exec "$SITL_VENV_PYTHON" "$SITL_SOURCE/Tools/autotest/sim_vehicle.py" "${sitl_launch_args[@]}"
