#!/usr/bin/env bash
# Shared project paths; source from the other shell scripts.
set -euo pipefail
SITL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SITL_SOURCE="$SITL_ROOT/vendor/ardupilot"
SITL_VENV_PYTHON="$SITL_ROOT/.venv/bin/python"
source "$SITL_ROOT/config/versions.env"
export PATH="$SITL_ROOT/.venv/bin:$PATH"
export PIP_CACHE_DIR="$SITL_ROOT/.cache/pip"
export PYTHONNOUSERSITE=1

require_environment() {
  if [[ ! -x "$SITL_VENV_PYTHON" || ! -f "$SITL_SOURCE/waf" ]]; then
    echo '先に ./scripts/setup.sh を実行してください。' >&2
    exit 1
  fi
  if [[ "$(git -C "$SITL_SOURCE" rev-parse HEAD)" != "$ARDUPILOT_COMMIT" ]]; then
    echo 'ArduPilotのコミットが固定バージョンと異なります。config/versions.envを確認してください。' >&2
    exit 1
  fi
}
