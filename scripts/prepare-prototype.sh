#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_environment
"$SITL_VENV_PYTHON" "$SITL_ROOT/config/prototype-01/generate.py" --check
"$SITL_VENV_PYTHON" "$SITL_ROOT/config/prototype-01/prepare.py"
"$SITL_ROOT/scripts/build.sh"
echo '第１弾（B〜D）のモデルを準備しました。config/prototype-01/README.mdの仮定と再現範囲を確認してください。'
