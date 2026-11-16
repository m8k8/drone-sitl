#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_environment
"$SITL_VENV_PYTHON" "$SITL_ROOT/config/g06/generate.py" --check
"$SITL_VENV_PYTHON" "$SITL_ROOT/config/g06/prepare.py"
"$SITL_ROOT/scripts/build.sh"
echo 'G06暫定モデル用のビルドが完了しました。config/g06/manifest.jsonの仮定を確認してください。'
