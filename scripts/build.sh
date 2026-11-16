#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_environment
cd "$SITL_SOURCE"
"$SITL_VENV_PYTHON" ./waf configure --board sitl
"$SITL_VENV_PYTHON" ./waf copter -j "${SITL_BUILD_JOBS:-4}"
echo 'ビルド完了。B〜Dモデルの準備後は ./scripts/run.sh または ./scripts/prototype-test.sh で起動できます。'
