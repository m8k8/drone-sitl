#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_environment
exec "$SITL_VENV_PYTHON" "$SITL_ROOT/scripts/prototype_test.py" "$@"
