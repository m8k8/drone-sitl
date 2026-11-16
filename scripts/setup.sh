#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"

if [[ "$(uname -s)" != Darwin ]]; then
  echo 'このセットアップはmacOS向けです。READMEの対応環境を確認してください。' >&2
  exit 1
fi
for tool in git clang clang++; do
  if ! command -v "$tool" >/dev/null; then
    echo "必要なコマンドがありません: $tool (xcode-select --install)" >&2
    exit 1
  fi
done
if [[ -n "${SITL_PYTHON:-}" ]]; then
  sitl_setup_python="$SITL_PYTHON"
elif [[ -x /opt/homebrew/opt/python@3.12/bin/python3.12 ]]; then
  sitl_setup_python=/opt/homebrew/opt/python@3.12/bin/python3.12
elif command -v python3.12 >/dev/null; then
  sitl_setup_python="$(command -v python3.12)"
else
  echo 'Python 3.12が必要です。brew install python@3.12 を実行してください。' >&2
  exit 1
fi
"$sitl_setup_python" -c 'import sys; assert sys.version_info[:2] == (3, 12), "Python 3.12を指定してください"'
mkdir -p "$SITL_ROOT/vendor" "$PIP_CACHE_DIR" "$SITL_ROOT/runs"
if [[ ! -d "$SITL_SOURCE" ]]; then
  git clone --depth 1 --branch "$ARDUPILOT_TAG" "$ARDUPILOT_REPOSITORY" "$SITL_SOURCE"
fi
if [[ "$(git -C "$SITL_SOURCE" rev-parse HEAD)" != "$ARDUPILOT_COMMIT" ]]; then
  echo '取得済みソースが固定コミットと異なるため停止しました。ソースを自動上書きしません。' >&2
  exit 1
fi
git -C "$SITL_SOURCE" submodule update --init --recursive --depth 1 --jobs 4
if [[ ! -x "$SITL_VENV_PYTHON" ]]; then
  "$sitl_setup_python" -m venv "$SITL_ROOT/.venv"
fi
"$SITL_VENV_PYTHON" -c 'import sys; assert sys.version_info[:2] == (3, 12), ".venvはPython 3.12で作成してください"'
"$SITL_VENV_PYTHON" -m pip install --no-deps -r "$SITL_ROOT/requirements-cli.txt"
"$SITL_VENV_PYTHON" -c 'import em, lxml, pymavlink, MAVProxy, pexpect, numpy, psutil, serial, defusedxml, pynmeagps; print("CLI依存の読み込み: OK")'
echo 'セットアップ完了。次は ./scripts/prepare-prototype.sh を実行してください。'
