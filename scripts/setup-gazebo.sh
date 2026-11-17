#!/usr/bin/env bash
set -euo pipefail

sitl_root="$(cd "$(dirname "$0")/.." && pwd)"
gazebo_cache="$sitl_root/.cache/gazebo"
gazebo_prefix="$gazebo_cache/env"
micromamba="$gazebo_cache/bin/micromamba"

if [[ "$(uname -s)" != Darwin || "$(uname -m)" != arm64 ]]; then
  echo 'このセットアップは Apple Silicon の macOS 用です。' >&2
  exit 1
fi

mkdir -p "$gazebo_cache"
if [[ ! -x "$micromamba" ]]; then
  archive="$(mktemp "$gazebo_cache/micromamba.XXXXXXXX")"
  trap 'rm -f "$archive"' EXIT
  curl --fail --silent --show-error --location --retry 2 --proto '=https' --proto-redir '=https' \
    --output "$archive" https://micro.mamba.pm/api/micromamba/osx-arm64/latest
  tar -xjf "$archive" -C "$gazebo_cache" bin/micromamba
fi

action=create
if [[ -f "$gazebo_prefix/conda-meta/history" ]]; then
  action=install
fi
"$micromamba" --no-rc --root-prefix "$gazebo_cache/mamba-root" "$action" --yes \
  --prefix "$gazebo_prefix" --override-channels --channel conda-forge \
  'python=3.12' 'gz-jetty=1.*' 'gz-transport-python=15.*' 'gz-msgs-python=12.*'

"$gazebo_prefix/bin/python" -c 'from gz.transport import Node; from gz.msgs.pose_pb2 import Pose; from gz.sim import Model; assert hasattr(Model, "set_world_pose_cmd")'
echo 'Gazebo Jetty を準備しました。./scripts/run.sh --gazebo で起動できます。'
