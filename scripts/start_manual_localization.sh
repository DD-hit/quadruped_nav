#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$project_root/ros2_ws/install/setup.bash"
exec /usr/bin/python3 "$project_root/scripts/manual_localization.py" "$@"
