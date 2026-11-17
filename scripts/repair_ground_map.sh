#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_root/ros2_ws/install/setup.bash"
/usr/bin/python3 "$project_root/scripts/repair_ground_map.py"
/usr/bin/python3 "$project_root/tests/test_map_alignment.py"
