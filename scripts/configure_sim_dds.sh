#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
profile="$project_dir/ros2_ws/src/quadruped_bringup/config/dds/socket_limits.conf"
sudo -n install -m 0644 "$profile" /etc/sysctl.d/90-quadruped-nav-dds.conf
sudo -n sysctl -p /etc/sysctl.d/90-quadruped-nav-dds.conf
