#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/rl/workspace/quadruped_nav/ros2_ws/install/setup.bash
root=/home/rl/workspace/quadruped_nav
out=/data0/rl/quadruped_nav/runs/initial_pose_coverage_20261113_01/full
while [ ! -f "$out/case01/result.json" ]; do sleep 2; done
/usr/bin/python3 "$root/tests/score_full_case.py" "$out/case01"
run_case() {
  label="$1"
  shift
  /usr/bin/python3 "$root/tests/run_initial_pose_case.py" --root "$root" --output "$out/$label" "$@"
  /usr/bin/python3 "$root/tests/score_full_case.py" "$out/$label"
}
run_case case02 --x 4.5 --y 1.8 --yaw-deg 180
run_case case03 --x 0 --y 1.8 --yaw-deg 270
run_case case04 --x 2.8 --y -1.8 --yaw-deg 135
run_case case05 --x -2.8 --y 0 --yaw-deg 45 --seed-dx 0 --seed-dy 0 --seed-yaw-error-deg 90
run_case case06 --x -4.5 --y -1.8 --yaw-deg 0 --seed-dx 2 --seed-dy 0 --seed-yaw-error-deg 0
