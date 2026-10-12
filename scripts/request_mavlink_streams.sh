#!/usr/bin/env bash
set -euo pipefail
if ! command -v rosrun >/dev/null 2>&1; then
  printf 'Source the ROS and catkin workspace setup files first.\n' >&2
  exit 1
fi
# Original message interval requests. Run only after MAVROS is connected.
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
rosrun mavros mavcmd long 511 32 10000 0 0 0 0 0
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
