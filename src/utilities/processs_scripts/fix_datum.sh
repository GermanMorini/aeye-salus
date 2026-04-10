#!/usr/bin/env bash
set -euo pipefail

ros2 service call /mavros/cmd/set_home mavros_msgs/srv/CommandHome "{
  current_gps: true,
  yaw: 0.0,
  latitude: 0.0,
  longitude: 0.0,
  altitude: 0.0
}"