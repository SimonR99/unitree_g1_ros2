#!/usr/bin/env bash
# quiet_run.sh — wrapper used as a launch_ros `prefix=` to strip rmw_cyclonedds
# / rcutils boilerplate from a child node's stderr.
#
# On the Unitree G1 the native cyclonedds publishers (rt/lowstate, /api/...,
# /utlidar/..., etc.) emit DDS discovery records whose string fields are not
# null-terminated. rmw_cyclonedds_cpp's deserializer rejects them and rcutils
# spams a multi-line "error state being overwritten" block from EVERY rclpy
# node, drowning out useful logs. This wrapper filters that specific spam
# while keeping every other line (Python tracebacks, our own info logs, etc.)
# intact.
#
# Usage:
#   prefix=['/path/to/quiet_run.sh']     # via ros2 launch
#   ./quiet_run.sh ros2 run pkg exe ...   # standalone
set -e

exec "$@" 2> >(stdbuf -oL awk '
  BEGIN { skip = 0 }

  # rcutils prints multi-line blocks framed by ">>> ... <<<" when the stored
  # error state gets overwritten. Drop the whole block.
  /^>>> \[rcutils\|error_handling/ { skip = 1; next }
  skip && /^<<<$/                  { skip = 0; next }
  skip                             { next }

  # Single-line noise from cyclonedds when it cannot deserialize a discovery
  # or DCPSParticipant record sent by a Unitree native publisher.
  /^bad_alloc caught: std::bad_alloc[[:space:]]*$/ { next }
  /Deserialization of data failed/                 { next }
  /-> Function +deserialize_change/                { next }

  # Anything else: forward unchanged.
  { print }
' >&2)
