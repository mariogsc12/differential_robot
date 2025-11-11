#!/bin/bash

SCRIPT_PID=$$

processes=(
  "static_transform_publisher"
  "rviz.*"
  "gz.*"
  "gazebo.*"
  ".*robot.*"
  "controller.*"
  "lifecycle.*"
  ".*launch.*"
  ".*ros2.*"
  "diffbot*"
  "joy.*"
  ".*controller.*"
)

for pattern in "${processes[@]}"; do
    pids=$(pgrep -f "$pattern" | grep -v "$SCRIPT_PID")
  
    if [ -n "$pids" ]; then
        echo "Killing processes matching '$pattern':"
        for pid in $pids; do
        ps -p $pid -o pid,cmd --no-headers
        kill -9 $pid 2>/dev/null
        done
    fi
done