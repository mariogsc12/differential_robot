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
  "rover.*"
  "joy.*"
  ".*controller.*"
)

soft_kill(){
  echo "Starting SOFT kill using SIGTERM..."
  for pattern in "${processes[@]}"; do
    pids=$(pgrep -f "$pattern" | grep -v "$SCRIPT_PID")
  
    if [ -n "$pids" ]; then
        echo "Killing processes matching '$pattern':"
        for pid in $pids; do
        ps -p $pid -o pid,cmd --no-headers
        kill -TERM $pid 2>/dev/null
        done
    fi
  done
}

hard_kill() {
  echo "Starting HARD kill using pkill -9..."
  pkill -9 -f ros
  ros2 daemon stop
  ros2 daemon start
}

print_help() {
  echo ""
  echo "Bash file to kill ROS 2 processes."
  echo ""
  echo "Arguments:"
  echo "  --help         Show this help"
  echo "  -s, --soft     Soft kill (SIGTERM)"
  echo "  -h, --hard     Hard kill (SIGKILL + daemon restart)"
  echo ""
}

parse_command_line(){
  if [[ $# -eq 0 ]]; then
    soft_kill
    return
  fi

  while [[ $# -gt 0 ]]; do
    case $1 in
      --help)
        print_help
        exit 0
        ;;
      -h|--hard)
        hard_kill
        exit 0
        ;;
      -s|--soft)
        soft_kill
        exit 0
        ;;
      -b|--both)
        soft_kill
        echo ""
        hard_kill
        exit 0
        ;;
      *)
        echo "Unkown option. Launching soft killing as default"
        soft_kill
        exit 0
        ;;
    esac
    shift
  done
}

main(){
  parse_command_line "$@"
}

main "$@"