#!/bin/bash

if ! command -v "roslaunch" 2>&1 >/dev/null
then
source /opt/ros/noetic/setup.bash
fi

list="$(rosnode list | grep 'realsense')"
if [ ! -z "$list" ] && [[ ! "$list" == ERROR* ]]; then
  echo  "Realsense node already online."
  exit 1
fi

if [ ! "$#" -eq 0 ] && [ "$1" = "offline" ]; then
  echo "Starting realsense node in offline mode.."
  roslaunch realsense2_camera opensource_tracking.launch offline:=true
else
  echo "Starting realsense node.."
  roslaunch realsense2_camera opensource_tracking.launch
fi