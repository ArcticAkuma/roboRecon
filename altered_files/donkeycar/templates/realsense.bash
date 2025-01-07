#!/bin/bash

if ! command -v "roslaunch" 2>&1 >/dev/null
then
source /opt/ros/noetic/setup.bash
fi

if [ -z "$(rosnode list | grep 'realsense')" ]; then
  echo  "Realsense node already online."
else
if [ ! "$#" -eq 0 ] && [ "$1" = "offline" ]; then
    roslaunch realsense2_camera opensource_tracking.launch offline:=true
  else
    roslaunch realsense2_camera opensource_tracking.launch
  fi
fi