#!/bin/bash

if ! command -v "roslaunch" 2>&1 >/dev/null
then
source /opt/ros/noetic/setup.bash
fi

if [ -z "$(rosnode list | grep 'realsense')" ]
then
  echo  "Realsense node already online."
else
  roslaunch realsense2_camera opensource_tracking.launch
fi