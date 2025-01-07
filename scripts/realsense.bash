#!/bin/bash

if ! command -v "roslaunch" 2>&1 >/dev/null
then
source /opt/ros/noetic/setup.bash
fi

roslaunch realsense2_camera opensource_tracking.launch