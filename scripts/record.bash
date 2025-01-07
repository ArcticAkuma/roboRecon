#!/bin/bash

DIRECTORY="rosbag"

if ! command -v "roslaunch" 2>&1 >/dev/null
then
  source /opt/ros/noetic/setup.bash
fi

if [ "$#" -eq 0 ]; then
  echo "Please provide file name of ros bag file."
  exit 1
fi

if [ ! -d "$DIRECTORY" ]; then
  mkdir -p $DIRECTORY
fi

file_name="$1"
if [[ ! "$file_name" == *.bag ]]; then
  file_name="$file_name.bag"
fi

file_path=$DIRECTORY/$file_name
if [ -d "$file_path" ]; then
  echo "$file_path does already exist."
  exit 1
fi

rosbag record -O "$file_path" /camera/aligned_depth_to_color/camera_info  camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /camera/imu /camera/imu_info /tf_static
