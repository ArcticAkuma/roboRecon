#!/bin/bash

echo -e '\0033\0143'
echo " ______       _           ______                         "
echo "(_____ \     | |         (_____ \                        "
echo " _____) )___ | |__   ___  _____) )_____  ____ ___  ____  "
echo "|  __  // _ \|  _ \ / _ \|  __  /| ___ |/ ___) _ \|  _ \ "
echo "| |  \ \ |_| | |_) ) |_| | |  \ \| ____( (__| |_| | | | |"
echo "|_|   |_\___/|____/ \___/|_|   |_|_____)\____)___/|_| |_|"
echo "                                                         "
echo ""
source ~/roboRecon/roboEnv/bin/activate
echo "   >> Python environment initialized"
echo ""
source /opt/ros/noetic/setup.bash
echo "   >> ROS noetic initialized"
source ~/roboRecon/noetic_ws/devel/setup.bash
echo "   >> ROS project initialized"

eth0="$(ifconfig eth0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')"
wlan0="$(ifconfig wlan0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')"
addr=""

if [ ! -z "$eth0" ]
then
  addr=$eth0
elif [ ! -z "$wlan0" ]
then
  addr=$wlan0
else
  echo "   >> Unable to fetch network IP"
  echo "   >> Please configure ROS_IP accordingly"
  echo ""
  echo ""
  exit 1
fi

  echo ""
  export ROS_MASTER_URI="http://${addr}:11311"
  echo "   >> ROS_MASTER_URI configured as: http://${addr}:11311"
  export ROS_IP=$addr
  echo "   >> ROS_IP configured as: ${addr}"

echo ""
echo ""