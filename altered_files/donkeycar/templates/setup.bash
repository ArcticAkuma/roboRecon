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
source /opt/ros/noetic/setup.bash
echo "   >> ROS noetic initialized"

eth0="$(ifconfig eth0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')"
wlan0="$(ifconfig wlan0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')"

if [ ! -z "$eth0" ]; then
  export ROS_IP=$eth0
  echo "   >> ROS_IP configured to eth0-address: ${eth0}"
elif [ ! -z "$wlan0" ]; then
  export ROS_IP=$wlan0
  echo "   >> ROS_IP configured to wlan0-address: ${wlan0}"
else
  echo "   >> Unable to fetch network IP"
  echo "   >> Please configure ROS_IP accordingly"
fi


echo ""
echo ""