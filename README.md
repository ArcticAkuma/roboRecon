# RoboRecon - a minor-level university robotics project

![License](https://img.shields.io/badge/license-MIT-green)

### Project Overview
This repository is part of a minor-level university project focused on the implementation of an autonomous RC car. 
The primary goal of the project is to design and build a self-driving RC car equipped with Simultaneous Localization and Mapping (SLAM) and navigation capabilities. 
The implementation utilizes [ROS1 Noetic](http://wiki.ros.org/noetic) running locally on an [NVIDIA Jetson Nano Xavier NX](https://www.nvidia.com/de-de/autonomous-machines/embedded-systems/jetson-xavier-nx/).<p>

Due to hardware-limitations this implementation is designed to function with a single sensor: the [Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) depth camera. 
The D435 provides both depth and RGB data, which is used for SLAM, obstacle detection, and navigation. 

### Hardware
The car uses two servos, one for steering and another for throttle, which are managed via an Electronic Speed Controller (ESC).
An [Adafruit PCA9685](https://www.adafruit.com/product/815) PWM module is deployed to control these servos. 
The core computational tasks are handled by the NVIDIA Jetson Nano Xavier, which processes the sensor data and runs the software stack. 
The car's only sensor is the Intel RealSense D435 depth camera.<p>

It is important to note that the hardware configuration, particularly the reliance on a single sensor, imposes major limitations on the system’s performance and reliability. 
The absence of additional sensors, such as a LiDAR, means that the system has to make compromises in terms of precision and robustness. 
A multi-sensor setup could significantly improve the accuracy, reliability, and overall capabilities of the car.

### Prerequisites
**General:**
* **[Nvidia JetPack](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) 5.1.4** @ Ubuntu 20.04
* **[ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)** (ros-noetic-desktop recommended)

**ROS packages:**
* **[librealsense2](https://github.com/IntelRealSense/librealsense):** ```sudo apt install ros-noetic-librealsense2*```
* **[realsense2](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy):** ```sudo apt install ros-noetic-realsense2-*```
* **[robot_localization](http://wiki.ros.org/robot_localization):** ```sudo apt install ros-noetic-robot-localization```
* **[rtabmap_ros](http://wiki.ros.org/rtabmap_ros):** ```sudo apt install ros-noetic-rtabmap-ros```
* **[depthimage_to_laserscan](http://wiki.ros.org/depthimage_to_laserscan):** ```sudo apt install ros-noetic-depthimage-to-laserscan```
* **[joy](http://wiki.ros.org/joy):** ```sudo apt install ros-noetic-joy```
* **[navigation](http://wiki.ros.org/navigation):** ```sudo apt install ros-noetic-navigation```
* **[ros-i2cpwmboards](https://github.com/EricWiener/ros-i2cpwmboards):** ```git clone https://github.com/EricWiener/ros-i2cpwmboards.git```

### Installation
* Create a ROS workspace
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src/
    ```
* Clone repository 
   ```bash
   git clone https://github.com/ArcticAkuma/roboRecon -b ros1 --singlebranch
   cd ~/catkin_ws
    ```
* Install ROS package dependencies as described previously

* Build
   ```bash
   catkin_make
    ```
* Source environment
   ```bash
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    ```

### Usage
Please be aware, that the i2c addresses of the servos are currently hard-coded with no possibility to be changed from the launch file.
You may need to adjust them [here](src/low_level_control.py#L60).
1. **Start SLAM & navigation**<p>
    Currently, there is no automatic goal finding.<br/> 
    Set it by publishing a [geometry_msgs/PoseStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) to the navigation [move_base](http://wiki.ros.org/move_base#Action_Subscribed_Topics) on move_base_simple/goal.
   ```bash
   roslaunch robo_recon recon.launch
    ```
   There are two arguments, that can be set:
    * controller (default: false): If 'true' enables steering with a DualSense controller, but disables navigation stack.
    * rtab_db (default: ~/.ros/robo_recon_rtab.db): Sets the location of the generated rtabmap database file. Can be used for multi-session operations.
    ```bash
    roslaunch robo_recon recon.launch controller:=[true/false] rtab_db:=[path/to/rtabmap.db]
    ```
2. **Drive only (no SLAM, no navigation)**<p>
    Possibility to drive using a DualSense controller.<br/> 
    When using other controllers it might be necessary to change 'max_throttle' and 'max_steering' parameters to accommodate for maximum joystick input.
   ```bash
   roslaunch robo_recon joystick.launch
    ```
3. **Rviz quickstart**<p>
    Possibility to quickstart rviz with corresponding config file.
   ```bash
   roslaunch robo_recon rviz.launch
    ```
4. **Dummy.launch??**<p>
    This is the same implementation as in _1. Start SLAM & navigation_, but it will always start SLAM from scratch.<br/>
    It also provides the controller argument as detailed above. 
   ```bash
   roslaunch robo_recon dummy.launch
    ```
   
### License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

