# RoboRecon - a university minor project

![License](https://img.shields.io/badge/license-MIT-green)

### Legacy files
This GitHub branch represents an initial attempt at implementing a data transmission system via TCP sockets. 
The primary objective was to establish a wireless communication channel to transmit data from a [Donkeycar](https://github.com/autorope/donkeycar) to a secondary system over a network.
The intention was to receive sensor data, control commands, or other relevant information from the DonkeyCar in real-time and have it processed or analyzed on a remote system.

The chosen approach involved setting up a TCP socket connection between the DonkeyCar's onboard system and the second system, enabling uni-directional data transfer. 
This setup was intended to facilitate real-time mapping, remote monitoring and additional processing tasks that could be computationally expensive for the robot itself.

However, after initial exploration, the project was eventually abandoned in favor of a more robust and scalable solution using [ROS](https://ros.org/). 
ROS provided a more comprehensive framework for handling complex robotic systems, offering built-in tools for communication, data handling, and distributed processing, which were more suitable for the development of the project.