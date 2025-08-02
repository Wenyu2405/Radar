#!/bin/bash
sudo ifconfig eno1 192.168.1.50



#进入所在位置：D
cd /home/wenyu/ws_livox
#ROS2环境来来来
source ~/ws_livox/install/setup.bash
#启动！
ros2 launch livox_ros2_driver livox_lidar_launch.py

#/home/wenyu/ws_livox/src/livox_ros2_driver/launch/livox_lidar_launch.py
#chmod +x start_livox.sh
#ls /dev/ttyUSB*
