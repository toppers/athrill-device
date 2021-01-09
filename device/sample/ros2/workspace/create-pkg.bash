#!/bin/bash

cd src
ros2 pkg create --build-type ament_cmake hello_world    \
    --dependencies rclcpp hello_world_msgs