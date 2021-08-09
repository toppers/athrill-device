#include <stdio.h>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include "ros_device.h"
#include "ros_device_cppgen.hpp"

using namespace std::chrono_literals;

void ros_device_init(void)
{
    rclcpp::init(0, NULL);
    auto node = rclcpp::Node::make_shared("athrill_node");
    ros_device_gen_init(node);
    return;
}

void ros_device_fin(void)
{
    rclcpp::shutdown();
    return;
}
