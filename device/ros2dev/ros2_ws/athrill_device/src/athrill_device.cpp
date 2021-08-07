#include <stdio.h>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "athrill_device.h"
#include "athrill_device_cppgen.hpp"


using namespace std::chrono_literals;

void athrill_device_init(void)
{
	rclcpp::init(0, NULL);
	auto node = rclcpp::Node::make_shared("athrill_node");
	athrill_device_gen_init(node);

    rclcpp::WallRate rate(10ms);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    return;
}
