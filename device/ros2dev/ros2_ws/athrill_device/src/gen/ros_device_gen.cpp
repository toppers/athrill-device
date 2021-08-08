#include "ros_device_gen.h"
#include "ros_device_cppgen.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

static std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher;

RosDevReturnType ros_device_send_Int32(RosDevTopicIdType topic_id, Ros2DevInt32Type* msg)
{
    auto topic_msg = std::make_unique<std_msgs::msg::Int32>();
    topic_msg->data = msg->data;
    publisher->publish(std::move(topic_msg));
	return ROSDEV_E_OK;
}

RosDevReturnType ros_device_receive_Int32(RosDevTopicIdType topic_id, Ros2DevInt32Type* msg)
{
	//TODO
	return ROSDEV_E_OK;
}

void ros_device_gen_init(std::shared_ptr<rclcpp::Node> node)
{
	 publisher = node->create_publisher<std_msgs::msg::Int32>("test_data", 1);
}