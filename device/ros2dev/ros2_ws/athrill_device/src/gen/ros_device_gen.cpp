#include "ros_device_gen.h"
#include "ros_device_cppgen.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace std::chrono_literals;
static std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher;

RosDevReturnType ros_device_send_Int32(RosDevTopicIdType topic_id, RosDevInt32Type* msg)
{
	if (topic_id == 0)
	{
		auto topic_msg = std::make_unique<std_msgs::msg::Int32>();
		topic_msg->data = msg->data;
		publisher->publish(std::move(topic_msg));
		return ROSDEV_E_OK;
	}
	else 
	{
		return ROSDEV_E_INVALID;
	}
}

typedef struct {
	bool is_exist;
	RosDevInt32Type value;
} RosTopicTestDataType;
static RosTopicTestDataType test_sub_data;
RosDevReturnType ros_device_receive_Int32(RosDevTopicIdType topic_id, RosDevInt32Type* msg)
{
	if (topic_id == 0) {
		if (test_sub_data.is_exist) {
			*msg = test_sub_data.value;
			test_sub_data.is_exist = false;
			return ROSDEV_E_OK;
		}
		else
		{
			return ROSDEV_E_NOENT;
		}
	}
	else 
	{
		return ROSDEV_E_INVALID;
	}
}

static void subCallback(const std_msgs::msg::Int32::SharedPtr msg) 
{
	test_sub_data.is_exist = true;
	test_sub_data.value.data = msg->data;
	return;
}

static void ros_node(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::WallRate rate(10ms);

	auto subscriber = node->create_subscription<std_msgs::msg::Int32>("test_sub_data", 1, subCallback);
	while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
	rclcpp::shutdown();
	return;
}

void ros_device_gen_init(std::shared_ptr<rclcpp::Node> node)
{
	publisher = node->create_publisher<std_msgs::msg::Int32>("test_pub_data", 1);
	std::thread thr(ros_node, node);
	thr.detach();
	return;
}