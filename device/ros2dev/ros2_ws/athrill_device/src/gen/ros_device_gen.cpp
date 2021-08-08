#include "ros_device_gen.h"
#include "ros_device_cppgen.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace std::chrono_literals;
//TODO <package>::msg
using namespace std_msgs::msg;

//TODO static std::shared_ptr<rclcpp::Publisher<type>> publisher_<topic_name>;
static std::shared_ptr<rclcpp::Publisher<Int32>> publisher_test_pub_data;

//TODO RosDevReturnType ros_device_send_<type>(RosDevTopicIdType topic_id, RosDev<type>Type* msg)
RosDevReturnType ros_device_send_Int32(RosDevTopicIdType topic_id, RosDevInt32Type* msg)
{
	if (topic_id == 0)
	{
		//TODO
		auto topic_msg = std::make_unique<std_msgs::msg::Int32>();
		topic_msg->data = msg->data;
		publisher_test_pub_data->publish(std::move(topic_msg));
		return ROSDEV_E_OK;
	}
	else 
	{
		return ROSDEV_E_INVALID;
	}
}

//TODO Ros<TopicName>DataType
typedef struct {
	bool is_exist;
	RosDevInt32Type value;
} RosTopicTestSubDataType;

//TODO static RosTopic<TopicName>Type buffer_<topic_name>;
static RosTopicTestSubDataType buffer_test_sub_data;
RosDevReturnType ros_device_receive_Int32(RosDevTopicIdType topic_id, RosDevInt32Type* msg)
{
	if (topic_id == 0) {
		//TODO
		if (buffer_test_sub_data.is_exist) {
			*msg = buffer_test_sub_data.value;
			buffer_test_sub_data.is_exist = false;
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
//TODO static void <TopicName>Callback(const <Type>::SharedPtr msg) 
static void TestSubDataCallback(const Int32::SharedPtr msg) 
{
	buffer_test_sub_data.is_exist = true;
	buffer_test_sub_data.value.data = msg->data;
	return;
}

static void ros_node(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::WallRate rate(10ms);

	//TODO 	auto subscriber_<topic_name> = node->create_subscription<<Type>>("test_sub_data", 1, TestSubDataCallback);
	auto subscriber_test_sub_data = node->create_subscription<Int32>("test_sub_data", 1, TestSubDataCallback);
	while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
	rclcpp::shutdown();
	return;
}

void ros_device_gen_init(std::shared_ptr<rclcpp::Node> node)
{
	//TODO publisher_<topic_name> = node->create_publisher<<Type>>("<topic_name>", 1);
	publisher_test_pub_data = node->create_publisher<Int32>("test_pub_data", 1);
	std::thread thr(ros_node, node);
	thr.detach();
	return;
}