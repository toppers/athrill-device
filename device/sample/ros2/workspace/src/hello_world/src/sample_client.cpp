#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "hello_world/sample_client.h"

using namespace std::chrono_literals;


class Talker : public rclcpp::Node
{
public:
  explicit Talker(const std::string & topic_name)
  : Node("athrill2")
  {
    // chatterトピックの送信設定
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    pub_ = create_publisher<std_msgs::msg::String>(topic_name, qos);
    RCLCPP_INFO(this->get_logger(), "Talker up");
  }

  void publish(const char* strp)
  {
    // 送信するメッセージ
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = strp;

    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    pub_->publish(std::move(msg));
    return;
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

static std::shared_ptr<Talker> talker_node = NULL;

void sample_client_init(void)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(0, NULL);

  // talkerノードの生成とスピン開始
  talker_node = std::make_shared<Talker>("chatter");

  return;
}

void sample_client_request(const char* strp, unsigned long long clock)
{
  if (strp == NULL) {
    std::string str = std::to_string(clock);
    talker_node->publish(str.c_str());
  }
  else {
    talker_node->publish(strp);
  }
  return;
}
