// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/my_message.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode()
  : Node("subscriber_node")
  {
    subscriber_ = create_subscription<custom_interfaces::msg::MyMessage>(
      "my_topic", 10,
      std::bind(&SubscriberNode::callback, this, _1));
  }

  void callback(const custom_interfaces::msg::MyMessage::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Name = %s", msg->name.c_str());
    RCLCPP_INFO(get_logger(), "x = %f", msg->center.x);
    RCLCPP_INFO(get_logger(), "y = %f", msg->center.y);
    RCLCPP_INFO(get_logger(), "z = %f", msg->center.z);
  }

private:
  rclcpp::Subscription<custom_interfaces::msg::MyMessage>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SubscriberNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
