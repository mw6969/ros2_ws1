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

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode()
  : Node("publisher_node")
  {
    publisher_ = create_publisher<custom_interfaces::msg::MyMessage>("my_topic", 10);
    timer_ = create_wall_timer(
      500ms, std::bind(&PublisherNode::timer_callback, this));
    counter_ = 0;
    declare_parameter("x_dimension", 1.0);
    declare_parameter("y_dimension", 1.0);
    declare_parameter("z_dimension", 1.0);

    get_parameter("x_dimension", xdim_);
    get_parameter("y_dimension", ydim_);
    get_parameter("z_dimension", zdim_);
  }

  void timer_callback()
  {
    counter_ += 1;
    message_.center.x = xdim_*double(rand())/double(RAND_MAX);
    message_.center.y = ydim_*double(rand())/double(RAND_MAX);
    message_.center.z = zdim_*double(rand())/double(RAND_MAX);
    message_.name = std::string("random point ") + std::to_string(counter_);
    publisher_->publish(message_);
  }

private:
  rclcpp::Publisher<custom_interfaces::msg::MyMessage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  custom_interfaces::msg::MyMessage message_;
  unsigned int counter_;
  double xdim_;
  double ydim_;
  double zdim_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
