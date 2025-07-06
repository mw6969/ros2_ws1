#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("my_node");

  rclcpp::Rate loop_rate(500ms);
  int counter = 0;
  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Hello %d", counter++);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}