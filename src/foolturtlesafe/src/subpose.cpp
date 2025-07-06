// This program subscribes to turtle1/pose and shows its
// messages on the screen.
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode()
  : Node("subpose_node")
  {
    subpose_ = create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10,
      std::bind(&SubscriberNode::callback, this, _1));
  }

  void callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Pose received: "
    << "position=(" <<  msg->x << "," << msg->y << ")"
    << " direction=" << msg->theta);
  }

private:
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subpose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SubscriberNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}





