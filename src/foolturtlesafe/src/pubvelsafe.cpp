// This program publishes randomly-generated velocity
// messages for turtlesim.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // For geometry_msgs::msg::Twist.
#include "turtlesim/msg/pose.hpp"
#include <stdlib.h> // For rand() and RAND_MAX.
#include <chrono>

using namespace std::chrono_literals;

class PubVelSafe : public rclcpp::Node
{
public:
  PubVelSafe()
  : Node("pubvelsafe_node"), x_(0.0), y_(0.0)
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&PubVelSafe::pose_callback, this, std::placeholders::_1));
    //To set a publishing frequency of 2Hz.
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PubVelSafe::timer_callback, this));
    // Seed the random number generator.
    srand(time(0));
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    x_ = msg->x;
    y_ = msg->y;
  }

  void timer_callback()
  {
    // Create and fill in the message. The other four
    // fields, which are ignored by turtlesim, default to 0.
    geometry_msgs::msg::Twist msg;
    msg.angular.z = 2.0 * double(rand()) / RAND_MAX - 1.0;

    constexpr double center = 5.5;
    constexpr double d = 2.5; // To use half of the safe zone side.
    const bool in_safe_zone = (x_ > center - d) && (x_ < center + d) &&
                              (y_ > center - d) && (y_ < center + d);
    if (in_safe_zone)
    {
      msg.linear.x = 1.0;
    }
    else
    {
      msg.linear.x = double(rand()) / RAND_MAX;
    }

    pub_->publish(msg);

    RCLCPP_INFO_STREAM(this->get_logger(),
      "Pose: (" << x_ << ", " << y_ << ") "
      << (in_safe_zone ? "IN safe zone" : "OUT of safe zone")
      << " -> linear=" << msg.linear.x << " angular=" << msg.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_;
  double y_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PubVelSafe>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

