#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "urdf_tutorial_interfaces/srv/set_grasp_transform.hpp"

using SetGrasp = urdf_tutorial_interfaces::srv::SetGraspTransform;

class CustomGraspBroadcaster : public rclcpp::Node {
public:
  CustomGraspBroadcaster()
  : Node("custom_grasps_tf2_broadcaster") {
    service_ = this->create_service<SetGrasp>(
      "set_grasp_transform",
      std::bind(&CustomGraspBroadcaster::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(), "CustomGraspBroadcaster ready");
  }

private:
  void handle_service(
    const std::shared_ptr<SetGrasp::Request> request,
    std::shared_ptr<SetGrasp::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received SetGraspTransform request for object %d", request->object_id);
    auto t = request->object_grasp_transform;
    t.header.stamp = this->now();
    t.header.frame_id = "tf_" + std::to_string(request->object_id);
    t.child_frame_id = "ur_grasp";

    tf_broadcaster_->sendTransform(t);
    response->ret = true;
    RCLCPP_INFO(this->get_logger(), "Broadcasted grasp TF for object %d", request->object_id);
  }

  rclcpp::Service<SetGrasp>::SharedPtr service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomGraspBroadcaster>());
  rclcpp::shutdown();
  return 0;
}