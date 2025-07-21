#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "urdf_tutorial_interfaces/srv/set_grasp_transform.hpp"
#include "kinenikros2/srv/inverse_kinematics.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using SetGrasp = urdf_tutorial_interfaces::srv::SetGraspTransform;
using IKService = kinenikros2::srv::InverseKinematics;

class DemoGraspClient : public rclcpp::Node {
public:
  DemoGraspClient()
  : Node("demo_grasp_client") {
    RCLCPP_INFO(this->get_logger(), "DemoGraspClient starting...");

    // Create clients
    grasp_client_ = this->create_client<SetGrasp>("/set_grasp_transform");
    ik_client_ = this->create_client<IKService>("/inverse_kinematics");

    // Send requests
    send_requests();
  }

private:
  rclcpp::Client<SetGrasp>::SharedPtr grasp_client_;
  rclcpp::Client<IKService>::SharedPtr ik_client_;

  void send_requests() {
    // Wait for services
    RCLCPP_INFO(this->get_logger(), "Waiting for /set_grasp_transform service...");
    if (!grasp_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Service /set_grasp_transform not available");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Calling /set_grasp_transform...");

    auto grasp_req = std::make_shared<SetGrasp::Request>();
    grasp_req->object_id = 12;
    geometry_msgs::msg::TransformStamped t;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.15;
    {
        tf2::Quaternion q;
        q.setRPY(3.14, 0.0, 0.0);
        q.normalize();
        grasp_req->object_grasp_transform.transform.rotation.x = q.x();
        grasp_req->object_grasp_transform.transform.rotation.y = q.y();
        grasp_req->object_grasp_transform.transform.rotation.z = q.z();
        grasp_req->object_grasp_transform.transform.rotation.w = q.w();
    }
    grasp_req->object_grasp_transform = t;

    auto grasp_future = grasp_client_->async_send_request(grasp_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), grasp_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service set_grasp_transform");
      return;
    }
    if (!grasp_future.get()->ret) {
      RCLCPP_ERROR(this->get_logger(), "set_grasp_transform returned false");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Grasp transform set successfully");

    // Now call IK
    RCLCPP_INFO(this->get_logger(), "Waiting for /inverse_kinematics service...");
    if (!ik_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Service /inverse_kinematics not available");
      return;
    }

    auto ik_req = std::make_shared<IKService::Request>();
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = t.transform.translation.x;
    target_pose.position.y = t.transform.translation.y;
    target_pose.position.z = t.transform.translation.z;
    target_pose.orientation = t.transform.rotation;
    ik_req->pose = target_pose;

    auto ik_future = ik_client_->async_send_request(ik_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), ik_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service inverse_kinematics");
      return;
    }
    auto ik_resp = ik_future.get();
    if (!ik_resp->status) {
      RCLCPP_ERROR(this->get_logger(), "inverse_kinematics returned failure");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Received %zu IK solutions", ik_resp->ik_solution.size());

    // Publish using visualization marker or topic as needed (handled by RViz)
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoGraspClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}