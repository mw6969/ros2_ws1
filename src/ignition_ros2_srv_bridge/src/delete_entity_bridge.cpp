#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/boolean.pb.h>

using namespace std::placeholders;

class GazeboEraser : public rclcpp::Node
{
public:
  GazeboEraser()
  : Node("gazebo_delete_bridge")
  {
    service_ = this->create_service<ros_gz_interfaces::srv::DeleteEntity>(
      "/world/ign_gazebo_world/delete",
      std::bind(&GazeboEraser::delete_entity_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "DeleteEntity bridge ready");

    //parameter with the world name (defaulted to my_world) - to be set at the launch file!
    declare_parameter("world_name", "my_world");
    get_parameter("world_name", world_name_);
  }

private:
//Callback to delete an entity 
//The request is: 
// ros_gz_interfaces/Entity entity  # Gazebo Sim entity to be deleted.
//with entity.msg as:
// uint64 id      # Entity unique identifier across all types. Defaults to 0
// string name    # Entity name, which is not guaranteed to be unique.
// uint8 type     # Entity type: 2 for MODEL
void delete_entity_callback(
  const std::shared_ptr<ros_gz_interfaces::srv::DeleteEntity::Request> request,
  std::shared_ptr<ros_gz_interfaces::srv::DeleteEntity::Response> response)
{
  ignition::msgs::Entity entity_msg;
  const auto &entity = request->entity;

  entity_msg.set_type(static_cast<ignition::msgs::Entity_Type>(entity.type));
  entity_msg.set_name(entity.name);
  entity_msg.set_id(entity.id);

  ignition::transport::Node ign_node;
  ignition::msgs::Boolean rep;
  bool result;

  std::string ign_delete_service_name = "/world/"+world_name_+"/remove";

  bool success = ign_node.Request(
    ign_delete_service_name,
    entity_msg,
    1000,
    rep,
    result);

  response->success = success && rep.data();
  std::string status_message = response->success ? "Entity deleted" : "Failed to delete entity";
  std::cout << status_message << std::endl;
  std::cout << "entity name "<< entity.name<<std::endl;
  std::cout << "entity type "<< entity.type<<std::endl;
  std::cout << "rep.data() "<< rep.data()<<std::endl;
}


  rclcpp::Service<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr service_;
  std::string world_name_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboEraser>());
  rclcpp::shutdown();
  return 0;
}
