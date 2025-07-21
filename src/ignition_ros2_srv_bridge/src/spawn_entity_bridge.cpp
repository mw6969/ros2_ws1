#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/boolean.pb.h>

using namespace std::placeholders;

class GazeboSpawner : public rclcpp::Node
{
public:
  GazeboSpawner()
  : Node("gazebo_spawner_bridge")
  {
    service_ = this->create_service<ros_gz_interfaces::srv::SpawnEntity>(
      "/world/ign_gazebo_world/create",
      std::bind(&GazeboSpawner::spawn_entity_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "SpawnEntity bridge ready");

    //parameter with the world name (defaulted to my_world) - to be set at the launch file!
    declare_parameter("world_name", "my_world");
    get_parameter("world_name", world_name_);
  }

private:
//Callback to spawn an entity given by an sdf file
//The request also has the name of the entity, that overrides that of the sdf
//and if allow_renaming is set to true the sever changes it in case of overlap.
//The request also has the pose where to spawn the entity
//and the frame relative to which this pose is defined (set to "world" by default)
void spawn_entity_callback(
  const std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity::Request> request,
  std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity::Response> response)
{
  ignition::msgs::EntityFactory factory_msg;

  const auto &entity = request->entity_factory;

  if (!entity.sdf_filename.empty())
  {
    factory_msg.set_sdf_filename(entity.sdf_filename);
  }
  else
  {
    response->success = false;
    std::cout << "No filename provided" << std::endl;
    return;
  }

  if (!entity.name.empty())
  {
    factory_msg.set_name(entity.name);
  }

  // Default allow_renaming to true
  factory_msg.set_allow_renaming(true);
  // Override default when client explicitly disables it
  if (entity.allow_renaming == false)
  {
    factory_msg.set_allow_renaming(false);
  }

  // Set pose
  factory_msg.mutable_pose()->mutable_position()->set_x(entity.pose.position.x);
  factory_msg.mutable_pose()->mutable_position()->set_y(entity.pose.position.y);
  factory_msg.mutable_pose()->mutable_position()->set_z(entity.pose.position.z);
  factory_msg.mutable_pose()->mutable_orientation()->set_x(entity.pose.orientation.x);
  factory_msg.mutable_pose()->mutable_orientation()->set_y(entity.pose.orientation.y);
  factory_msg.mutable_pose()->mutable_orientation()->set_z(entity.pose.orientation.z);
  factory_msg.mutable_pose()->mutable_orientation()->set_w(entity.pose.orientation.w);


  ignition::transport::Node ign_node;
  ignition::msgs::Boolean rep;
  bool result;

  std::string ign_create_service_name = "/world/"+world_name_+"/create";

  bool success = ign_node.Request(
    //"/world/camera_world/create",
    ign_create_service_name,
    factory_msg,
    1000,
    rep,
    result);

  response->success = success && rep.data();
  std::string status_message = response->success ? "Entity spawned" : "Failed to spawn entity";
  std::cout << status_message << std::endl;
}


  rclcpp::Service<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr service_;
  std::string world_name_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboSpawner>());
  rclcpp::shutdown();
  return 0;
}
