cmake_minimum_required(VERSION 3.10)
project(ignition_ros2_srv_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ignition-transport11 REQUIRED)


set(dependencies
    rclcpp 
    ros_gz_interfaces
)

add_executable(spawn_entity_bridge src/spawn_entity_bridge.cpp)
ament_target_dependencies(spawn_entity_bridge ${dependencies})
add_executable(delete_entity_bridge src/delete_entity_bridge.cpp)
ament_target_dependencies(delete_entity_bridge ${dependencies})


target_link_libraries(spawn_entity_bridge 
  ignition-transport11::core
  protobuf::libprotobuf
  "${cpp_typesupport_target}")
  
target_link_libraries(delete_entity_bridge 
  ignition-transport11::core
  protobuf::libprotobuf
  "${cpp_typesupport_target}")


ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  spawn_entity_bridge
  delete_entity_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()


