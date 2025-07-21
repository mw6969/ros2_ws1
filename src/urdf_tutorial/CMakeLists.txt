cmake_minimum_required(VERSION 3.5)
project(urdf_tutorial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf_tutorial_interfaces REQUIRED)
find_package(kinenikros2 REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    visualization_msgs
    geometry_msgs
    tf2_ros
)

add_executable(rviz_node src/rviz_node.cpp)
ament_target_dependencies(rviz_node ${dependencies})

add_executable(custom_grasps_tf2_broadcaster src/custom_grasps_tf2_broadcaster.cpp)
target_link_libraries(
   custom_grasps_tf2_broadcaster PUBLIC
   ${urdf_tutorial_interfaces_TARGETS}
   ${geometry_msgs_TARGETS}
   rclcpp::rclcpp
   tf2_ros::tf2_ros
)

add_executable(demo_grasp_client src/demo_grasp_client.cpp)
target_link_libraries(
   demo_grasp_client PUBLIC
   ${urdf_tutorial_interfaces_TARGETS}
   ${kinenikros2_TARGETS}
   ${geometry_msgs_TARGETS}
   rclcpp::rclcpp
   tf2_ros::tf2_ros
   tf2::tf2
)

install(TARGETS
  rviz_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  custom_grasps_tf2_broadcaster
  demo_grasp_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY objects DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
