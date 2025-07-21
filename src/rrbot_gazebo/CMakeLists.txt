cmake_minimum_required(VERSION 3.5)
project(rrbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_image REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(sensor_msgs REQUIRED)
  
set(dependencies
    rclcpp
    ros_gz_bridge
    ros_gz_image
    ros_gz_sim
    sensor_msgs
)
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
  
ament_export_dependencies(${dependencies})
ament_package()

