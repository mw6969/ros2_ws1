cmake_minimum_required(VERSION 3.5)
project(foolturtlesafe)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

set(dependencies
    rclcpp 
    std_msgs
    geometry_msgs
    turtlesim
)

add_executable(pubvelsafe src/pubvelsafe.cpp)
ament_target_dependencies(pubvelsafe ${dependencies})

add_executable(subpose src/subpose.cpp)
ament_target_dependencies(subpose ${dependencies})

install(TARGETS
  pubvelsafe
  subpose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()


