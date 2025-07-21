# ignition_ros2_srv_bridge

This package implements a bridge to the ign gazebo create service, that allows to spawn an object into the the gazebo world.

The service wrapped by this ROS 2 server is:

- https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_interfaces/srv/SpawnEntity.srv
- https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_interfaces/srv/DeleteEntity.srv

The gazebo world name is a parameter that has to be set at the launch file.

## TODO

Extend with the other available services:

- https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_interfaces/srv/SetEntityPose.srv





  
