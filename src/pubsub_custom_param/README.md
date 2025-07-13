# The pubsub_custom_param package

Package to illustrate the ROS 2 publish/subscribe anonymous message passing using a user defined interface (package *custom_interface*) and use parameters to tune the publisher.

Used in the [Tools tutorial](https://sir.upc.edu/projects/ros2tutorials/3-tools/index.html)  of the Introduction to ROS course.

## Usage

Using the default values of the parameters:

```
$ros2 launch pubsub_custom_param pubsub_launch.py
```

Using the values of the parameters set in the launch file:

```
$ros2 launch pubsub_custom_param pubsub_param_launch.py
```

Using the values of the parameters set in a configuration file:

```
$ros2 launch pubsub_custom_param pubsub_paramfile_launch.py
```


