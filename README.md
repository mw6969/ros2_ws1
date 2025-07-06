# ROS2 Tutorials Practical Exercises Repository

This repository contains the practical assignments for the "Introduction to ROS 2" course (240AR060) offered at UPC. Each assignment is maintained in its own Git branch, allowing you to explore and work on each tutorial independently.

## Branches

* **colcon\_ws1**: ROS basic concepts and development tools
* **colcon\_ws2**: Communications using topics
* **colcon\_ws3**: Tools
* **colcon\_ws4**: Communications using services
* **colcon\_ws5**: Simulation
* **colcon\_ws6**: Communications using actions
* **colcon\_ws7**: Robot control
* **colcon\_wsFinalWork**: Final project

## Practical Assignments Overview

### Tutorial 1: ROS basic concepts and development tools

In this tutorial, you will:

* Learn the fundamental concepts of ROS 2 and its middleware architecture.
* Explore core components and command-line tools.
* Understand the ROS 2 ecosystem and package structure.
* Create your first ROS 2 workspace and package.
* **Exercise 1**: Follow the instruction in [Setup](https://sir.upc.edu/projects/ros2tutorials/appendices/setup/index.html#setupappendix-label) in order to set up your git repository to implement the solution of the proposed exercises.

### Tutorial 2: Communications using topics

This tutorial focuses on ROS 2 topics:

* Dive into the `rclcpp::Node` class and its usage.
* Implement a publisher and a subscriber node.
* Work with standard, common, and custom message types.
* Record and play back data with **rosbag**.
* Configure Quality of Service (QoS) settings and executors.
* Complete **Exercise 2** to solidify your understanding.

### Tutorial 3: Tools

Learn about the essential ROS 2 development utilities:

* Use the ROS 2 launch system to start complex graphs.
* Manage node parameters dynamically.
* Model robots with URDF and work with the `tf2` library.
* Visualize robot state in RViz.
* Complete **Exercise 3a** and **3b** to apply these tools in practice.

### Tutorial 4: Communications using services

Understand request/reply patterns in ROS 2:

* Explore ROS 2 service communication features and interfaces.
* Implement a service server and client in C++.
* Handle callbacks and callback groups for concurrency.
* Complete **Exercise 4** to build a simple service-based interaction.

### Tutorial 5: Simulation

Discover how to simulate robots with Gazebo:

* Learn Gazebo basics and integration with ROS 2.
* Use camera images and the ArUco library for pose estimation.
* Complete **Exercise 5** to simulate a camera-driven detection pipeline.

### Tutorial 6: Communications using actions

Work with ROS 2 actions for long-running tasks:

* Examine action communication features and interfaces.
* Use the `rclcpp_action` library to implement action clients and servers.
* Study the pan-tilt example demonstrating goal management and feedback.

### Tutorial 7: Robot control

Implement hardware abstraction and control frameworks:

* Get an overview of `ros2_control` and its architecture.
* Understand the Hardware Abstraction Layer (HAL) and controller management.
* Configure and simulate controllers in Gazebo.
* Use ROS 2 actions to command controllers.

### Final Work: Integration and Evaluation

The final assignment brings together all course topics:

* Define project motivation and objectives.
* Set up the development environment and provided packages.
* Develop, test, and evaluate a complete ROS 2 application.
* Follow best practices in code structure and documentation.

---

Feel free to switch to any branch to explore or work on the corresponding practical assignment. Happy coding with ROS 2!
