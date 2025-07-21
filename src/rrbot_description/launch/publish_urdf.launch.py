#Launches rrbot_description/launch/publish_urdf.launch.py to publish the /robot_description

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    LogInfo,
    Shutdown,
)


def generate_launch_description():

    # rrbot description
    description_package = LaunchConfiguration('description_package', default='rrbot_description')
    description_file = LaunchConfiguration('description_file', default='rrbot.xacro')

    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rrbot_description"), "rviz", "rrbot.rviz"]
    )

    #robot_description_content: The URDF XML content as a string, i.e. the raw URDF string dynamically generated from the .xacro file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "name:=",
            "rrbot",
        ]
    )
    #The robot_description information as a dictionary to be passed to the Nodes as parameters
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": False}, robot_description],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        on_exit=Shutdown(),
    )

    log_robot_description = LogInfo(
            msg=robot_description.get("robot_description"),
    )

    # Description
    ld =  LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(log_robot_description)    

    return ld