#Launch file to spawn the robot in /robot_description
#We can configure the pose and the name of the spawned robot

from launch import LaunchDescription
from launch.substitutions import Command,  FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    print("Launching spawn_rrbot_world.launch.py **********************************")

    #robot base pose
    position = [0.0, 0.0, 0.02] #XYZ
    orientation = [0.0, 0.0, 0.0] #RPY

    entity_name="rrbot"

    # rrbot description
    description_package = LaunchConfiguration('description_package', default='rrbot_description')
    description_file = LaunchConfiguration('description_file', default='rrbot.xacro')
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


    #Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x',str(position[0]),
                   '-y',str(position[0]),
                   '-z',str(position[2]),
                   '-R',str(orientation[0]),
                   '-P',str(orientation[1]),
                   '-Y',str(orientation[2]),
                   '-string', robot_description_content
                    ]
    )

    #create and return launch description object
    return LaunchDescription(
        [
            spawn_robot,
        ]
    )