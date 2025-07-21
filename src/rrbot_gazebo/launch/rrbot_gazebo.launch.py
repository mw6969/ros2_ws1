#Launch file to start a gazebo worls with the rrbot
#1) Launches rrbot_description/launch/publish_urdf.launch.py to publish the /robot_description
#2) Launches rrbot_gazebo/launch/rrbot_world.launch.py to start gazebo with the world rrbot_world.sdf
#3) Launches rrbot_gazebo/launch/spawn_rrbot.launch.py to spawn rrbot into the gazebo world 

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
 
    ###launch files 
    #1) publish_urdf_launch: to launch robot_state_publisher and rviz
    #2) rrbot_world_launch: to launch gazebo with the specified world
    #3) spawn_rrbot_launch: to spawn the robot in the gazebo world
    publish_urdf_launch = PathJoinSubstitution([FindPackageShare('rrbot_description'),'launch', 'publish_urdf.launch.py'])
    rrbot_world_launch = PathJoinSubstitution([FindPackageShare('rrbot_gazebo'),'launch', 'rrbot_world.launch.py'])
    spawn_rrbot_launch = PathJoinSubstitution([FindPackageShare('rrbot_gazebo'),'launch', 'spawn_rrbot.launch.py'])
    
    ###include the launch files
    publish_urdf__node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(publish_urdf_launch),
    )
    rrbot_world_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rrbot_world_launch),
    )
    spawn_rrbot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_rrbot_launch),
    )

    # Description
    ld =  LaunchDescription()

    ld.add_action(publish_urdf__node)
    ld.add_action(rrbot_world_node)
    ld.add_action(spawn_rrbot_node)

    return ld