#Launch file to start a gazebo world with the camera
#1) launches ignition_gazebo_sensors_tutorial/launch/camera_ignition_gazebo.launch.py to start gazebo with the camera.world
#2) runs rviz

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
 
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ignition_gazebo_sensors_tutorial"), "rviz", "camera.rviz"]
    )

    #This transform is the same set at the camera_world.sdf file where the camera object is placed
    camera_frame_br = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_broadcaster",
        #arguments=[x, y, z, roll, pitch, yaw, parent_frame_id, child_frame_id]
        arguments=["-0.35", "0", "0.54", "0", "1.04", "0", "world", "camera_link"],#RPY (rotations apply in ZYX order)
    )

    #This is the transform between the camera frame and the optical camera frame
    optical_camera_frame_br = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="optical_camera_broadcaster",
        #arguments=[x, y, z, roll, pitch, yaw, parent_frame_id, child_frame_id]
        arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "camera_link_optical"],#RPY (rotations apply in ZYX order)
    )

    ###launch files 
    camera_world_launch = PathJoinSubstitution([FindPackageShare('ignition_gazebo_sensors_tutorial'),'launch', 'camera_ignition_gazebo.launch.py'])
    
    ###include the launch files
    camera_world_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_world_launch),
    )    

    ###launch rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        on_exit=Shutdown(),
    )

    # Description
    ld =  LaunchDescription()

    ld.add_action(camera_world_node)
    ld.add_action(camera_frame_br)
    ld.add_action(optical_camera_frame_br)
    ld.add_action(rviz_node)

    return ld




