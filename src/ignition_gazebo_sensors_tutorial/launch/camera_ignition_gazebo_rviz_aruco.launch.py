#1) launches ignition_gazebo_sensors_tutorial/launch/camera_ignition_gazebo.launch.py to start gazebo with the camera_world
#2) runs rviz
#3) launches gazebo_sensors_tutorial/launch/camera_image_pipeline.launch.py to start the aruco nodes

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
        [FindPackageShare("ignition_gazebo_sensors_tutorial"), "rviz", "camera_aruco.rviz"]
    )

    #This transform is the same set at the camera_world.sdf file where the camera object is placed
    #  <pose>-0.35 0 0.54 0 1.04719551 0</pose>
    #But be careful:
    # <pose>x y z R P Y</pose>  <!-- Ignition -->
    # arguments = ['x', 'y', 'z', 'Y', 'P', 'R', 'parent_frame', 'child_frame']  <!-- ROS 2 -->
    #The frame names in the arguments, i.e. camera_link, is taken form the camera-plugin.sdf file
    camera_frame_br = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_broadcaster",
        #arguments=[x, y, z, yaw, pitch, roll, parent_frame_id, child_frame_id] - rotations apply in ZYX order
        arguments=["-0.35", "0", "0.54", "0", "1.04", "0", "world", "camera_link"],
    )

    #This is the transform between the camera frame and the optical camera frame
    #There is an internal transform between the camera frame and the camera's optical frame that follows the ROS camera optical frame convention, i.e., to align from a typical camera_link (Z up, X forward) to camera_link_optical (Z forward, X right, Y down: <pose>0 0 0 -1.5708 0 -1.5708</pose>
    #The frame names in the arguments, i.e. camera_link_optical, is taken form the camera-plugin.sdf file
    optical_camera_frame_br = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="optical_camera_broadcaster",
        #arguments=[x, y, z, yaw, pitch, roll, parent_frame_id, child_frame_id] - rotations apply in ZYX order
        arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "camera_link_optical"],
    )

    ###launch files 
    camera_world_launch = PathJoinSubstitution([FindPackageShare('ignition_gazebo_sensors_tutorial'),'launch', 'camera_ignition_gazebo.launch.py'])
    camera_image_pipeline_launch = PathJoinSubstitution([FindPackageShare('ignition_gazebo_sensors_tutorial'),'launch', 'camera_image_pipeline.launch.py'])
    
    ###include the launch files
    camera_world_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_world_launch),
    )
    camera_image_pipeline_node316 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_image_pipeline_launch),
        launch_arguments=[('marker_id', '316'),
                          ('marker_frame', 'aruco_316')],
    )    

    ###
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
    ld.add_action(camera_image_pipeline_node316)
    ld.add_action(rviz_node)

    return ld




