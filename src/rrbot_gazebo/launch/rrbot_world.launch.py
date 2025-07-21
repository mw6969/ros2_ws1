#Launch file to run gazebo with the world rrbot_world

from launch import LaunchDescription
from launch.actions import TimerAction, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    print("Launching rrbot_world.launch.py **********************************")

    # Force OpenGL to run in software mode (necessary in headless environments or containers).
    set_opengl = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='True')

    #Environment variables  
    pkg_path = FindPackageShare("rrbot_gazebo")
    gazebo_models_path=PathJoinSubstitution([pkg_path,'models'])
    meshes_path=PathJoinSubstitution([FindPackageShare("rrbot_description"),'meshes'])
    
    # Set Ignition resource paths
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', 
        value=[EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),':',
                                    '/usr/share/ignition/ignition-gazebo6/worlds:',
                                    gazebo_models_path,':',
                                    meshes_path]
        #value=gazebo_models_path
    )
    
    #Gazebo world  
    simulation_world_file_path = PathJoinSubstitution(
        [pkg_path, 
        "worlds/rrbot_world.sdf"]
    )
    # Start the Gazebo node
    simulation_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', simulation_world_file_path], 
        output='screen'
    )    
    
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/rrbot_world/model/rrbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            #'/camera1_sensor/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            #'/camera1_sensor/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        remappings=[
            ('/world/rrbot_world/model/rrbot/joint_state', '/joint_states')
            #('/camera1_sensor/image_raw', '/camera/image_raw'),
            #('/camera1_sensor/camera_info', '/camera/camera_info')
        ]
    )

    # Description
    ld =  LaunchDescription()

    ld.add_action(ign_resource_path)
    ld.add_action(simulation_cmd)    
    ld.add_action(set_opengl)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld