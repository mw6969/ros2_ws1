from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    print("Launching camera_ignition_gazebo.launch.py **********************************")
    #Environment variables  
    pkg_path = FindPackageShare("ignition_gazebo_sensors_tutorial")
    
    # Force OpenGL to run in software mode (necessary in headless environments or containers).
    set_opengl = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='True')

    # Set Ignition resource paths
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=PathJoinSubstitution([pkg_path, "models"])
    )

    # The world to simulate: change this to your world file
    simulation_world_file_path = PathJoinSubstitution(
        [pkg_path, 
        "worlds/camera_world.sdf"]
    )

    # Start the Gazebo node
    simulation_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', simulation_world_file_path], 
        output='screen'
    )
   
    # Start the Gazebo-ROS bridge node
    # The names of the ign topics to be bridged can be seen by opening the camera_world.sdf file 
    # (i.e. ign gazebo camera_world) and typing: "ign topic -l".
    # These names are directly bridged to ROS 2, so they are remapped to simpler names
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',  
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/camera_world/model/camera1/link/camera_link/sensor/camera1_sensor/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/world/camera_world/model/camera1/link/camera_link/sensor/camera1_sensor/image@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        remappings=[
            ('/world/camera_world/model/camera1/link/camera_link/sensor/camera1_sensor/image', '/camera/image'),
            ('/world/camera_world/model/camera1/link/camera_link/sensor/camera1_sensor/camera_info', '/camera/camera_info')
        ]
    )

    #Start spawn_entity_bridge node from the ignition_ros2_srv_bridge package
    #Start delete_entity_bridge node from the ignition_ros2_srv_bridge package
    #The parameter world_name is the name of the world in worlds/camera_world.sdf file
    spawn_bridge_node = Node(
        package='ignition_ros2_srv_bridge',
        executable='spawn_entity_bridge',
        parameters=[{
                'world_name': "camera_world",
        }],
        name='spawn_entity_bridge',
        output='screen'
    )

    delete_bridge_node = Node(
        package='ignition_ros2_srv_bridge',
        executable='delete_entity_bridge',
        parameters=[{
            'world_name': "camera_world",
        }],
        name='delete_entity_bridge',
        output='screen'   
    )

    ld = LaunchDescription()
    ld.add_action(ign_resource_path)
    ld.add_action(simulation_cmd)
    ld.add_action(set_opengl)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(spawn_bridge_node)
    ld.add_action(delete_bridge_node)

    return ld