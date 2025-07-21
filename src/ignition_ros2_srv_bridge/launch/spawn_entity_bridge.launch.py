from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ignition_ros2_srv_bridge',
            executable='spawn_entity_bridge',
            parameters=[{
                'world_name': "camera_world",
            }],
            name='spawn_entity_bridge',
            output='screen'
        )
    ])
