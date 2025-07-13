from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch argument to optionally launch the second subscriber
    launch_second_sub = DeclareLaunchArgument(
        'launch_second_subscriber',
        default_value='true',
        description='Whether to launch the second subscriber node'
    )

    return LaunchDescription([
        launch_second_sub,

        # Publisher in namespace pub1
        Node(
            package='pubsub_custom_param',
            executable='publisher',
            name='pub1_node',
            namespace='pub1',
            output='screen'
        ),

        # Publisher in namespace pub2
        Node(
            package='pubsub_custom_param',
            executable='publisher',
            name='pub2_node',
            namespace='pub2',
            output='screen'
        ),

        # Subscriber remapped to pub1's topic
        Node(
            package='pubsub_custom_param',
            executable='subscriber',
            name='subscriber1',
            output='screen',
            remappings=[('/topic', '/pub1/topic')]
        ),

        # Optional subscriber remapped to pub2's topic
        Node(
            package='pubsub_custom_param',
            executable='subscriber',
            name='subscriber2',
            output='screen',
            remappings=[('/topic', '/pub2/topic')],
            condition=IfCondition(LaunchConfiguration('launch_second_subscriber'))
        )
    ])
