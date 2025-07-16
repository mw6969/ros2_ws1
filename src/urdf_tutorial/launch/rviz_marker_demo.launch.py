# rviz_marker_demo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # --- Setup parameters ---
    launch_rviz        = LaunchConfiguration("launch_rviz")
    prefix             = LaunchConfiguration("prefix", default="")
    ur_type            = LaunchConfiguration("ur_type", default="ur3")
    description_package= LaunchConfiguration("description_package", default="ur_description")
    description_file   = LaunchConfiguration("description_file", default="ur.urdf.xacro")

    # --- Forming robot_description via xacro ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " name:=ur",
        " ur_type:=", ur_type,
        " prefix:=", prefix,
    ])
    robot_description = {"robot_description": robot_description_content}

    # --- Nodes ---
    # TF world→assembly_frame
    world_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_broadcaster",
        arguments=["0.3","0","0","0","0","0","1","world","assembly_frame"],
    )

    # Markers
    marker_publisher = Node(
        package="urdf_tutorial",
        executable="rviz_node",
        name="marker_publisher_node",
        output="screen",
    )

    # Prepare frame grasp 11 for object 11
    grasp_11_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="grasp_11_broadcaster",
        arguments=[
            "0.0", "0.0", "0.15",  # offset 15 cm in Z
            "1", "0", "0", "0",    # quaternion - 180° around X
            "tf_11",               # parent
            "grasp_11"             # child
        ],
    )

    # Robot from URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": False}, robot_description],
    )
    joint_state_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # RViz config
    pkg_share = FindPackageShare("urdf_tutorial")
    merged_rviz = PathJoinSubstitution([pkg_share, "rviz", "merged_visualization.rviz"])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", merged_rviz],
        condition=IfCondition(launch_rviz),
        on_exit=Shutdown(),
    )

    # Robot UR3 offset by +3.0 in X
    robot_offset_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="robot_offset_broadcaster",
        arguments=[
            "0.0", "0.0", "0.0",
            "0", "0", "0", "1",
            "assembly_frame",
            "base_link"
        ],
    )

    return [
        robot_offset_broadcaster,
        world_broadcaster,
        marker_publisher,
        grasp_11_broadcaster,
        robot_state_publisher,
        joint_state_gui,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch RViz?"),
        OpaqueFunction(function=launch_setup),
    ])
