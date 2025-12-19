import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robot_xacro_name = "differential_drive_robot"
    name_package = "mobile_robot"
    model_file_relative_path = "model/robot.xacro"
    world_file_relative_path = "worlds/depot_world.sdf"

    path_model_file = os.path.join(get_package_share_directory(name_package), model_file_relative_path)

    robot_description = xacro.process_file(path_model_file).toxml()

    gazebo_ros_package_launch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"))

    path_world_file = os.path.join(get_package_share_directory(name_package), world_file_relative_path)
    gazebo_launch = IncludeLaunchDescription(gazebo_ros_package_launch, launch_arguments={"gz_args": [f"-r -v -v4 {path_world_file}"], "on_exit_shutdown": "true"}.items())

    spawn_model_node_gazebo = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_xacro_name,
            "-topic", "robot_description"
        ],
        output="screen"
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    bridge_params = os.path.join(
        get_package_share_directory(name_package),
        "parameters",
        "bridge_parameters.yaml"
    )

    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}"
        ],
        output="screen"
    )

    slam_params = os.path.join(
        get_package_share_directory(name_package),
        "parameters",
        "mapper_params_online_async.yaml"
    )

    declare_launch_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params,
        description="Full path to SLAM toolbox params"
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            )
        ),
        launch_arguments={
            "slam_params_file": LaunchConfiguration("slam_params_file")
        }.items()
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(name_package),
        "rviz",
        "map.rviz"
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file]
    )

    nav2_params = os.path.join(
        get_package_share_directory(name_package),
        "parameters",
        "nav2_params.yaml"
    )

    declare_launch_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=nav2_params,
        description="Full path to Nav2 params"
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            )
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("nav2_params_file")
        }.items()
    )

    launch_description_object = LaunchDescription()

    launch_description_object.add_action(gazebo_launch)
    launch_description_object.add_action(spawn_model_node_gazebo)
    launch_description_object.add_action(node_robot_state_publisher)
    launch_description_object.add_action(start_gazebo_ros_bridge_cmd)
    launch_description_object.add_action(declare_launch_params)
    launch_description_object.add_action(slam_toolbox_launch)
    launch_description_object.add_action(nav2_launch)
    launch_description_object.add_action(rviz_launch)

    return launch_description_object
