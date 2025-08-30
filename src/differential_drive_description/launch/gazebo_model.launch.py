import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robot_xacro_name = "differential_drive_robot"
    name_package = "differential_drive_description"
    model_file_relative_path = "model/robot.xacro"

    path_model_file = os.path.join(get_package_share_directory(name_package), model_file_relative_path)

    robot_description = xacro.process_file(path_model_file).toxml()

    gazebo_ros_package_launch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"))

    gazebo_launch = IncludeLaunchDescription(gazebo_ros_package_launch, launch_arguments={"gz_args": ["-r -v -v4 empty.sdf"], "on_exit_shutdown": "true"}.items())

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

    launch_description_object = LaunchDescription()

    launch_description_object.add_action(gazebo_launch)
    launch_description_object.add_action(spawn_model_node_gazebo)
    launch_description_object.add_action(node_robot_state_publisher)
    launch_description_object.add_action(start_gazebo_ros_bridge_cmd)

    return launch_description_object