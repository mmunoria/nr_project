import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    world_path = os.path.join(
        os.path.expanduser("~"),
        "NR",
        "final_ws",
        "worlds",
        "ring_world.world"
    )

    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    turtlebot3_description_dir = get_package_share_directory("turtlebot3_description")

    model_path = os.path.join(turtlebot3_gazebo_dir, "models")

    xacro_file = os.path.join(
        turtlebot3_description_dir,
        "urdf",
        "turtlebot3_waffle_pi.urdf"
    )

    # If your package only has xacro, change to .urdf.xacro and use xacro
    robot_description = {
        "robot_description": Command(["xacro ", xacro_file])
    }

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=model_path
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world_path}"
        }.items()
    )

    spawn_turtlebot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "turtlebot3_waffle_pi",
            "-file", os.path.join(
                turtlebot3_gazebo_dir,
                "models",
                "turtlebot3_waffle_pi",
                "model.sdf"
            ),
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1"
        ],
        output="screen"
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(
                turtlebot3_gazebo_dir,
                "params",
                "turtlebot3_waffle_pi_bridge.yaml"
            )
        }],
        output="screen"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    rviz_config = os.path.join(
        turtlebot3_description_dir,
        "rviz",
        "model.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        gazebo_resource_path,
        gz_sim,
        bridge_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_turtlebot,
        rviz_node,
    ])