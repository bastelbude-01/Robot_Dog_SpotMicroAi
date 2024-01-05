import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition


def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("spot_description"),
                    "urdf",
                    "spot.urdf.xacro",
                ),
            ]
        ),
        value_type=str,
    )

    

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                get_package_share_directory("spot_controller"),
                "config",
                "spot_controllers.yaml",
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    leg_fr_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr_leg_controller",
                   "--controller-manager", "/controller_manager"],
    )
    leg_fl_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fl_leg_controller",
                   "--controller-manager", "/controller_manager"],
    )
    leg_hr_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hr_leg_controller",
                   "--controller-manager", "/controller_manager"],
    )
    leg_hl_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hl_leg_controller",
                   "--controller-manager", "/controller_manager"],
    )

    

    return LaunchDescription(
        [    
            controller_manager,
            joint_state_broadcaster_spawner,
            leg_fr_controller_spawner,
            leg_fl_controller_spawner,
            leg_hr_controller_spawner,
            leg_hl_controller_spawner,
        ]
    )
