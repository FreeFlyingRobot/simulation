import os
from pathlib import Path

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_robot_state_publisher(
    context: LaunchContext, model_path: LaunchConfiguration
):
    model_path_str = context.perform_substitution(model_path)
    with open(model_path_str, "r") as urdf_file:
        robot_description = urdf_file.read()
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": robot_description,
                }
            ],
        )
    ]


def generate_launch_description():
    ld = LaunchDescription()

    SIM_DIR = os.environ["SIM_DIR"]
    default_model_path = (
        Path(SIM_DIR) / "models" / "airstand" / "platform" / "platform.urdf"
    )
    default_rviz_config_path = Path(SIM_DIR) / "rviz" / "stand.rviz"

    GUI_PARAM_NAME = "jsp_gui"
    RVIZ_CONFIG_PARAM_NAME = "rviz_config"
    MODEL_PARAM_NAME = "model"

    ld.add_action(
        DeclareLaunchArgument(
            name=GUI_PARAM_NAME,
            default_value="false",
            choices=["true", "false"],
            description="Flag to enable joint_state_publisher_gui",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name=RVIZ_CONFIG_PARAM_NAME,
            default_value=str(default_rviz_config_path),
            description="Absolute path to rviz config file",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name=MODEL_PARAM_NAME,
            default_value=str(default_model_path),
            description="Absolute path to urdf model",
        )
    )

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("jsp_gui")),
        )
    )
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration(RVIZ_CONFIG_PARAM_NAME)],
        )
    )
    ld.add_action(
        OpaqueFunction(
            function=generate_robot_state_publisher,
            args=[LaunchConfiguration(MODEL_PARAM_NAME)],
        )
    )

    return ld
