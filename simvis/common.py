import os
from pathlib import Path

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODEL_PARAM_NAME = "model"
WORLD_PARAM_NAME = "world"
PLATFORM_Z_PARAM_NAME = "platform_z"

SIM_DIR = Path(os.environ["SIM_DIR"])
MODELS_DIR = Path(os.environ["MODELS_DIR"])
PLATFORM_MODEL_PATH = MODELS_DIR / "airstand" / "platform" / "platform.urdf"
RVIZ_CONFIG_PARAM_NAME = "rviz_config"


def get_robot_state_publisher_generator(kwargs=None):
    def generate_robot_state_publisher(
        context: LaunchContext, model_path: LaunchConfiguration
    ):
        nonlocal kwargs

        model_path_str = context.perform_substitution(model_path)
        with open(model_path_str, "r") as xml_description:
            robot_description = xml_description.read()

        parameters = [{"robot_description": robot_description, 'use_sim_time': True}]
        if kwargs is not None:
            if "parameters" in kwargs:
                parameters.extend(kwargs["parameters"])
                del kwargs["parameters"]
        else:
            kwargs = {}

        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=parameters,
                **kwargs
            )
        ]

    return generate_robot_state_publisher
