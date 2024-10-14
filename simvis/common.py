import os
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_TYPE_PARAM_NAME = "type"
RVIZ_CONFIG_PARAM_NAME = "rviz"
ROBOT_NAME_PARAM_NAME = "robot"

SIM_DIR = Path(os.environ["SIM_DIR"])
MODELS_DIR = Path(os.environ["MODELS_DIR"])


@dataclass
class SimData:
    robot_path: Path
    world_path: Path

    @property
    def rviz_conf_path(self) -> Path:
        return self.world_path.parent / "gazebo.rviz"

    @property
    def gz_bridge_conf_path(self) -> Path:
        return self.world_path.parent / "bridge.yaml"


class SimType(SimData, Enum):
    PLATFORM = (
        MODELS_DIR / "objects" / "platform" / "full_platform.urdf",
        MODELS_DIR / "worlds" / "stand" / "stand.sdf",
    )
    WALL = (
        MODELS_DIR / "flyers" / "full" / "flyer.urdf",
        MODELS_DIR / "worlds" / "wall" / "wall.sdf",
    )


def get_robot_state_publisher_generator(kwargs=None):
    def generate_robot_state_publisher(context: LaunchContext, model_path: LaunchConfiguration | Path):
        nonlocal kwargs

        if isinstance(model_path, Path):
            model_path_str = str(model_path.resolve())
        else:
            model_path_str = context.perform_substitution(model_path)
        with open(model_path_str, "r") as xml_description:
            robot_description = xml_description.read()

        parameters = [{"robot_description": robot_description, "use_sim_time": True}]
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
                **kwargs,
            )
        ]

    return generate_robot_state_publisher
