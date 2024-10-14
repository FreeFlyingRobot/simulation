from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from simvis.common import ROBOT_NAME_PARAM_NAME, get_robot_state_publisher_generator


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name=ROBOT_NAME_PARAM_NAME,
            description="Absolute path to urdf model",
        )
    )
    ld.add_action(
        OpaqueFunction(
            function=get_robot_state_publisher_generator(),
            args=[LaunchConfiguration(ROBOT_NAME_PARAM_NAME)],
        )
    )

    return ld
