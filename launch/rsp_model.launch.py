from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from simvis.common import MODEL_PARAM_NAME, PLATFORM_MODEL_PATH, get_robot_state_publisher_generator


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name=MODEL_PARAM_NAME,
            default_value=str(PLATFORM_MODEL_PATH),
            description="Absolute path to urdf model",
        )
    )
    ld.add_action(
        OpaqueFunction(
            function=get_robot_state_publisher_generator(),
            args=[LaunchConfiguration(MODEL_PARAM_NAME)],
        )
    )

    return ld
