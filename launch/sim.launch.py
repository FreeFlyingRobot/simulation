from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simvis.common import SIM_DIR, SIM_TYPE_PARAM_NAME, ROBOT_NAME_PARAM_NAME, RVIZ_CONFIG_PARAM_NAME, SimType, SimData

pkg_simvis = Path(get_package_share_directory("simvis"))
pkg_ros_gz_sim = Path(get_package_share_directory("ros_gz_sim"))


def generate_gz_config(context: LaunchContext, sim_type: LaunchConfiguration):
    sim_type_str = context.perform_substitution(sim_type)
    sim_data: SimData = SimType[sim_type_str.upper()]

    return [
        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_ros_gz_sim / "launch" / "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": f"--gui-config {str(SIM_DIR / 'config' / 'gz.config')} \
                             --render-engine ogre {str(sim_data.world_path)}"
            }.items(),
        ),
        # RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_simvis / "launch" / "rviz.launch.py")),
            launch_arguments={RVIZ_CONFIG_PARAM_NAME: str(sim_data.rviz_conf_path)}.items(),
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        # Robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_simvis / "launch" / "rsp.launch.py")),
            launch_arguments={ROBOT_NAME_PARAM_NAME: str(sim_data.robot_path)}.items(),
        ),
        # ROS <-> GZ message bridge
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[
                {
                    "config_file": str(sim_data.gz_bridge_conf_path),
                    "qos_overrides./tf_static.publisher.durability": "transient_local",
                }
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    ld = LaunchDescription()

    # Setup to launch the simulator and Gazebo world
    ld.add_action(DeclareLaunchArgument(SIM_TYPE_PARAM_NAME, default_value="platform"))
    ld.add_action(DeclareLaunchArgument("rviz", default_value="true", description="Open RViz"))

    ld.add_action(
        OpaqueFunction(
            function=generate_gz_config,
            args=[LaunchConfiguration(SIM_TYPE_PARAM_NAME)],
        )
    )

    return ld
