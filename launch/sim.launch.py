from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simvis.common import SIM_DIR, PLATFORM_MODEL_PATH, MODELS_DIR, RVIZ_CONFIG_PARAM_NAME, WORLD_PARAM_NAME, ROBOT_NAME_PARAM_NAME


pkg_simvis = Path(get_package_share_directory('simvis'))
pkg_ros_gz_sim = Path(get_package_share_directory('ros_gz_sim'))


def generate_gz_sim(context: LaunchContext, world_path: LaunchConfiguration):
    world_path_str = context.perform_substitution(world_path)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_ros_gz_sim / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f"--gui-config {str(SIM_DIR / 'config' / 'gz.config')} \
                         --render-engine ogre \
                         {str(MODELS_DIR / world_path_str)}"
        }.items(),
    )]


def generate_spawner(context: LaunchContext, z: LaunchConfiguration, robot_model: LaunchConfiguration):
    z_str = context.perform_substitution(z)
    robot_model_str = context.perform_substitution(robot_model)

    return [ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/airstand_world/create', '--reqtype', 'gz.msgs.EntityFactory', '--reptype',
            'gz.msgs.Boolean', '--timeout', '1000', '--req',
            f"""'sdf_filename: "{robot_model_str}", name: "robot", pose: {{position: {{z: {z_str}}}}}'"""
        ],
        shell=True,
        output='screen'
    )]


def generate_launch_description():
    ld = LaunchDescription()

    # Setup to launch the simulator and Gazebo world
    ld.add_action(DeclareLaunchArgument(
        WORLD_PARAM_NAME,
        default_value='worlds/stand/stand.sdf',
        description="Gazebo world SDF"
    ))
    ld.add_action(OpaqueFunction(
        function=generate_gz_sim,
        args=[LaunchConfiguration(WORLD_PARAM_NAME)],
    ))

    ld.add_action(DeclareLaunchArgument(
        ROBOT_NAME_PARAM_NAME,
        default_value=str(PLATFORM_MODEL_PATH),
        description="Robot model path"
    ))

    # Rviz
    ld.add_action(DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_simvis / 'launch' / 'simple_rviz.launch.py')),
        launch_arguments={RVIZ_CONFIG_PARAM_NAME: str(SIM_DIR / "config" / "gazebo.rviz")}.items(),
        condition=IfCondition(LaunchConfiguration('rviz'))
    ))

    # Robot state publisher
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_simvis / 'launch' / 'rsp_model.launch.py'))
    ))

    # Bridge ROS topics and Gazebo messages for establishing communication
    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': str(SIM_DIR / 'config' / 'ros_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            output='screen'
        )
    )

    return ld
