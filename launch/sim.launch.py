from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simvis.common import SIM_DIR, MODELS_DIR, RVIZ_CONFIG_PARAM_NAME


def generate_launch_description():
    ld = LaunchDescription()

    pkg_simvis = Path(get_package_share_directory('simvis'))
    pkg_ros_gz_sim = Path(get_package_share_directory('ros_gz_sim'))

    # Setup to launch the simulator and Gazebo world
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_ros_gz_sim / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f"--gui-config {str(SIM_DIR / 'config' / 'gz.config')} \
                         --render-engine ogre \
                         {str(MODELS_DIR / 'airstand' / 'infinistand.sdf')}"
        }.items(),
    ))

    ld.add_action(ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/airstand_world/create', '--reqtype', 'gz.msgs.EntityFactory', '--reptype',
            'gz.msgs.Boolean', '--timeout', '1000', '--req',
            f"""'sdf_filename: "{str(MODELS_DIR / 'airstand'/ 'platform' / 'platform.urdf')}", name: "platform", pose: {{position: {{z: 0.295}}}}'"""
        ],
        shell=True,
        output='screen'
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
