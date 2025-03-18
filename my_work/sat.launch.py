from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnExecutionComplete
import os
from os import environ

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ld = LaunchDescription()

    canadarm_demos_path = get_package_share_directory('canadarm')
    simulation_models_path = get_package_share_directory('simulation')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), canadarm_demos_path])}

    leo_model = os.path.join(canadarm_demos_path, 'worlds', 'sat.world')

    start_world = ExecuteProcess(
        cmd=['gz sim', leo_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )

    return LaunchDescription([
        start_world,
    ])
