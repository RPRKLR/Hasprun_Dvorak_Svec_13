import sys
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.action import TimerActions, ExecuteProcess, DeclareLaunchArgument
import xacro
import launch_ros

def generate_launch_description():
    # drone_control_config_path = os.path.join(get_package_share_directory('LRS-FEI-main'), 'config', 'drone_control.yaml')
    # rviz_config_path = os.path.join(get_package_share_directory('LRS-FEI-main'), 'rviz', 'rviz_config.rviz')
    # Might need to change the package name
    pkg_share = launch_ros.substitutions.FindPackageShare(package='LRS-FEI-main').find('LRS-FEI-main')
    default_model_path = os.path.join(pkg_share, 'urdf/drone_model.urdf')
    world_path = os.path.join(pkg_share, 'world/world_name.sdf')

    use_sim_time = True

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'drone_caption', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
                                      '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                             description='Flag to enable use_sim_time'),
        spawn_entity
    ])