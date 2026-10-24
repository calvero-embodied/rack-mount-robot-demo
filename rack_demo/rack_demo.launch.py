"""Dedicated scene; leaves all installed lecture files unchanged."""
import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from demo_scene import robot_description
from simulation_files import write_files


def generate_launch_description():
    share = Path(get_package_share_directory('crane_plus_gazebo'))
    description = robot_description()
    robot_file,world_file=write_files(description,share)
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_file,
        '--gui-config', str(share/'gui/gui.config')], output='screen',
        additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
            'IGN_GAZEBO_RESOURCE_PATH': str(Path(
                get_package_share_directory('crane_plus_description')).parent)})
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description':description, 'use_sim_time':True}])
    spawn = Node(package='ros_gz_sim', executable='create',
        arguments=['-file', robot_file, '-name', 'crane_plus', '-z', '1.015'],
        output='screen')
    controllers = [Node(package='controller_manager', executable='spawner',
        arguments=[name], output='screen') for name in
        ['joint_state_broadcaster', 'crane_plus_arm_controller',
         'crane_plus_gripper_controller']]
    prepare=ExecuteProcess(cmd=['bash',str(Path(__file__).resolve().parent/'prepare_session.sh')],
                           output='screen')
    monitor=ExecuteProcess(cmd=['bash',str(Path(__file__).resolve().parent/'monitor_contacts.sh')],output='screen')
    return LaunchDescription([gazebo, rsp, monitor,
        RegisterEventHandler(OnProcessExit(target_action=controllers[1],on_exit=[prepare])),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=controllers)),
        spawn, Node(package='ros_gz_bridge', executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])])
