# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals

from launch_pal.arg_utils import LaunchArgumentsBase
from tiago_pro_description.launch_arguments import TiagoProArgs

from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left

    cmd_vel: DeclareLaunchArgument = DeclareLaunchArgument(
        name='cmd_vel',
        default_value='input_joy/cmd_vel',
        description='Joystick cmd_vel topic')


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        parameters=[os.path.join(get_package_share_directory('tiago_pro_bringup'), 'config',
                                 'joy_teleop', "joy_teleop.yaml")],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel'))])

    launch_description.add_action(joy_teleop_node)

    pkg_dir = get_package_share_directory('tiago_pro_bringup')

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joystick',
        parameters=[os.path.join(pkg_dir, 'config', 'joy_teleop', 'joy_config.yaml')])

    launch_description.add_action(joy_node)

    joystick_analyzer = Node(
        package='diagnostic_aggregator',
        executable='add_analyzer',
        namespace='joystick',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(pkg_dir, 'config', 'joy_teleop', 'joystick_analyzers.yaml')
        ],
    )
    launch_description.add_action(joystick_analyzer)

    # starting safe command node for joystick teleop
    safe_command_head = Node(
        package='collision_aware_joint_trajectory_wrapper',
        executable='safe_command_node',
        name='safe_command_node',
        namespace='head_controller',
        output='screen',
        parameters=[{
            'controller_name': 'head_controller'
        }],
        remappings=[
            ('/head_controller/robot_description', '/robot_description'),
            ('/head_controller/robot_description_semantic', '/robot_description_semantic'),
        ]
    )

    launch_description.add_action(safe_command_head)

    safe_command_torso = Node(
        package='collision_aware_joint_trajectory_wrapper',
        executable='safe_command_node',
        name='safe_command_node',
        namespace='torso_controller',
        output='screen',
        parameters=[{
            'controller_name': 'torso_controller'
        }],
        remappings=[
            ('/torso_controller/robot_description', '/robot_description'),
            ('/torso_controller/robot_description_semantic', '/robot_description_semantic'),
        ]
    )

    launch_description.add_action(safe_command_torso)

    torso_incrementer_server = Node(
        package='joy_teleop',
        executable='incrementer_server',
        name='incrementer',
        namespace='torso_controller',
        remappings=[('joint_trajectory', 'safe_command')])

    launch_description.add_action(torso_incrementer_server)

    head_incrementer_server = Node(
        package='joy_teleop',
        executable='incrementer_server',
        name='incrementer',
        namespace='head_controller',
        remappings=[('joint_trajectory', 'safe_command')])

    launch_description.add_action(head_incrementer_server)

    gripper_incrementer_server = Node(
        package='joy_teleop',
        executable='incrementer_server',
        name='incrementer',
        namespace='gripper_right_controller',
        condition=LaunchConfigurationEquals('end_effector_right', 'pal-pro-gripper'))

    launch_description.add_action(gripper_incrementer_server)

    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
