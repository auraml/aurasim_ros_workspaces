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
from launch.actions import GroupAction, OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationNotEquals
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from launch_pal.param_utils import parse_parametric_yaml, merge_param_files
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_description.launch_arguments import TiagoProArgs

from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    namespace: DeclareLaunchArgument = CommonArgs.namespace


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):

    right_arm_controller = OpaqueFunction(
        function=setup_arm_controllers,
        kwargs={"arm_side": "right"},
        condition=LaunchConfigurationNotEquals("arm_type_right", "no-arm"),
    )
    launch_description.add_action(right_arm_controller)

    left_arm_controller = OpaqueFunction(
        function=setup_arm_controllers,
        kwargs={"arm_side": "left"},
        condition=LaunchConfigurationNotEquals("arm_type_left", "no-arm"),
    )
    launch_description.add_action(left_arm_controller)
    torso_controller = OpaqueFunction(function=setup_torso_controllers)
    launch_description.add_action(torso_controller)

    return


def setup_torso_controllers(context, *args, **kwargs):
    torso_joint_space_controller = setup_torso_controller(
        context, "torso_joint_space_controller", load_gains_separately=True
    )

    torso_joint_space_controller_vel = setup_torso_controller(
        context, "torso_joint_space_vel_controller", load_gains_separately=True
    )

    return [torso_joint_space_controller, torso_joint_space_controller_vel]


def setup_torso_controller(context, controller_name, load_gains_separately=False):
    param_file = os.path.join(
        get_package_share_directory("tiago_pro_controller_configuration"),
        "config",
        "tsid",
        f"{controller_name}.yaml",
    )
    parsed_yaml = parse_parametric_yaml(
        source_files=[param_file], param_rewrites={})

    use_sim_time = read_launch_argument("use_sim_time", context)
    if load_gains_separately:
        sim_postfix = "_sim" if use_sim_time == "True" else ""

        gains_file = os.path.join(
            get_package_share_directory("tiago_pro_controller_configuration"),
            "config",
            "tsid",
            "gains",
            f"{controller_name}{sim_postfix}.yaml",
        )

        parsed_gains = parse_parametric_yaml(
            source_files=[gains_file], param_rewrites={}
        )

        parsed_yaml = merge_param_files([parsed_yaml, parsed_gains])

    launch_controller = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name=controller_name,
                controller_params_file=parsed_yaml,
                extra_spawner_args=["--inactive"],
            )
        ],
        forwarding=False,
    )

    return launch_controller


def setup_arm_controllers(context, arm_side, *args, **kwargs):

    cartesian_space_controller_ee_frame = setup_arm_side_controller(
        context,
        "cartesian_space_controller_ee_frame",
        arm_side,
        load_gains_separately=True,
    )

    cartesian_space_controller_robot_frame = setup_arm_side_controller(
        context,
        "cartesian_space_controller_robot_frame",
        arm_side,
        load_gains_separately=True,
    )

    cartesian_vel_ee_frame = setup_arm_side_controller(
        context, "cartesian_vel_controller_ee_frame", arm_side, load_gains_separately=True
    )
    cartesian_vel_robot_frame = setup_arm_side_controller(
        context, "cartesian_vel_controller_robot_frame", arm_side, load_gains_separately=True
    )
    joint_space_controller_vel = setup_arm_side_controller(
        context, "joint_space_controller_vel", arm_side, load_gains_separately=True
    )
    joint_space_controller = setup_arm_side_controller(
        context, "joint_space_controller", arm_side, load_gains_separately=True
    )

    return [
        cartesian_vel_ee_frame,
        cartesian_vel_robot_frame,
        joint_space_controller_vel,
        joint_space_controller,
        cartesian_space_controller_ee_frame,
        cartesian_space_controller_robot_frame,
    ]


def setup_arm_side_controller(
    context, controller_name, arm_side="right", load_gains_separately=False
):

    arm_prefix = f"arm_{arm_side}"

    side_controller_name = f"{arm_prefix}_{controller_name}"
    remappings = {"ARM_SIDE_PREFIX": arm_prefix}

    param_file = os.path.join(
        get_package_share_directory("tiago_pro_controller_configuration"),
        "config",
        "tsid",
        f"{controller_name}.yaml",
    )

    parsed_yaml = parse_parametric_yaml(
        source_files=[param_file], param_rewrites=remappings
    )

    use_sim_time = read_launch_argument("use_sim_time", context)
    if load_gains_separately:
        sim_postfix = "_sim" if use_sim_time == "True" else ""

        gains_file = os.path.join(
            get_package_share_directory("tiago_pro_controller_configuration"),
            "config",
            "tsid",
            "gains",
            f"{controller_name}{sim_postfix}.yaml",
        )

        parsed_gains = parse_parametric_yaml(
            source_files=[gains_file], param_rewrites=remappings
        )

        parsed_yaml = merge_param_files([parsed_yaml, parsed_gains])

    launch_controller = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name=side_controller_name,
                controller_params_file=parsed_yaml,
                extra_spawner_args=["--inactive"],
            )
        ],
        forwarding=False,
    )

    return launch_controller


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
