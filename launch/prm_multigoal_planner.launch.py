# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("prm_multigoal_planner"), "config", "prm_multigoal_planner.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params", default_value=default_params),
            DeclareLaunchArgument("ik_solver_ns", default_value="ik_solver"),
            DeclareLaunchArgument("planning_scene_service", default_value="/get_planning_scene"),
            DeclareLaunchArgument("action_name", default_value="prm_multigoal_plan"),
            Node(
                package="prm_multigoal_planner",
                executable="prm_multigoal_planner_node",
                name="prm_multigoal_planner",
                output="screen",
                parameters=[
                    LaunchConfiguration("params"),
                    {
                        "ik_solver_ns": LaunchConfiguration("ik_solver_ns"),
                        "planning_scene_service": LaunchConfiguration("planning_scene_service"),
                        "action_name": LaunchConfiguration("action_name"),
                    },
                ],
            ),
        ]
    )
