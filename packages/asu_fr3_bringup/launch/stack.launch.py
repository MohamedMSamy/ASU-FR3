#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from asu_fr3_bringup import description, overlay, policy


def robot_description_dependent_nodes_spawner(
    context: LaunchContext,
    robot_ip,
    arm_id,
    use_fake_hardware,
    fake_sensor_commands,
    load_gripper,
    arm_prefix,
    start_robot_state_publisher,
):
    mode_str = context.launch_configurations.get("mode", "mujoco")
    if mode_str not in ("mujoco", "isaac", "real"):
        raise RuntimeError(f"unknown mode '{mode_str}' (use mujoco, isaac or real)")
    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    arm_prefix_str = context.perform_substitution(arm_prefix)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    eff = overlay.get_effective()
    robot_description = description.render_robot_description(
        eff,
        robot_ip=robot_ip_str,
        arm_id=arm_id_str,
        arm_prefix=arm_prefix_str,
        load_gripper=load_gripper_str,
        use_fake_hardware=use_fake_hardware_str,
        fake_sensor_commands=fake_sensor_commands_str,
        hardware="isaac" if mode_str == "isaac" else "",
        mode=mode_str,
    )

    controllers_name = (
        f"{arm_prefix_str}_controllers.yaml"
        if arm_prefix_str != ""
        else description.controllers_name(mode_str)
    )
    franka_controllers = str(eff.file("config", controllers_name))
    print(policy.check(mode_str, franka_controllers, description.scene_name(mode_str), eff), flush=True)   # refuses a wrong gravity setup before anything starts
    print(
        eff.report(
            {
                "controllers": ("config", controllers_name),
                "scene": ("models", description.scene_name(mode_str)),
                "urdf": ("models", "fr3_single.urdf.xacro"),
            }
        ),
        flush=True,
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(start_robot_state_publisher),
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                franka_controllers,
                {"robot_description": robot_description},
            ],
            remappings=[("joint_states", "franka/joint_states")],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
            on_exit=Shutdown(),
        ),
    ]


def generate_launch_description():
    arm_id_parameter_name = "arm_id"
    arm_prefix_parameter_name = "arm_prefix"
    robot_ip_parameter_name = "robot_ip"
    load_gripper_parameter_name = "load_gripper"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    use_rviz_parameter_name = "use_rviz"
    start_robot_state_publisher_name = "start_robot_state_publisher"

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    arm_prefix = LaunchConfiguration(arm_prefix_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    mode = LaunchConfiguration("mode")
    # mujoco = simulated hardware plugin (crisp_mujoco_sim); real = franka_hardware.
    # mujoco = crisp_mujoco_sim, isaac = topic-based plugin (no real hardware either way), real = franka_hardware.
    use_fake_hardware = PythonExpression(["'true' if '", mode, "' in ('mujoco', 'isaac') else 'false'"])
    # isaac mode runs the controllers only: no Franka-specific nodes (gripper, hand adapter, external torques).
    def not_isaac():
        return IfCondition(PythonExpression(["'", mode, "' != 'isaac'"]))
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    start_robot_state_publisher = LaunchConfiguration(start_robot_state_publisher_name)

    # RViz profile from the config overlay (config/asu_fr3.rviz; RVIZ_CONFIG picks another file in config/).
    rviz_file = str(overlay.get_effective().file("config", os.environ.get("RVIZ_CONFIG", "asu_fr3.rviz")))

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ip,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            arm_prefix,
            start_robot_state_publisher,
        ],
    )

    launch_description = LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="mujoco",
                description="mujoco (MuJoCo plugin), isaac (controllers only; Isaac Sim over ROS topics) or real (franka_hardware).",
            ),
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                default_value="172.16.0.2",
                description="Hostname or IP address of the robot (used in real mode).",
            ),
            DeclareLaunchArgument(
                arm_id_parameter_name,
                default_value="fr3",
                description="ID of the type of arm used. Supported values: fer, fr3, fp3",
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value="false",
                description="Visualize the robot in Rviz",
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value="false",
                description="Fake sensor commands (only with simulated hardware).",
            ),
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value="true",
                description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded "
                "without an end-effector.",
            ),
            DeclareLaunchArgument(
                arm_prefix_parameter_name,
                default_value="",
                description="The prefix of the arm.",
            ),
            DeclareLaunchArgument(
                start_robot_state_publisher_name,
                default_value="true",
                description="",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[
                    {
                        "source_list": [
                            "franka/joint_states",
                            "franka_gripper/joint_states",
                        ],
                        "rate": 1000,
                    }
                ],
                condition=not_isaac(),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[{"source_list": ["franka/joint_states"], "rate": 1000}],
                condition=IfCondition(PythonExpression(["'", mode, "' == 'isaac'"])),
            ),
            robot_description_dependent_nodes_spawner_opaque_function,
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["cartesian_impedance_controller", "--inactive"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_impedance_controller", "--inactive"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["twist_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["pose_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["external_torques_broadcaster"],
                output="screen",
                condition=not_isaac(),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["torque_feedback_controller", "--inactive"],
                output="screen",
                condition=not_isaac(),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gravity_compensation", "--inactive"],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_gripper"),
                                "launch",
                                "gripper.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    robot_ip_parameter_name: robot_ip,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    "namespace": arm_prefix,
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", load_gripper, "' == 'true' and '", mode, "' != 'isaac'"])
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
                condition=IfCondition(use_rviz),
            ),
            Node(
                package="asu_fr3_bringup",
                executable="target_viz",
                name="target_viz",
                condition=IfCondition(use_rviz),
            ),
            Node(
                package="asu_fr3_bringup",
                executable="crisp_py_franka_hand_adapter",
                name="crisp_py_franka_hand_adapter",
                output="screen",
                condition=not_isaac(),
            ),
        ]
    )

    return launch_description
