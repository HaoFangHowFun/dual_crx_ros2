# SPDX-License-Identifier: Apache-2.0

import os
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _load_robot_placement():
    config_path = os.path.join(
        get_package_share_directory("dual_crx_description"),
        "config",
        "robot_placement.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _load_mock_initial_positions():
    config_path = os.path.join(
        get_package_share_directory("dual_crx_bringup"),
        "config",
        "mock_initial_positions.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _as_string(values):
    return " ".join(str(value) for value in values)


def _mock_arm_nodes(
    *,
    namespace,
    prefix,
    child_link,
    placement,
    initial_positions,
    ros2_control_config,
):
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dual_crx_bringup"), "urdf", "crx5ia_mock.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "child_link:=",
            child_link,
            " ",
            "origin_xyz:='",
            str(placement["xyz"][0]),
            " ",
            str(placement["xyz"][1]),
            " ",
            str(placement["xyz"][2]),
            "' ",
            "origin_rpy:='",
            str(placement["rpy"][0]),
            " ",
            str(placement["rpy"][1]),
            " ",
            str(placement["rpy"][2]),
            "' ",
            "initial_j1:=",
            str(initial_positions["j1"]),
            " ",
            "initial_j2:=",
            str(initial_positions["j2"]),
            " ",
            "initial_j3:=",
            str(initial_positions["j3"]),
            " ",
            "initial_j4:=",
            str(initial_positions["j4"]),
            " ",
            "initial_j5:=",
            str(initial_positions["j5"]),
            " ",
            "initial_j6:=",
            str(initial_positions["j6"]),
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            value=robot_description_content,
            value_type=str,
        )
    }

    ros_parameters = [
        robot_description,
        ParameterFile(ros2_control_config, allow_substs=True),
    ]

    controller_manager_name_argument = f" -c /{namespace}/controller_manager"

    return [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            output="both",
            parameters=ros_parameters,
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            output="both",
            parameters=[robot_description],
        ),
        Node(
            package="slider_publisher",
            executable="slider_gui_node",
            namespace=namespace,
            name="slider_gui_node",
            output="both",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 joint_state_broadcaster",
                controller_manager_name_argument,
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 joint_trajectory_controller",
                controller_manager_name_argument,
            ],
            shell=True,
            output="screen",
        ),
    ]


def launch_setup(context, *args, **kwargs):
    placement = _load_robot_placement()
    mock_initial_positions = _load_mock_initial_positions()
    bringup_prefix = get_package_prefix("dual_crx_bringup")

    use_mock = LaunchConfiguration("use_mock")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_control_server = LaunchConfiguration("launch_control_server")
    launch_servo = LaunchConfiguration("launch_servo")
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    left_ros2_control_config = LaunchConfiguration("left_ros2_control_config")
    right_ros2_control_config = LaunchConfiguration("right_ros2_control_config")
    left_physical_ros2_control_config = LaunchConfiguration(
        "left_physical_ros2_control_config"
    )
    right_physical_ros2_control_config = LaunchConfiguration(
        "right_physical_ros2_control_config"
    )
    left_motion_control = LaunchConfiguration("left_motion_control")
    right_motion_control = LaunchConfiguration("right_motion_control")
    left_gpio_config_package = LaunchConfiguration("left_gpio_config_package")
    right_gpio_config_package = LaunchConfiguration("right_gpio_config_package")
    left_gpio_config_path = LaunchConfiguration("left_gpio_config_path")
    right_gpio_config_path = LaunchConfiguration("right_gpio_config_path")

    left_xyz = _as_string(placement["left_arm"]["xyz"])
    left_rpy = _as_string(placement["left_arm"]["rpy"])
    right_xyz = _as_string(placement["right_arm"]["xyz"])
    right_rpy = _as_string(placement["right_arm"]["rpy"])
    merger_script = os.path.join(
        bringup_prefix, "lib", "dual_crx_bringup", "joint_state_merger.py"
    )

    launch_items = [
        ExecuteProcess(
            cmd=[
                "python3",
                merger_script,
                "--ros-args",
                "-p",
                "left_topic:=/left_arm/joint_states",
                "-p",
                "right_topic:=/right_arm/joint_states",
                "-p",
                "output_topic:=/joint_states",
                "-p",
                "publish_period:=0.05",
            ],
            output="log",
            shell=False,
        ),
    ]

    if use_mock.perform(context) == "true":
        launch_items.extend(
            _mock_arm_nodes(
                namespace="left_arm",
                prefix="left_",
                child_link="left_ee_mount",
                placement=placement["left_arm"],
                initial_positions=mock_initial_positions["left_arm"],
                ros2_control_config=left_ros2_control_config,
            )
        )
        launch_items.extend(
            _mock_arm_nodes(
                namespace="right_arm",
                prefix="right_",
                child_link="right_ee_mount",
                placement=placement["right_arm"],
                initial_positions=mock_initial_positions["right_arm"],
                ros2_control_config=right_ros2_control_config,
            )
        )
    else:
        physical_launch = PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_physical_control.launch.py",
                ]
            )
        )
        launch_items.extend(
            [
                IncludeLaunchDescription(
                    physical_launch,
                    launch_arguments={
                        "robot_ip": left_robot_ip,
                        "robot_model": "crx5ia",
                        "robot_series": "crx",
                        "ros2_control_config": left_physical_ros2_control_config,
                        "gpio_config_package": left_gpio_config_package,
                        "gpio_config_path": left_gpio_config_path,
                        "motion_control": left_motion_control,
                        "prefix": "left_",
                        "namespace": "left_arm",
                        "child_link": "left_ee_mount",
                        "launch_rviz": "false",
                        "origin_x": str(placement["left_arm"]["xyz"][0]),
                        "origin_y": str(placement["left_arm"]["xyz"][1]),
                        "origin_z": str(placement["left_arm"]["xyz"][2]),
                        "origin_rr": str(placement["left_arm"]["rpy"][0]),
                        "origin_rp": str(placement["left_arm"]["rpy"][1]),
                        "origin_ry": str(placement["left_arm"]["rpy"][2]),
                    }.items(),
                ),
                IncludeLaunchDescription(
                    physical_launch,
                    launch_arguments={
                        "robot_ip": right_robot_ip,
                        "robot_model": "crx5ia",
                        "robot_series": "crx",
                        "ros2_control_config": right_physical_ros2_control_config,
                        "gpio_config_package": right_gpio_config_package,
                        "gpio_config_path": right_gpio_config_path,
                        "motion_control": right_motion_control,
                        "prefix": "right_",
                        "namespace": "right_arm",
                        "child_link": "right_ee_mount",
                        "launch_rviz": "false",
                        "origin_x": str(placement["right_arm"]["xyz"][0]),
                        "origin_y": str(placement["right_arm"]["xyz"][1]),
                        "origin_z": str(placement["right_arm"]["xyz"][2]),
                        "origin_rr": str(placement["right_arm"]["rpy"][0]),
                        "origin_rp": str(placement["right_arm"]["rpy"][1]),
                        "origin_ry": str(placement["right_arm"]["rpy"][2]),
                    }.items(),
                ),
            ]
        )

    urdf_file = os.path.join(
        get_package_share_directory("dual_crx_moveit_config"),
        "config",
        "dual_crx.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("dual_crx", package_name="dual_crx_moveit_config")
        .robot_description(
            file_path=urdf_file,
            mappings={
                "left_xyz": left_xyz,
                "left_rpy": left_rpy,
                "right_xyz": right_xyz,
                "right_rpy": right_rpy,
            },
        )
        .robot_description_semantic(file_path="config/dual_crx.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    if use_mock.perform(context) == "true":
        for arm in ("left", "right"):
            servo_path = os.path.join(
                get_package_share_directory("dual_crx_bringup"),
                "config",
                f"{arm}_servo.yaml",
            )
            with open(servo_path, "r", encoding="utf-8") as handle:
                servo_parameters = yaml.safe_load(handle)
            launch_items.append(
                Node(
                    package="moveit_servo",
                    executable="servo_node",
                    namespace=f"{arm}_servo",
                    name="servo_node",
                    output="log",
                    condition=IfCondition(launch_servo),
                    parameters=[{"moveit_servo": servo_parameters}, moveit_config.to_dict()],
                )
            )

    launch_items.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="dual_crx_robot_state_publisher",
            output="both",
            parameters=[moveit_config.robot_description],
            remappings=[("joint_states", "/joint_states")],
        )
    )

    launch_items.append(
        Node(
            package="dual_crx_control",
            executable="dual_crx_control_server",
            name="dual_crx_control_server",
            output="both",
            condition=IfCondition(launch_control_server),
            parameters=[{"operating_mode": "mock" if use_mock.perform(context) == "true" else "physical"}],
        )
    )

    launch_items.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="log",
            parameters=[moveit_config.to_dict()],
        )
    )

    launch_items.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="dual_crx_rviz",
            output="both",
            condition=IfCondition(launch_rviz),
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            arguments=[
                "--display-config",
                PathJoinSubstitution(
                    [FindPackageShare("dual_crx_moveit_config"), "rviz", "dual_crx.rviz"]
                ),
            ],
        )
    )

    return launch_items


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_control_server", default_value="true"),
            DeclareLaunchArgument("launch_servo", default_value="true"),
            DeclareLaunchArgument("left_robot_ip", default_value="192.168.2.100"),
            DeclareLaunchArgument("right_robot_ip", default_value="192.168.1.100"),
            DeclareLaunchArgument(
                "left_motion_control",
                default_value="0",
                description="Set to 1 only after left-arm connection-only checks pass.",
            ),
            DeclareLaunchArgument(
                "right_motion_control",
                default_value="0",
                description="Set to 1 only after right-arm connection-only checks pass.",
            ),
            DeclareLaunchArgument(
                "left_ros2_control_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("dual_crx_bringup"),
                        "config",
                        "left_mock_ros2_controllers.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "right_ros2_control_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("dual_crx_bringup"),
                        "config",
                        "right_mock_ros2_controllers.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "left_physical_ros2_control_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("fanuc_hardware_interface"),
                        "config",
                        "example_ros2_controllers.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "right_physical_ros2_control_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("fanuc_hardware_interface"),
                        "config",
                        "example_ros2_controllers.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "left_gpio_config_package",
                default_value="fanuc_hardware_interface",
            ),
            DeclareLaunchArgument(
                "right_gpio_config_package",
                default_value="fanuc_hardware_interface",
            ),
            DeclareLaunchArgument(
                "left_gpio_config_path",
                default_value="config/example_gpio_config_small.yaml",
            ),
            DeclareLaunchArgument(
                "right_gpio_config_path",
                default_value="config/example_gpio_config_small.yaml",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
