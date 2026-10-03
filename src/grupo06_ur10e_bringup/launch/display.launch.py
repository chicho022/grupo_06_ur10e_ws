"""Visualización del UR10e (Grupo 6, IMT-342).

Uso:
  ros2 launch grupo06_ur10e_bringup display.launch.py            # RViz + GUI de sliders (pruebas de FK)
  ros2 launch grupo06_ur10e_bringup display.launch.py gui:=false # RViz sin GUI (pruebas de IK)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    ur_type = LaunchConfiguration("ur_type")
    rviz_config = LaunchConfiguration("rviz_config")

    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="true: abre joint_state_publisher_gui. "
                        "false: no lo abre (usar con ik_node).",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Modelo de robot UR.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("grupo06_ur10e_bringup"), "rviz", "ur10e.rviz"]
            ),
            description="Archivo de configuración de RViz2.",
        ),
    ]

    # URDF generado a partir del xacro oficial de Universal Robots
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
                ),
                " ur_type:=",
                ur_type,
                " name:=ur",
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="log",
    )

    return LaunchDescription(
        declared_arguments + [robot_state_publisher, joint_state_publisher_gui, rviz]
    )
