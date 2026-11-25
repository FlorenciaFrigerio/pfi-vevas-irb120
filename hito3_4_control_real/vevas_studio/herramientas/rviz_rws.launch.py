"""Muestra en RViz el modelo del repositorio con los /joint_states publicados por RWS."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    aqui = Path(__file__).resolve().parent
    urdf = aqui.parent / "modelo" / "irb120_rws.urdf"
    descripcion = ParameterValue(urdf.read_text(encoding="utf-8"), value_type=str)

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": descripcion}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(aqui / "rviz_rws.rviz")],
            output="screen",
        ),
    ])
