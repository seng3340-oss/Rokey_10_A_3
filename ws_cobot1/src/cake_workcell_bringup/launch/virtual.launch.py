"""강의 가상 브링업을 재사용하여 로봇과 별도의 작업 환경을 함께 표시한다."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    """real 모드 인자를 제공하지 않으며 로봇 이동 명령은 발행하지 않는다."""
    share = Path(get_package_share_directory("cake_workcell_bringup"))
    models = Path(get_package_share_directory("cake_workcell_description"))
    config = yaml.safe_load((share / "config/workcell.yaml").read_text())["fixture"]

    # 최신 스테이션을 기본으로 표시한다. 기존 고정구 위치 인자는 legacy 배치에서 쓴다.
    arguments = [
        DeclareLaunchArgument(
            "layout",
            default_value="station",
            choices=["station", "legacy"],
            description="실측 스테이션 또는 기존 드로잉 데모 배치",
        ),
        DeclareLaunchArgument("fixture_x", default_value=str(config["x_mm"] / 1000)),
        DeclareLaunchArgument("fixture_y", default_value=str(config["y_mm"] / 1000)),
        DeclareLaunchArgument("fixture_z", default_value=str(config["z_mm"] / 1000)),
        DeclareLaunchArgument("fixture_yaw", default_value=str(math.radians(config["yaw_deg"]))),
        DeclareLaunchArgument("show_rviz", default_value="true"),
    ]

    # 시각화 중 로봇과의 TF 충돌을 피하기 위해 작업대는 독립 description Topic을 사용한다.
    description = Command([
        FindExecutable(name="xacro"), " ", str(models / "urdf/workcell.urdf.xacro"),
        " layout:=", LaunchConfiguration("layout"),
        " fixture_x:=", LaunchConfiguration("fixture_x"),
        " fixture_y:=", LaunchConfiguration("fixture_y"),
        " fixture_z:=", LaunchConfiguration("fixture_z"),
        " fixture_yaw:=", LaunchConfiguration("fixture_yaw"),
    ])

    # 강의 launch는 어댑터에서 가상 모드로 고정하고 중복 RViz만 제외한다.
    return LaunchDescription([
        *arguments,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/course_virtual.launch.py")),
            launch_arguments={"mode": "virtual", "host": "127.0.0.1", "port": "12345"}.items(),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="workcell_state_publisher",
            parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
            remappings=[("robot_description", "/workcell_description")],
            output="screen",
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="robot_to_workcell",
            arguments=["--x", "0", "--y", "0", "--z", "0", "--frame-id", "base_link", "--child-frame-id", "workcell_root"],
        ),
        Node(
            package="rviz2", executable="rviz2", name="workcell_rviz",
            arguments=["-d", str(share / "rviz/workcell.rviz")],
            condition=IfCondition(LaunchConfiguration("show_rviz")),
            output="screen",
        ),
    ])
