"""강의 가상 브링업을 재사용하여 로봇과 별도의 작업 환경을 함께 표시한다."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
import runpy
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _read_pen_length(context):
    """가상 실행 터미널에서 돌출 길이를 확인한다. 명시한 launch 인자가 있으면 재질문하지 않는다."""
    share = Path(get_package_share_directory("cake_workcell_bringup"))
    parse = runpy.run_path(str(share / "scripts/virtual_pen.py"))["parse_extension_cm"]
    value = LaunchConfiguration("pen_extension_cm").perform(context).strip()
    while True:
        try:
            if not value:
                value = input("가상 펜: 그리퍼 끝에서 펜촉까지 돌출 길이(cm) [기본 4]: ").strip() or "4"
            extension_m = parse(value)
            break
        except ValueError as exc:
            print(exc)
            value = ""
        except EOFError as exc:
            raise RuntimeError("터미널에서 실행하거나 pen_extension_cm:=4를 지정하세요.") from exc
    context.launch_configurations["virtual_pen_extension_m"] = str(extension_m)
    print(f"가상 펜 돌출 길이: {extension_m * 100:g}cm. 모델과 컨트롤러 TCP에 함께 적용합니다.")
    return []


def _workcell_publisher(context):
    """가상 작업대 문자열을 만들고 옆면 검증일 때만 케이크 표시 위치를 맞춘다."""
    models = Path(get_package_share_directory("cake_workcell_description"))
    share = Path(get_package_share_directory("cake_workcell_bringup"))
    mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in ("layout", "fixture_x", "fixture_y", "fixture_z", "fixture_yaw")
    }
    description = xacro.process_file(str(models / "urdf/workcell.urdf.xacro"), mappings=mappings).toxml()
    if LaunchConfiguration("side_pen_45").perform(context).lower() == "true":
        helpers = runpy.run_path(str(share / "scripts/virtual_pen.py"))
        center = [float(LaunchConfiguration(key).perform(context))
                  for key in ("side_center_x", "side_center_y", "side_top_z")]
        description = helpers["align_side_workcell"](description, center)
    return [Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="workcell_state_publisher",
        parameters=[{"robot_description": description}],
        remappings=[("robot_description", "/workcell_description")],
        output="screen",
    )]


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
        DeclareLaunchArgument("side_pen_45", default_value="false"),
        DeclareLaunchArgument("side_center_x", default_value="0.3992743225097656"),
        DeclareLaunchArgument("side_center_y", default_value="-0.054559329986572266"),
        DeclareLaunchArgument("side_top_z", default_value="0.220"),
        DeclareLaunchArgument("show_rviz", default_value="true"),
        DeclareLaunchArgument("pen_extension_cm", default_value=""),
    ]

    # 강의 launch는 어댑터에서 가상 모드로 고정하고 중복 RViz만 제외한다.
    return LaunchDescription([
        *arguments,
        OpaqueFunction(function=_read_pen_length),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/course_virtual.launch.py")),
            launch_arguments={"mode": "virtual", "host": "127.0.0.1", "port": "12345"}.items(),
        ),
        OpaqueFunction(function=_workcell_publisher),
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
