"""설치된 강의 브링업을 수정하지 않고 가상 컨트롤러 부분만 재사용한다."""

import importlib.util
import runpy

import xacro
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    SetLaunchConfiguration,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def _course_actions(context):
    """실물 주소를 받지 않으며 RViz는 상위 통합 launch 한 곳에서만 띄운다."""
    course = Path(get_package_share_directory("m0609_rg2_bringup"))
    spec = importlib.util.spec_from_file_location(
        "course_virtual_only", course / "launch/bringup.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # 원본 RViz와 RSP를 제외하고, 펜 모델을 포함한 RSP 한 개를 실행한다.
    # 제조사·교육장 드라이버 소스에는 변경을 가하지 않는다.
    actions = [
        action for action in module.generate_launch_description().entities
        if not (isinstance(action, Node) and action.node_executable in ("rviz2", "robot_state_publisher"))
    ]
    # 강의 원본을 렌더링한 문자열에만 가상 펜을 붙여 모델과 제어 기준을 맞춘다.
    share = Path(get_package_share_directory("cake_workcell_bringup"))
    helpers = runpy.run_path(str(share / "scripts/virtual_pen.py"))
    extension = float(LaunchConfiguration("virtual_pen_extension_m").perform(context))
    description = xacro.process_file(
        str(course / "xacro/my_m0609_rg2.urdf.xacro"),
        mappings={
            "name": "dsr01", "host": "127.0.0.1", "port": "12345",
            "mode": "virtual", "model": "m0609", "update_rate": "100",
        },
    ).toxml()
    side_pen = LaunchConfiguration("side_pen_45", default="false").perform(context).lower() == "true"
    description = helpers["add_pen"](description, extension, side_pen_45=side_pen)
    pen_setup = Node(package="cake_workcell_bringup", executable="virtual_pen.py", output="screen")
    # TCP 준비 실패를 놓친 채 가상 로봇을 계속 사용하지 않도록 전체 launch를 종료한다.
    def check_setup(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="가상 펜 TCP 초기화 실패"))]
        return []

    return [
        RegisterEventHandler(OnProcessExit(target_action=pen_setup, on_exit=check_setup)),
        SetLaunchConfiguration("mode", "virtual"),
        SetLaunchConfiguration("host", "127.0.0.1"),
        SetLaunchConfiguration("port", "12345"),
        *actions,
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", namespace="dsr01",
            parameters=[{"robot_description": description}], output="screen",
        ),
        # 초기화 서비스가 켜질 때까지 기다린다. TCP와 가상 그리퍼만 설정한다.
        pen_setup,
    ]


def generate_launch_description():
    """상위 virtual launch에서 확인한 펜 길이로 가상 로봇만 실행한다."""
    return LaunchDescription([OpaqueFunction(function=_course_actions)])
