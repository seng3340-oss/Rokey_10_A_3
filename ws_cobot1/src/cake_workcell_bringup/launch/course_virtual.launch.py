"""설치된 강의 브링업을 수정하지 않고 가상 컨트롤러 부분만 재사용한다."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """실물 주소를 받지 않으며 RViz는 상위 통합 launch 한 곳에서만 띄운다."""
    course = Path(get_package_share_directory("m0609_rg2_bringup"))
    spec = importlib.util.spec_from_file_location(
        "course_virtual_only", course / "launch/bringup.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # 원본 launch에 RViz 끄기 인자가 없어 RViz Node만 걸러낸다.
    # 제조사·교육장 드라이버 소스에는 변경을 가하지 않는다.
    actions = [
        action for action in module.generate_launch_description().entities
        if not (isinstance(action, Node) and action.node_executable == "rviz2")
    ]
    return LaunchDescription([
        SetLaunchConfiguration("mode", "virtual"),
        SetLaunchConfiguration("host", "127.0.0.1"),
        SetLaunchConfiguration("port", "12345"),
        *actions,
    ])
