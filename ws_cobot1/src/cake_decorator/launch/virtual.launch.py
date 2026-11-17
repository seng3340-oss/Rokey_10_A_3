"""가상 모드 통합 테스트 Launch 파일.

프론트엔드(Vite) + 백엔드(FastAPI) + ROS2 노드 전체를 실행한다.
실제 로봇 없이 Doosan 에뮬레이터로 전체 파이프라인을 테스트.

실행:
  ros2 launch cake_decorator virtual.launch.py

또는 직접:
  python3 virtual.launch.py  (LaunchService 사용)
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """전체 가상 모드 통합 시스템 launch."""

    # ── 인자 ──
    args = [
        DeclareLaunchArgument(
            'config_file',
            # launch는 share/cake_decorator/launch에, 설정은 share/cake_decorator/config에
            # 설치되므로 상위 디렉터리의 config를 가리킨다. 소스 실행 때도 같은 경로가 된다.
            default_value=str(
                Path(__file__).resolve().parent.parent
                / 'config' / 'workspace_config.yaml'
            ),
            description='작업 공간 설정 파일 경로',
        ),
        DeclareLaunchArgument(
            'backend_port', default_value='5000',
            description='FastAPI 백엔드 포트',
        ),
        DeclareLaunchArgument(
            'frontend_port', default_value='8080',
            description='Vite 프론트엔드 포트',
        ),
        DeclareLaunchArgument(
            'show_rviz', default_value='false',
            description='RViz2 표시 여부',
        ),
    ]

    config_file = LaunchConfiguration('config_file')
    backend_port = LaunchConfiguration('backend_port')
    frontend_port = LaunchConfiguration('frontend_port')

    # ── 1. drawing_path_node (2D→3D 경로 변환) ──
    # 이 노드도 cake_decorator 패키지의 실행 파일이다(오타로 옛 패키지명을 쓰면
    # launch가 시작부터 패키지를 찾지 못해 노드가 하나도 뜨지 않는다).
    drawing_path = Node(
        package='cake_decorator',
        executable='drawing_path_node',
        name='drawing_path_node',
        parameters=[config_file],
        output='screen',
    )

    # ── 2. robot_executor_node (로봇 실행 — 가상 모드) ──
    robot_executor = Node(
        package='cake_decorator',
        executable='robot_executor_node',
        name='robot_executor_node',
        parameters=[config_file],
        output='screen',
    )

    # ── 3. task_manager_node (오케스트레이터) ──
    task_manager = Node(
        package='cake_decorator',
        executable='task_manager_node',
        name='task_manager_node',
        parameters=[config_file],
        output='screen',
    )

    # ── 4. ros_bridge_node (백엔드↔ROS2) ──
    ros_bridge = Node(
        package='cake_decorator',
        executable='ros_bridge_node',
        name='ros_bridge_node',
        parameters=[
            config_file,
            {'backend_url': ['http://127.0.0.1:', backend_port]},
            {'poll_interval': 3.0},
        ],
        output='screen',
    )

    # ── 안내 메시지 ──
    info = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════════════════╗\n',
        '║  🎂 케이크 레터링 자동화 — 가상 모드 통합 테스트    ║\n',
        '╠══════════════════════════════════════════════════════╣\n',
        '║  프론트엔드:  http://localhost:8080                 ║\n',
        '║  백엔드 API:  http://localhost:5000/docs            ║\n',
        '║  헬스체크:    http://localhost:5000/api/health       ║\n',
        '╠══════════════════════════════════════════════════════╣\n',
        '║  ROS2 노드:                                         ║\n',
        '║    - drawing_path_node (2D→3D 경로 변환)            ║\n',
        '║    - robot_executor_node (로봇 실행)                ║\n',
        '║    - task_manager_node (오케스트레이터)              ║\n',
        '║    - ros_bridge_node (백엔드↔ROS2)                  ║\n',
        '╠══════════════════════════════════════════════════════╣\n',
        '║  1. 프론트엔드에서 케이크 그림을 그린다             ║\n',
        '║  2. "주문 제출" 버튼을 누른다                       ║\n',
        '║  3. ros_bridge가 주문을 감지한다                    ║\n',
        '║  4. task_manager가 7-Phase 실행을 시작한다          ║\n',
        '║  5. 터미널에서 진행 상황 로그를 확인한다            ║\n',
        '╚══════════════════════════════════════════════════════╝\n',
    ])

    return LaunchDescription([
        *args,
        info,
        drawing_path,
        robot_executor,
        task_manager,
        ros_bridge,
    ])