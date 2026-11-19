"""브릿지 실행과 업무 노드 개발 파일을 ROS Python 패키지로 등록한다."""

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "cake_decorator"

# 설명서와 주석으로 된 설정·launch 틀을 함께 설치한다.
# 브릿지에 더해 좌표 변환 노드(DP)와 로봇 실행 노드(RE)도 ros2 run으로 실행할 수
# 있도록 등록한다. 가상 로봇 환경에서 2D→3D 변환과 드로잉 실행을 검증하려면
# 이 두 엔트리포인트가 필요하다. TM(task_manager)도 같은 방식으로 등록한다.
# main()이 없는 모듈은 실행스크립트로 등록하지 않는다.
setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (
            f"share/{PACKAGE_NAME}",
            ["package.xml", "README.md"],
        ),
        (
            f"share/{PACKAGE_NAME}/config",
            glob("config/*.yaml"),
        ),
        (
            f"share/{PACKAGE_NAME}/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cake project team",
    maintainer_email="applewlghks321@gmail.com",
    description="케이크 주문 브릿지·작업 실행과 읽기 전용 HMI 상태 전달.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ros_bridge_node = cake_decorator.ros_bridge_node:main",
            # 2D→3D 경로 변환 서비스 서버. TM이 호출하는 generate_trajectory를 제공한다.
            "drawing_path_node = cake_decorator.drawing_path_node:main",
            # DrawOnCake·GraspTool·PickAndPlace 액션 서버. 가상 로봇에 MoveLine을 보낸다.
            "robot_executor_node = cake_decorator.robot_executor_node:main",
            # CakeOrder 액션 서버. DP·RE를 순서대로 호출하는 오케스트레이터.
            "task_manager_node = cake_decorator.task_manager_node:main",
            "test_gripper = cake_decorator.test_gripper:main",
            "test = cake_decorator.test:main",
            "test_interfaces = cake_decorator.test_interfaces:main",
        ],
    },
)
