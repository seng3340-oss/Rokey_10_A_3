"""브릿지 실행과 업무 노드 개발 파일을 ROS Python 패키지로 등록한다."""

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "cake_decorator"

# 설명서와 주석으로 된 설정·launch 틀을 함께 설치한다.
# 구현된 브릿지만 등록한다. TM·DP·RE는 아직 실행할 수 없다.
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
    description="케이크 주문 브릿지와 업무 노드 개발 틀. 로봇 실행은 미구현.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ros_bridge_node = cake_decorator.ros_bridge_node:main",
        ],
    },
)
