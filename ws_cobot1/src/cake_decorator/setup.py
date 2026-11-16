"""노드 구현 전에도 colcon이 인식할 수 있도록 Python 패키지 틀을 등록한다."""

from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "cake_decorator"

# 설명서와 주석으로 된 설정·launch 틀을 함께 설치한다.
# 노드 main()은 아직 없으므로 실행 가능한 console_scripts는 등록하지 않는다.
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
    description="케이크 ROS 업무 노드 개발 틀. 실제 실행 기능은 미구현.",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
