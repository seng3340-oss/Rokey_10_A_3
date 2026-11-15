# 모델과 설정을 ROS 패키지 share 경로에 설치한다. 외부 드라이버는 수정하지 않는다.
cmake_minimum_required(VERSION 3.8)
project(cake_workcell_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})
install(FILES SOURCES.md mesh_checksums.json DESTINATION share/${PROJECT_NAME})
# 모델 검사는 ROS/컨트롤러를 실행하지 않고 xacro 결과만 확인한다.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(workcell_model tests/test_workcell.py)
endif()
ament_package()
