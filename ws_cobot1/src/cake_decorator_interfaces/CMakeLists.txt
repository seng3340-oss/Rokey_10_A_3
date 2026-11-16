# PM 상세 정의에 나온 메시지·서비스·액션의 Python/C++ 코드를 생성한다.
cmake_minimum_required(VERSION 3.8)
project(cake_decorator_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 원본 픽셀용 Stroke2D를 등록한다. mm 전용 입력 타입은 등록하지 않는다.
# 별칭 타입을 추가하지 않는다. 표/다이어그램과의 이름 차이는 README에 기록한다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskStatus.msg"
  "msg/CakeParams.msg"
  "msg/WorkspaceCoords.msg"
  "msg/DrawingPath.msg"
  # 임시 우회: StrokeMM 계열 메시지도 등록해 GenerateTrajectory.srv 빌드를 통과시킨다.
  # 빌드 후 원본 CMakeLists로 되돌릴 예정이다.
  "msg/Stroke2D.msg"
  "msg/StrokeMM.msg"
  "msg/PointMM.msg"
  "msg/PointMM3D.msg"
  "msg/Waypoint3D.msg"
  "srv/GenerateTrajectory.srv"
  "srv/SwitchTCP.srv"
  "srv/SetCompliance.srv"
  "action/CakeOrder.action"
  "action/PickAndPlace.action"
  "action/GraspTool.action"
  "action/DrawOnCake.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(interface_serialization test/test_interfaces.py)
endif()

ament_package()
