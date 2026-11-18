# 모델과 설정을 ROS 패키지 share 경로에 설치한다. 외부 드라이버는 수정하지 않는다.
cmake_minimum_required(VERSION 3.8)
project(cake_workcell_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config rviz scripts DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/virtual_pen.py DESTINATION lib/${PROJECT_NAME})
ament_package()
