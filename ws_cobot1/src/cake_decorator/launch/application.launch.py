"""ROS 업무 노드 4개를 실행할 launch 파일의 자리. 현재는 실행할 수 없다."""

# 구현 시 LaunchDescription에 BR·TM·DP·RE 노드를 등록한다.
# setup.py에 각 main()을 console_scripts로 등록한 뒤 실행 파일명을 맞춘다.
# ROS_DOMAIN_ID와 로봇 네임스페이스는 기존 강의·작업대 설정과 일치시킨다.
# 로봇 드라이버는 기존 virtual/real 브링업에서 실행하며 이곳에서 중복 실행하지 않는다.
# 실제 로봇 대상 전환·작업 시작·그리퍼 동작을 launch 생성만으로 수행하지 않는다.
# 지금은 노드와 generate_launch_description()이 없으므로 ros2 launch를 실행하지 않는다.
