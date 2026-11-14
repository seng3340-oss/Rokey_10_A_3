# 로봇 실행환경 소스 이관

`~/ws_cobot_pjt_jihwan/ws_dsr/src`의 소스를 이 프로젝트의 `ws_dsr/src`로 복사했습니다.
원본은 비교·복구용으로 유지합니다. 1,463개 소스·자산 파일의 SHA-256 일치를 확인했습니다.
중첩 `.git`과 Python 캐시는 제외했으며 기존 `build/install/log`는 가져오지 않았습니다.
외부 코드의 LICENSE는 그대로 보존합니다.
원본 src 저장소 기준 커밋: `4d5657f36a160eedb533ab1c975cd8a30c3e53b2`.

## 포함 내용

- `doosan-robot2/`: 두산 ROS 2 드라이버, 메시지, Python API, 모델, 에뮬레이터 도구
- `onrobot-ros2/`: OnRobot RG2 제어·메시지·모델
- `rg2/`: M0609 + RG2 통합 브링업과 가상 그리퍼
- `rokey1/`: 기존 수업 예제

테스트 프로젝트와 모델 m0609, 네임스페이스 dsr01, 실물 호스트 192.168.1.100이 일치합니다.
ROS_DOMAIN_ID는 셸에서 30을 설정합니다. 실행환경 파일 이관이 실물 재검증을 의미하지는 않습니다.
원본의 RG2 Modbus 주소는 192.168.1.1이며 로봇 주소와 별개입니다.

## 새 경로에서 준비

기존 경로의 install을 그대로 옮기지 않았으므로 새 경로에서 의존성 확인과 빌드가 필요합니다.
아래 명령은 준비 절차이며 이관 과정에서 실행하지 않았습니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ws_cobot_pjt/ws_dsr
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=30
```

과거 실물 로그에는 pymodbus 누락이 있었으므로 RG2 Python 의존성도 확인해야 합니다.
빌드가 끝나면 기존 sod 별칭이 가리키는 ~/ws_cobot_pjt/ws_dsr/install/setup.bash를 사용할 수 있습니다.
sodvir/sodreal, 네트워크 설정은 변경하지 않았습니다. 에뮬레이터나 실물 브링업은 실행하지 않았습니다.
