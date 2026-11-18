#!/usr/bin/env python3
"""가상 환경에서 OnRobot RG2 그리퍼 동작 테스트.

사용법:
  1) 가상 환경 런치 (별도 터미널):
     ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual

  2) 이 스크립트 실행 (별도 터미널):
     cd ws_cobot1
     source install/setup.bash
     ros2 run cake_decorator test_gripper

  또는 패키지 빌드 없이 직접 실행:
     cd ws_cobot1
     source install/setup.bash
     python3 src/cake_decorator/cake_decorator/test_gripper.py
"""

import time
import threading

import rclpy
from rclpy.node import Node
from onrobot_rg_msgs.srv import SetCommand


class GripperTestNode(Node):
    def __init__(self):
        super().__init__('gripper_test_node')

        self._gripper_client = self.create_client(
            SetCommand, '/onrobot/sendCommand'
        )

        self.get_logger().info('그리퍼 테스트 노드 시작')
        self.get_logger().info('서비스 대기 중: /onrobot/sendCommand ...')

        if not self._gripper_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '❌ /onrobot/sendCommand 서비스를 찾을 수 없습니다.\n'
                '   가상 환경이 실행 중인지 확인하세요.'
            )
            return

        self.get_logger().info('✅ 그리퍼 서비스 연결 완료!')
        self._run_test()

    def _call_gripper(self, command: str) -> bool:
        """그리퍼 명령을 동기로 호출하고 결과를 반환한다."""
        req = SetCommand.Request()
        req.command = command

        self.get_logger().info(f'  명령 전송: {command}')

        future = self._gripper_client.call_async(req)

        # 응답 대기
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(timeout=10.0):
            self.get_logger().error('  ❌ 응답 타임아웃')
            return False

        result = future.result()
        if result is None:
            self.get_logger().error('  ❌ 응답 없음')
            return False

        self.get_logger().info(
            f'  결과: success={result.success}, message="{result.message}"'
        )
        return result.success

    def _run_test(self):
        """그리퍼 열기/닫기 테스트 시퀀스."""
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  그리퍼 테스트 시작')
        self.get_logger().info('=' * 50)

        # 테스트 1: 그리퍼 열기 (100mm)
        self.get_logger().info('')
        self.get_logger().info('[테스트 1] 그리퍼 열기 (100mm)')
        ok = self._call_gripper('open(100)')
        self.get_logger().info(f'  → {"성공 ✅" if ok else "실패 ❌"}')

        time.sleep(2.0)

        # 테스트 2: 그리퍼 닫기 (10mm, 20N) — 펜 잡기 시뮬레이션
        self.get_logger().info('')
        self.get_logger().info('[테스트 2] 그리퍼 닫기 (10mm, 20N) — 펜 잡기')
        ok = self._call_gripper('close(10,20)')
        self.get_logger().info(f'  → {"성공 ✅" if ok else "실패 ❌"}')

        time.sleep(2.0)

        # 테스트 3: 다시 열기 — 펜 놓기 시뮬레이션
        self.get_logger().info('')
        self.get_logger().info('[테스트 3] 그리퍼 열기 (100mm) — 펜 놓기')
        ok = self._call_gripper('open(100)')
        self.get_logger().info(f'  → {"성공 ✅" if ok else "실패 ❌"}')

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  테스트 완료!')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = GripperTestNode()

    # 테스트 완료 후 잠시 대기했다가 종료
    time.sleep(1.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()