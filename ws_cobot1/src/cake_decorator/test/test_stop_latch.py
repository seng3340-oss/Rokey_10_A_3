"""HMI 소프트웨어 정지 래치가 이동·그리퍼 명령만 막고 정지·조회는 허용하는지 검증한다."""

from types import SimpleNamespace

from cake_decorator.doosan_robot import DoosanRobotClient


class _Logger:
    """오류 로그만 모아 거부 사유를 확인한다."""

    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


class _Future:
    """즉시 완료된 서비스 응답."""

    def __init__(self, result):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result


class _Client:
    """서비스 호출 횟수만 기록하는 가짜 클라이언트."""

    def __init__(self):
        self.calls = 0

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.calls += 1
        return _Future(SimpleNamespace(success=True))


def _robot(keys):
    """ROS 노드 없이 _call만 시험할 수 있게 필요한 속성만 채운다."""
    robot = object.__new__(DoosanRobotClient)
    robot._logger = _Logger()
    robot._timeout = 1.0
    robot._clients = {key: _Client() for key in keys}
    robot.stop_latched = False
    return robot


def test_latch_blocks_motion_and_gripper_commands():
    keys = ["move_line", "move_joint", "move_jointx", "set_digital_output", "gripper"]
    robot = _robot(keys)
    robot.stop_latched = True
    for key in keys:
        assert robot._call(key, object()) is None
        assert robot._clients[key].calls == 0
    assert len(robot._logger.errors) == len(keys)


def test_latch_allows_stop_and_queries():
    keys = ["move_stop", "current_pose", "get_tcp", "set_tcp", "set_mode", "check_motion"]
    robot = _robot(keys)
    robot.stop_latched = True
    for key in keys:
        assert robot._call(key, object()).success is True
        assert robot._clients[key].calls == 1


def test_release_allows_motion_again():
    robot = _robot(["move_line"])
    robot.stop_latched = True
    assert robot._call("move_line", object()) is None
    robot.stop_latched = False
    assert robot._call("move_line", object()).success is True
