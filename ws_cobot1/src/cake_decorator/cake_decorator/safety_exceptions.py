"""장비 예외의 오류 코드와 HMI에 전달할 설명을 정의한다."""

import time


class SafetyException(RuntimeError):
    """복구 결과를 포함해 작업 관리 노드까지 전달하는 예외다."""

    code = "SAFETY_CHECK_FAILED"


class OpenerMissingError(SafetyException):
    """안전 자세에서 따개 보유를 확인하지 못한 경우다."""

    code = "OPENER_MISSING"


class PenGraspError(SafetyException):
    """펜 파지 전 열림 또는 인출 후 보유 검사가 실패한 경우다."""

    code = "PEN_GRASP_FAILED"


# ── 도구 없음(재파지 대상) ──
# 들어올린 뒤 측정한 그리퍼 폭이 기준보다 좁아 "도구를 쥐지 못했다"고 판정한 경우만
# 거치대로 돌아가 한 번 더 집는다. 오류 코드는 부모와 같아 HMI·TM 계약은 바뀌지 않는다.
# 폭 측정 실패나 파지 전 열림 부족(그리퍼 이상)은 재시도하지 않고 기존처럼 안전 자세로 간다.
class OpenerNotHeldError(OpenerMissingError):
    """안전 자세에서 측정한 폭이 기준보다 좁아 따개가 없다고 판정했다(그리퍼는 열어 둔 상태)."""


class PenNotHeldError(PenGraspError):
    """펜을 들어올린 뒤 측정한 폭이 기준보다 좁아 펜이 없다고 판정했다."""


class PlateWeightError(SafetyException):
    """하중 측정·케이크 보유 판정·그 후 복귀 결과를 전달한다."""

    code = "CAKE_WEIGHT_FAILED"

    def __init__(self, message, *, resume_allowed=False):
        """실제 재집기 소진인지 구분한다. 안착 이후 측정 장애를 입고부터 반복하지 않는다."""
        super().__init__(message)
        self.resume_allowed = resume_allowed


def require_pen_width(robot, parameter, stage):
    """파지 전 열림·인출 후 보유 폭을 검사한다. 실패 후 복구는 실행 노드가 맡는다."""
    try:
        enabled = parameter("gripper_safety_check_enabled")
    except KeyError:
        enabled = False
    if not enabled:
        return
    minimum = parameter("pen_open_min_width_m" if stage == "open" else "pen_hold_min_width_m")
    width = robot.get_gripper_width()
    if width is None:
        raise PenGraspError("펜 파지 실패: 최신 그리퍼 폭을 확인할 수 없습니다")
    if width < minimum:
        detail = f"(측정 {width * 1000:.1f}mm, 기준 {minimum * 1000:.1f}mm)"
        if stage == "open":
            raise PenGraspError(f"펜 파지 전 열림 폭 부족 {detail}")
        # 실제로 측정된 폭이 좁다 = 펜을 쥐지 못했다. 실행 노드가 한 번 더 집을 수 있다.
        raise PenNotHeldError(f"펜을 들어올린 뒤 파지 폭 부족 {detail}")


# 재파지 전 기본 대기 시간[초]. 작업자가 거치대에 도구를 다시 놓을 시간이다.
REGRASP_WAIT_DEFAULT_SEC = 5.0


def wait_before_regrasp(robot, parameter, tool, cancelled=lambda: False):
    """도구 없음 판정 뒤 재파지를 해도 되는지 확인하고 정해진 시간만큼 기다린다.

    호출 시점: 그리퍼를 연 뒤, 거치대로 다시 출발하기 직전.
    입력: tool은 로그용 도구 이름("따개"/"펜"), cancelled는 취소 여부를 돌려주는 함수.
    반환: 재파지를 진행하면 True. HMI 정지 래치·그리퍼 출력 이상·취소 중이면 False이며,
          이때 호출자는 재시도 없이 기존 안전 자세 처리로 넘어간다.
    대기 시간은 tool_regrasp_wait_sec 파라미터(없으면 5초)이고, 0.1초마다 정지·취소를 확인한다.
    """
    def blocked():
        return (
            getattr(robot, "stop_latched", False)
            or getattr(robot, "gripper_output_fault", False)
            or cancelled()
        )

    if blocked():
        return False
    try:
        wait = float(parameter("tool_regrasp_wait_sec"))
    except KeyError:
        wait = REGRASP_WAIT_DEFAULT_SEC
    robot._logger.warning(f"{tool} 없음: {wait:g}초 뒤 거치대에서 한 번 더 집습니다")
    deadline = time.monotonic() + max(0.0, wait)
    while time.monotonic() < deadline:
        if blocked():
            return False
        time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
    return not blocked()


# Action의 오류 코드만 받은 경우에도 HMI에는 같은 한국어 이유를 제공한다.
ERROR_MESSAGES = {
    "OPENER_MISSING": "따개 없음, 안전 자세에서 대기중",
    "PEN_GRASP_FAILED": "펜 파지 실패, 안전 자세에서 대기중",
    "PEN_GRASP_RECOVERY_FAILED": "펜 파지 실패, 안전 자세 복귀 실패. 현재 자세를 확인하세요",
    "CAKE_WEIGHT_FAILED": "케이크 무게 확인 실패",
}
