"""장비 예외의 오류 코드와 HMI에 전달할 설명을 정의한다."""


class SafetyException(RuntimeError):
    """복구 결과를 포함해 작업 관리 노드까지 전달하는 예외다."""

    code = "SAFETY_CHECK_FAILED"


class OpenerMissingError(SafetyException):
    """안전 자세에서 따개 보유를 확인하지 못한 경우다."""

    code = "OPENER_MISSING"


class PenGraspError(SafetyException):
    """펜 파지 전 열림 또는 인출 후 보유 검사가 실패한 경우다."""

    code = "PEN_GRASP_FAILED"


class PlateWeightError(SafetyException):
    """하중 측정·케이크 보유 판정·그 후 복귀 결과를 전달한다."""

    code = "CAKE_WEIGHT_FAILED"


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
        description = "펜 파지 전 열림 폭 부족" if stage == "open" else "펜을 들어올린 뒤 파지 폭 부족"
        raise PenGraspError(f"{description} (측정 {width * 1000:.1f}mm, 기준 {minimum * 1000:.1f}mm)")


# Action의 오류 코드만 받은 경우에도 HMI에는 같은 한국어 이유를 제공한다.
ERROR_MESSAGES = {
    "OPENER_MISSING": "따개 없음, 안전 자세에서 대기중",
    "PEN_GRASP_FAILED": "펜 파지 실패, 안전 자세에서 대기중",
    "PEN_GRASP_RECOVERY_FAILED": "펜 파지 실패, 안전 자세 복귀 실패. 현재 자세를 확인하세요",
    "CAKE_WEIGHT_FAILED": "케이크 무게 확인 실패",
}
