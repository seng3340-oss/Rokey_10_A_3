"""플레이트·펜 안전예외 공통 정의.

예외 클래스와 메시지/코드 상수만 모은다. 실제 이동·복구 로직은 각 실행
모듈에 두어 순환 import를 피한다. 안전예외는 서비스/액션 결과를 실패로
반환하고, 로봇은 현재 또는 지정 Safe Position에서 대기한다.
"""


class SafetyError(RuntimeError):
    """안전예외 공통 베이스."""


class OpenerMissingError(SafetyError):
    """따개가 없을 때 그리퍼를 열고 Safe Position 대기로 전환한다."""


class PenGraspError(SafetyError):
    """펜 파지/보유 확인 실패 시 안전 정지로 전환한다."""


class PlateWeightError(SafetyError):
    """플레이트 하중 검사 실패."""


# 서비스/액션 응답과 로그에서 같은 문구를 쓰기 위한 상수.
OPENER_MISSING_MESSAGE = '따개 없음, 대기중'
PEN_GRASP_FAILED_CODE = 'PEN_GRASP_FAILED'
PEN_GRASP_FAILED_MESSAGE = '펜 파지 실패'
