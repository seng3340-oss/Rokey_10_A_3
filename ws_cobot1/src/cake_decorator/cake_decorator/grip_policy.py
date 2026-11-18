"""RG2 닫힘 후 실측 턱 간격(mm)을 작업 진행 여부로 해석한다."""

import math


def grip_width_error(width_mm):
    """허용이면 빈 문자열, 그 외에는 액션과 로그에 쓸 오류 코드를 반환한다."""
    if width_mm is None or not math.isfinite(width_mm) or width_mm < 0:
        return 'GRIP_CHECK_UNAVAILABLE'
    if width_mm <= 5.0:
        return 'GRIP_ERROR'
    if width_mm < 10.0:
        return 'GRIP_INDETERMINATE'
    return ''
