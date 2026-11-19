"""모터 명령과 분리된 전체 공정 표시 계약. 단계는 실제 호출 전후에만 변경한다."""

STAGES = {
    "opener_pick": ("오프너 집기", 0, 8),
    "cake_to_station": ("케이크 틀 스테이션 배치", 8, 25),
    "opener_return_before": ("오프너 반납", 25, 30),
    "pen_pick": ("펜 집기", 30, 38),
    "drawing": ("윗면 드로잉", 38, 70),
    "pen_return": ("펜 반납", 70, 78),
    "opener_repick": ("오프너 다시 집기", 78, 83),
    "cake_to_outlet": ("케이크 틀 반출구 배치", 83, 93),
    "opener_return_after": ("오프너 최종 반납", 93, 97),
    "safe_position": ("마지막 안전 위치 이동", 97, 100),
}


def stage_payload(order_id, key, state, message=""):
    """현재 주문의 단계·결과만 표현한다. 진행률은 단계 배분이며 물리 측정값이 아니다."""
    label, start, end = STAGES[key]
    return dict(
        order_id=order_id,
        stage_key=key,
        stage_state=state,
        phase=key.upper(),
        progress_pct=float(end if state == "completed" else start),
        message=message
        or f"{label} "
        + {"running": "진행 중", "completed": "완료", "failed": "실패"}[state],
        error_code=(key.upper() + "_FAILED") if state == "failed" else "",
    )
