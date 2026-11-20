"""HMI 소프트웨어 정지 요청과 ROS 브릿지 전달 계약.

흐름
    HMI 정지 버튼 → POST /api/robot/stop (정지 상태 기록, 순번 증가)
    → 브릿지가 롱폴링(/internal/bridge/stop/wait)으로 즉시 수신
    → 브릿지가 드라이버 move_stop(Quick stop) 호출·실행 노드 래치·주문 취소
    → 브릿지가 적용 결과를 /internal/bridge/stop/applied로 보고 → HMI 표시

해제
    정지 해제 API는 두지 않는다. 정지 후에는 오프너·마커·케이크가 중간 위치에 남을 수
    있어, 현장을 정리한 뒤 실행기를 재시작해야만 다시 주문을 받는다.

주의
    이 기능은 소프트웨어 정지 요청이며 인증된 안전 기능(물리 E-stop)을 대체하지 않는다.
    상태는 백엔드 프로세스 메모리에만 있다. 백엔드 재시작은 전체 실행기 재시작과 같으므로
    재시작 후에는 정지되지 않은 상태에서 시작한다.
"""

import threading
import time
from typing import Annotated

from fastapi import APIRouter, Depends, Request
from pydantic import Field

from .bridge import authorize, now
from .schemas import ContractModel

router = APIRouter(tags=["HMI 소프트웨어 정지"])

# 브릿지 롱폴링 한 번의 최대 대기 시간[초]. 브릿지 HTTP 타임아웃보다 짧아야 한다.
MAX_WAIT_SECONDS = 20.0
# 마지막 롱폴링 이후 이 시간[초] 안이면 브릿지가 정지 요청을 듣고 있다고 본다.
# 브릿지 롱폴링 주기(기본 2초)에 재연결·지연 여유를 더한 값이다.
LISTENER_FRESH_SECONDS = 8.0


class StopWait(ContractModel):
    """브릿지 롱폴링 요청. after와 다른 순번이 생기면 즉시 응답한다.

    after가 -1이면 현재 상태를 바로 돌려준다(브릿지 시작 시 초기 동기화).
    """

    after: Annotated[int, Field(strict=True, ge=-1)]
    timeout_sec: Annotated[
        float, Field(gt=0, le=MAX_WAIT_SECONDS, allow_inf_nan=False)
    ] = MAX_WAIT_SECONDS


class StopApplied(ContractModel):
    """브릿지가 해당 순번의 정지(또는 시작 시 초기 동기화)를 로봇 쪽에 적용한 결과."""

    seq: Annotated[int, Field(strict=True, ge=0)]
    applied: bool
    message: Annotated[str, Field(max_length=2048)] = ""


class RobotStopState:
    """정지 요청 상태를 스레드 안전하게 보관하고 변경을 롱폴링 대기자에게 알린다.

    seq는 정지 요청마다 1씩 증가한다. 정지 중 버튼을 다시 눌러도 증가시켜
    브릿지가 move_stop을 한 번 더 보내게 한다. 한 번 정지하면 프로세스가 끝날 때까지
    stopped는 False로 돌아가지 않는다.
    """

    def __init__(self):
        self._condition = threading.Condition()
        self.seq = 0
        self.stopped = False
        self.requested_at = None
        # 브릿지가 마지막으로 결과를 보고한 순번과 결과
        self.applied_seq = None
        self.applied = False
        self.applied_at = None
        self.message = ""
        # 브릿지 마지막 롱폴링 시각(monotonic). None이면 한 번도 연결되지 않음.
        self.last_listen = None

    def snapshot(self):
        """HMI·브릿지에 공통으로 돌려주는 상태 사본. 호출자가 잠금을 잡지 않아도 된다."""
        with self._condition:
            listening = (
                self.last_listen is not None
                and time.monotonic() - self.last_listen <= LISTENER_FRESH_SECONDS
            )
            return {
                "seq": self.seq,
                "stopped": self.stopped,
                "requested_at": self.requested_at,
                # 현재 순번이 브릿지에 적용됐을 때만 applied=True다.
                "applied": self.applied_seq == self.seq and self.applied,
                "applied_seq": self.applied_seq,
                "applied_at": self.applied_at,
                "message": self.message,
                "bridge_listening": listening,
            }

    def request_stop(self):
        """정지 상태로 바꾸고 순번을 올려 대기 중인 브릿지를 즉시 깨운다."""
        with self._condition:
            self.requested_at = now()
            self.seq += 1
            self.stopped = True
            self.message = "정지 요청 전달 대기"
            self._condition.notify_all()
        return self.snapshot()

    def wait_change(self, after, timeout):
        """seq가 after와 달라지거나 timeout이 지나면 현재 상태를 반환한다."""
        with self._condition:
            self.last_listen = time.monotonic()
            self._condition.wait_for(lambda: self.seq != after, timeout)
            self.last_listen = time.monotonic()
        return self.snapshot()

    def record_applied(self, report):
        """현재 순번의 보고만 반영한다. 지난 순번 보고는 무시해 상태 역전을 막는다."""
        with self._condition:
            if report.seq != self.seq:
                return False
            self.applied_seq = report.seq
            self.applied = report.applied
            self.applied_at = now()
            self.message = report.message
            return True


def stop_state(request: Request) -> RobotStopState:
    """앱 단위 정지 상태. create_app()이 만들어 app.state에 둔다."""
    return request.app.state.robot_stop


def stop_requested(request: Request) -> bool:
    """정지 중이면 새 주문 인수를 막기 위해 다른 라우터에서 사용한다."""
    return stop_state(request).stopped


@router.get("/api/robot/stop")
def read_stop(request: Request):
    """HMI가 정지 상태와 브릿지 적용 여부를 읽는다."""
    return stop_state(request).snapshot()


@router.post("/api/robot/stop")
def request_stop(request: Request):
    """HMI 정지 버튼. 상태만 기록하고 즉시 반환한다. 실제 정지는 브릿지가 수행한다."""
    return stop_state(request).request_stop()


@router.post("/internal/bridge/stop/wait", dependencies=[Depends(authorize)])
def wait_stop(body: StopWait, request: Request):
    """브릿지 롱폴링. 동기 함수라 FastAPI 스레드 풀에서 대기하며 이벤트 루프를 막지 않는다."""
    return stop_state(request).wait_change(body.after, body.timeout_sec)


@router.post("/internal/bridge/stop/applied", dependencies=[Depends(authorize)])
def stop_applied(body: StopApplied, request: Request):
    """브릿지의 적용 결과 보고. 지난 순번이면 기록하지 않고 current=False를 반환한다."""
    return {"current": stop_state(request).record_applied(body)}
