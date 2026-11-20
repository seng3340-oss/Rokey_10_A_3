"""브릿지의 내부 HTTP 연결. DB에 직접 접근하거나 주문을 임의로 재할당하지 않는다."""

import json
import time
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlparse
from urllib.request import Request, urlopen
from uuid import uuid4


class BackendError(RuntimeError):
    """접수 여부를 알 수 없는 전송 오류도 호출자가 구분할 수 있게 HTTP 상태를 보존한다."""

    def __init__(self, message, status=None):
        super().__init__(message)
        self.status = status


class BackendClient:
    """백엔드와 같은 호스트에서 인증된 내부 API를 호출한다."""

    def __init__(self, base_url, token, timeout=3.0, execution_mode="unknown"):
        parsed = urlparse(base_url)
        if (
            parsed.scheme not in ("http", "https")
            or not parsed.hostname
            or not token
            or not token.isascii()
        ):
            raise ValueError(
                "백엔드 주소와 ASCII 문자로 된 CAKE_BRIDGE_TOKEN이 필요합니다."
            )
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout = timeout
        if execution_mode not in ("unknown", "real", "simulation"):
            raise ValueError("운전 모드 설정을 확인하세요.")
        self.execution_mode = execution_mode

    def post(self, path, body=None, timeout=None):
        """204는 인수할 주문이 없다는 뜻이다. 실패한 claim은 자동 재시도하지 않는다.

        timeout을 주면 이번 요청에만 HTTP 대기 시간을 바꾼다(정지 롱폴링용).
        """
        request = Request(
            self.base_url + path,
            data=json.dumps(body or {}).encode(),
            method="POST",
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.token}",
            },
        )
        try:
            with urlopen(
                request, timeout=self.timeout if timeout is None else timeout
            ) as response:
                if response.status == 204:
                    return None
                return json.load(response)
        except HTTPError as error:
            raise BackendError(f"백엔드 HTTP {error.code}", error.code) from error
        except (URLError, TimeoutError, OSError, ValueError) as error:
            raise BackendError("백엔드 응답을 확인할 수 없습니다.") from error

    def wait_stop(self, after, wait_sec=2.0):
        """HMI 정지 상태가 순번 after에서 바뀔 때까지 서버에서 대기한다(롱폴링).

        정지 요청은 대기 중에도 즉시 응답되므로 wait_sec는 반응 속도와 무관하다.
        짧게(2초) 두는 이유는 백엔드 종료 시 진행 중 요청을 기다리는 시간을 줄이기 위해서다.
        서버는 최대 wait_sec 동안 응답을 붙잡으므로 HTTP 타임아웃은 그보다 길게 둔다.
        반환값은 서버 정지 상태 사본(seq, stopped 등)이다.
        """
        return self.post(
            "/internal/bridge/stop/wait",
            {"after": int(after), "timeout_sec": float(wait_sec)},
            timeout=wait_sec + 5.0,
        )

    def stop_applied(self, seq, applied, message=""):
        """정지·해제 순번 seq를 로봇 쪽에 적용한 결과를 HMI에 보고한다."""
        return self.post(
            "/internal/bridge/stop/applied",
            {"seq": int(seq), "applied": bool(applied), "message": message[:2048]},
        )

    def claim(self):
        """서버가 원자적으로 잠근 주문을 한 번 가져온다."""
        return self.post("/internal/bridge/claim")

    def heartbeat(self, task_server_ready, active_order_id, task_feedback_age_sec):
        """TM 관측 정보만 전송하며 주문 인수나 로봇 명령을 실행하지 않는다."""
        return self.post(
            "/internal/bridge/heartbeat",
            {
                "execution_mode": self.execution_mode,
                "task_server_ready": bool(task_server_ready),
                "active_order_id": active_order_id,
                "task_feedback_age_sec": task_feedback_age_sec,
            },
        )

    def plan(self, claim, plan):
        """TM이 만든 계획을 인수한 주문에만 연결한다. 경로를 재계산하지 않는다."""
        return self.post(
            f"/internal/bridge/orders/{quote(claim['order_id'], safe='')}/plan",
            {
                "claim_token": claim["claim_token"],
                "execution_mode": self.execution_mode,
                "plan": plan,
            },
        )

    def event(self, claim, kind, **details):
        """상태 기록은 같은 이벤트 ID로만 재시도한다. Goal을 다시 보내지는 않는다."""
        body = {
            "event_id": str(uuid4()),
            "claim_token": claim["claim_token"],
            "kind": kind,
            "execution_mode": self.execution_mode,
            **details,
        }
        path = f"/internal/bridge/orders/{quote(claim['order_id'], safe='')}/events"
        for attempt in range(3):
            try:
                return self.post(path, body)
            except BackendError as error:
                # 인증·순서 오류는 재시도로 해결하지 않는다.
                if error.status is not None and error.status < 500:
                    raise
                if attempt == 2:
                    raise
                time.sleep(0.2)
