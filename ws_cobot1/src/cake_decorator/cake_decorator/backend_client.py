"""브릿지의 내부 HTTP 연결. DB에 직접 접근하거나 주문을 임의로 재할당하지 않는다."""

import json
import os
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

    def __init__(self, base_url, token, timeout=3.0):
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

    def post(self, path, body=None):
        """204는 인수할 주문이 없다는 뜻이다. 실패한 claim은 자동 재시도하지 않는다."""
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
            with urlopen(request, timeout=self.timeout) as response:
                if response.status == 204:
                    return None
                return json.load(response)
        except HTTPError as error:
            raise BackendError(f"백엔드 HTTP {error.code}", error.code) from error
        except (URLError, TimeoutError, OSError, ValueError) as error:
            raise BackendError("백엔드 응답을 확인할 수 없습니다.") from error

    def heartbeat(self, ready, order_id, **errors):
        """서버 발견은 물리 로봇·안전 확인이 아님을 명확히 한 표시용 관측이다."""
        mode = os.environ.get('CAKE_SYSTEM_MODE', 'unknown')
        return self.post('/internal/bridge/heartbeat', {
            **errors,
            'task_server_ready': bool(ready), 'active_order_id': order_id,
            'execution_mode': {'real':'real', 'virtual':'simulation'}.get(mode, 'unknown'),
            'process_mode': 'full' if os.environ.get('CAKE_PLATE_SEQUENCE') == '1' else
                            'pen' if os.environ.get('CAKE_PEN_TOOL_CYCLE') == '1' else 'drawing',
        })

    def claim(self):
        """서버가 원자적으로 잠근 주문을 한 번 가져온다."""
        return self.post("/internal/bridge/claim")

    def event(self, claim, kind, **details):
        """상태 기록은 같은 이벤트 ID로만 재시도한다. Goal을 다시 보내지는 않는다."""
        body = {
            "event_id": str(uuid4()),
            "claim_token": claim["claim_token"],
            "kind": kind,
            **details,
            'process_mode': 'full' if os.environ.get('CAKE_PLATE_SEQUENCE') == '1' else
                            'pen' if os.environ.get('CAKE_PEN_TOOL_CYCLE') == '1' else 'drawing',
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
