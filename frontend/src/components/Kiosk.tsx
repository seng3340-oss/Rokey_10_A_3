import { useCallback, useEffect, useRef, useState } from "react";
import { parseWorkcellRecovery, readJson } from "../api/hmi";
import type { WorkcellRecovery } from "../api/hmi";
import { KioskHeader } from "./KioskHeader";
import { DrawingWorkspace } from "./DrawingWorkspace";
import { ReviewDialog } from "./ReviewDialog";
import { mountDrawing } from "../drawing/controller.js";
/** 고객 화면의 세 영역을 조립하고 DOM 준비 후 필기 컨트롤러를 연결한다. */
function KioskWorkspace({
  existingOrderId,
  onOrderReset,
}: {
  existingOrderId?: string;
  onOrderReset?: () => void;
}) {
  // 컨트롤러가 반환한 정리 함수를 React에 전달해 화면 해제 시 타이머와 3D를 해제한다.
  useEffect(
    () => mountDrawing({ existingOrderId, onOrderReset }),
    [existingOrderId, onOrderReset],
  );
  return (
    <>
      <KioskHeader />
      <DrawingWorkspace />
      <ReviewDialog />
    </>
  );
}

/** 제작 실패 잠금은 입력 화면 밖에서 관리해 새로고침·다른 태블릿에서도 유지한다. */
export function Kiosk() {
  const [recovery, setRecovery] = useState<WorkcellRecovery | null>(null);
  const [error, setError] = useState("");
  const [continuing, setContinuing] = useState<{
    orderId: string;
    revision: number;
  } | null>(null);
  const acknowledged = useRef(0);
  const finishOrder = useCallback(() => {
    if (continuing) acknowledged.current = continuing.revision;
    setContinuing(null);
  }, [continuing]);
  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    let pending: AbortController | null = null;
    /** 잠금 조회만 수행한다. 태블릿에서는 해제 요청이나 실패 주문 재전송을 하지 않는다. */
    async function poll() {
      const controller = new AbortController();
      pending = controller;
      const timeout = setTimeout(() => controller.abort(), 5000);
      try {
        const next = parseWorkcellRecovery(
          await readJson("/api/workcell/recovery", controller.signal),
        );
        if (active) {
          setRecovery(next);
          if (next.resume_requested && next.revision !== acknowledged.current) {
            setContinuing((previous) =>
              previous?.revision === next.revision
                ? previous
                : {
                    orderId: next.order_id,
                    revision: next.revision,
                  },
            );
          } else if (!next.requires_reset && next.reset_at) {
            setContinuing(null);
          }
          setError("");
        }
      } catch {
        // 마지막으로 확인한 잠금을 통신 실패로 풀지 않는다. 최초 조회 전도 입력을 숨긴다.
        if (active)
          setError("서버 연결을 확인하고 있습니다. 잠시만 기다려주세요.");
      } finally {
        clearTimeout(timeout);
        if (active) timer = setTimeout(poll, 2000);
      }
    }
    void poll();
    return () => {
      active = false;
      pending?.abort();
      clearTimeout(timer);
    };
  }, []);

  if (!recovery || (recovery.requires_reset && !recovery.resume_requested)) {
    // 잠금 중에는 입력 UI 자체를 해제한다. dialog 취소·뒤로 가기·무조작 타이머로
    // 주문 화면에 진입할 수 없고, HMI 해제 응답을 받은 뒤에만 새 입력기를 만든다.
    return (
      <main className="kiosk-recovery" role="alert" aria-live="assertive">
        <span className="overline">CAKEBOT</span>
        <h1>
          {recovery?.requires_reset
            ? "제작 과정에서 문제가 발생했습니다"
            : "운영 상태를 확인하고 있습니다"}
        </h1>
        <p>
          {recovery?.requires_reset
            ? "관리자에게 문의해주세요."
            : "잠시만 기다려주세요."}
        </p>
        {recovery?.requires_reset && (
          <>
            <p>
              관리자가 HMI에서 재설정을 완료할 때까지 새 주문을 받을 수
              없습니다.
            </p>
            <small>주문번호 {recovery.order_id}</small>
          </>
        )}
        {error && <p role="status">{error}</p>}
      </main>
    );
  }
  // 재개 중에는 기존 주문 조회만 연결하며 새 주문 POST를 보내지 않는다.
  if (continuing)
    return (
      <KioskWorkspace
        key={`resume-${continuing.revision}`}
        existingOrderId={continuing.orderId}
        onOrderReset={finishOrder}
      />
    );
  return <KioskWorkspace key="new" />;
}
