import { useEffect, useRef, useState } from "react";
import {
  parseWorkcellRecovery,
  readJson,
  sendWorkcellDecision,
} from "../api/hmi";
import type { WorkcellRecovery } from "../api/hmi";

/** 중단 단계를 표시하고 같은 주문 재개 또는 현장 초기화 확인 후 종료를 선택한다. */
export function WorkcellRecoveryPanel() {
  const [state, setState] = useState<WorkcellRecovery | null>(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const action = useRef<AbortController | null>(null);
  const mounted = useRef(false);
  useEffect(() => {
    mounted.current = true;
    let timer: ReturnType<typeof setTimeout>;
    let pending: AbortController | null = null;
    /** 주문 잠금을 조회한다. 정지 해제나 로봇 상태 변경은 요청하지 않는다. */
    async function poll() {
      const controller = new AbortController();
      pending = controller;
      const timeout = setTimeout(() => controller.abort(), 5000);
      try {
        const next = parseWorkcellRecovery(
          await readJson("/api/workcell/recovery", controller.signal),
        );
        if (mounted.current && !action.current) {
          setState(next);
          setError("");
        }
      } catch {
        if (mounted.current)
          setError("주문 잠금을 확인할 수 없습니다. 서버 연결을 확인하세요.");
      } finally {
        clearTimeout(timeout);
        if (mounted.current) timer = setTimeout(poll, 2000);
      }
    }
    void poll();
    return () => {
      mounted.current = false;
      pending?.abort();
      action.current?.abort();
      clearTimeout(timer);
    };
  }, []);

  /** 현장 확인 후 재개 예약 또는 최종 종료 중 하나만 요청한다. */
  async function decide(decision: "resume" | "initialize") {
    if (
      !state?.can_reset ||
      action.current ||
      (decision === "resume" && !state.can_resume)
    )
      return;
    const prompt =
      decision === "resume"
        ? `현장에서 빈 그리퍼·기본 TCP·안전 자세로 복귀시켰습니까? ${state.stopped_label}부터 같은 주문을 이어갑니다.`
        : "관리자가 초기 자세와 도구·케이크 정리를 마쳤습니까? 이 주문을 최종 실패로 종료하고 다음 주문을 받습니다. 로봇은 자동 이동하지 않습니다.";
    if (!window.confirm(prompt)) return;
    const controller = new AbortController();
    action.current = controller;
    setBusy(true);
    const timeout = setTimeout(() => controller.abort(), 5000);
    try {
      const next = await sendWorkcellDecision(
        decision,
        state.revision,
        controller.signal,
      );
      if (mounted.current) {
        setState(next);
        setError("");
      }
    } catch (reason) {
      if (mounted.current)
        setError(reason instanceof Error ? reason.message : "복구 요청 실패");
    } finally {
      clearTimeout(timeout);
      action.current = null;
      if (mounted.current) setBusy(false);
    }
  }

  return (
    <section className="hmi-card hmi-recovery" aria-live="polite">
      <div>
        <h2>
          {state?.requires_reset
            ? state.resume_requested
              ? "같은 주문 재개 중 · 새 주문 차단"
              : "공정 중단 · 관리자 복구 대기"
            : "주문 복구 상태"}
        </h2>
        {state?.requires_reset && <b>중단 위치: {state.stopped_label}</b>}
        <p>
          {state?.requires_reset
            ? `${state.order_id} · ${state.message}`
            : state
              ? "복구 대기 중인 주문이 없습니다."
              : "복구 상태 조회 중"}
        </p>
        <small>
          재개는 중단 단계부터 이어갑니다. 초기 자세 세팅은 관리자 정리 확인 후
          주문을 최종 실패로 종료합니다.
        </small>
        {state?.requires_reset &&
          !state.can_reset &&
          !state.resume_requested && (
            <p>진행·미확정 주문 또는 소프트웨어 정지를 먼저 확인하세요.</p>
          )}
        {state?.requires_reset && state.can_reset && !state.can_resume && (
          <p>검증된 재개 지점이 없어 초기 자세 세팅만 가능합니다.</p>
        )}
        {error && <p role="alert">{error}</p>}
      </div>
      <div className="hmi-recovery-actions">
        <button
          type="button"
          disabled={!state?.can_resume || busy || Boolean(error)}
          onClick={() => void decide("resume")}
        >
          재개하기
        </button>
        <button
          type="button"
          disabled={!state?.can_reset || busy || Boolean(error)}
          onClick={() => void decide("initialize")}
        >
          초기 자세 세팅
        </button>
      </div>
    </section>
  );
}
