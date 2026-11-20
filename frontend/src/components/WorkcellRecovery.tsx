import { useEffect, useRef, useState } from "react";
import {
  parseWorkcellRecovery,
  readJson,
  sendWorkcellDecision,
} from "../api/hmi";
import type { WorkcellRecovery } from "../api/hmi";

/** 확인창을 연 시점의 실패 세대다. 갱신된 다른 주문에 오래된 확인을 적용하지 않는다. */
type Confirmation = {
  decision: "resume" | "initialize";
  revision: number;
  orderId: string;
  label: string;
};

/** 예외 대기 때만 복구 카드를 활성화한다. 소프트웨어 정지는 부모의 독립 버튼이 담당한다. */
export function WorkcellRecoveryPanel({
  stopped = false,
  onError,
}: {
  stopped?: boolean;
  onError?: (message: string) => void;
}) {
  const [state, setState] = useState<WorkcellRecovery | null>(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const [confirmation, setConfirmation] = useState<Confirmation | null>(null);
  const action = useRef<AbortController | null>(null);
  const mounted = useRef(false);
  const confirmButton = useRef<HTMLButtonElement | null>(null);

  useEffect(() => {
    onError?.(error);
  }, [error, onError]);
  useEffect(() => {
    if (stopped) setConfirmation(null);
  }, [stopped]);
  useEffect(() => {
    if (confirmation) confirmButton.current?.focus();
  }, [confirmation]);

  useEffect(() => {
    mounted.current = true;
    let timer: ReturnType<typeof setTimeout>;
    let pending: AbortController | null = null;
    /** 복구 상태만 매초 조회한다. 조회 응답은 실행 중인 버튼 요청의 결과를 덮지 않는다. */
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
          setError(
            "주문 복구 상태를 확인할 수 없습니다. 서버 연결을 확인하세요.",
          );
      } finally {
        clearTimeout(timeout);
        if (mounted.current) timer = setTimeout(poll, 1000);
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

  const available = Boolean(state?.can_reset && !stopped && !busy && !error);
  /** 현장 확인을 받을 대상과 세대를 고정한다. 여기서는 로봇·서버 쓰기를 요청하지 않는다. */
  function ask(decision: Confirmation["decision"]) {
    if (!state || !available || (decision === "resume" && !state.can_resume))
      return;
    setConfirmation({
      decision,
      revision: state.revision,
      orderId: state.order_id,
      label: state.stopped_label,
    });
  }

  /** 관리자가 확인한 실패 세대에만 재개 예약 또는 최종 실패 종료를 요청한다. */
  async function decide() {
    if (
      !confirmation ||
      !state ||
      !available ||
      action.current ||
      confirmation.revision !== state.revision
    )
      return;
    const selected = confirmation;
    setConfirmation(null);
    const controller = new AbortController();
    action.current = controller;
    setBusy(true);
    const timeout = setTimeout(() => controller.abort(), 5000);
    try {
      const next = await sendWorkcellDecision(
        selected.decision,
        selected.revision,
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
    <article
      className={`summary-card recovery-card hmi-recovery${available ? " available" : ""}`}
      aria-label="주문 복구"
      aria-live="polite"
      title={state?.message || error}
    >
      <div className="recovery-head">
        <span className="label">주문 복구</span>
        <span className="recovery-state">
          {busy
            ? "요청 중"
            : stopped
              ? "정지 중"
              : state?.resume_requested
                ? "재개 중"
                : available
                  ? "복구 가능"
                  : "비활성"}
        </span>
      </div>
      <div className="recovery-actions">
        <button
          type="button"
          className="primary"
          disabled={!available || !state?.can_resume}
          onClick={() => ask("resume")}
        >
          재개하기
        </button>
        <button
          type="button"
          disabled={!available}
          onClick={() => ask("initialize")}
        >
          초기 자세 세팅
        </button>
      </div>
      {state?.requires_reset && (
        <span className="recovery-point">{state.stopped_label}</span>
      )}
      {confirmation && (
        // 전체 화면을 막는 브라우저 confirm 대신 비모달 확인창을 사용한다.
        // 창을 띄운 상태에서도 상단의 독립 정지 버튼은 즉시 작동해야 한다.
        <section
          className="hmi-decision-confirm"
          role="dialog"
          aria-modal="false"
          aria-labelledby="hmi-decision-title"
          onKeyDown={(event) => {
            if (event.key === "Escape") setConfirmation(null);
          }}
        >
          <h2 id="hmi-decision-title">
            {confirmation.decision === "resume"
              ? "현장 복구 확인"
              : "초기 자세 정리 확인"}
          </h2>
          <p className="mono">{confirmation.orderId}</p>
          <p>
            {confirmation.decision === "resume"
              ? `안전 자세·빈 그리퍼·기본 TCP로 복귀했습니까? ${confirmation.label}부터 같은 주문을 이어갑니다.`
              : "초기 자세와 도구·케이크 정리를 마쳤습니까? 현재 주문을 최종 실패로 종료하고 다음 주문을 받습니다. 로봇은 자동 이동하지 않습니다."}
          </p>
          {confirmation.revision !== state?.revision && (
            <p role="alert">
              복구 대상이 변경됐습니다. 상태를 확인한 뒤 다시 선택하세요.
            </p>
          )}
          <div className="dialog-actions">
            <button type="button" onClick={() => setConfirmation(null)}>
              취소
            </button>
            <button
              type="button"
              className="primary"
              ref={confirmButton}
              disabled={!available || confirmation.revision !== state?.revision}
              onClick={() => void decide()}
            >
              확인
            </button>
          </div>
        </section>
      )}
    </article>
  );
}
