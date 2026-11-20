import { useEffect, useRef, useState } from "react";
import {
  phaseLabel,
  formatLogTime,
  parseDetail,
  parsePage,
  readJson,
  orderLabels,
  parseRobotStop,
  sendRobotStop,
  surfaceDrawing,
} from "../api/hmi";
import type {
  Monitor,
  OrderDetail,
  OrderPage,
  OrderLog,
  RobotStop,
} from "../api/hmi";
import {
  dashboardOrder,
  dashboardSurface,
  dashboardTimestamp,
  dashboardToday,
} from "../api/hmiDashboard";
import type { DrawingPathPayload } from "../api/drawing";
import { HmiCake3D } from "./HmiCake3D";
import { WorkcellRecoveryPanel } from "./WorkcellRecovery";
import { WorkcellLogs } from "./WorkcellLogs";
import "../styles/hmi.css";

/** 좌표를 변경하지 않고 선택 주문의 한 면을 별도 2D 원본으로 표시한다. */
function Drawing({
  drawing,
  label,
}: {
  drawing: DrawingPathPayload | null;
  label: string;
}) {
  if (!drawing) return <div className="face-empty">이 면의 그림 없음</div>;
  return (
    <svg
      viewBox={`0 0 ${drawing.canvas_width} ${drawing.canvas_height}`}
      role="img"
      aria-label={label}
      className="hmi-drawing"
    >
      {drawing.strokes.map((stroke) => (
        <polyline
          key={stroke.stroke_id}
          points={stroke.points
            .map((point) => `${point.x},${point.y}`)
            .join(" ")}
          fill="none"
          stroke="currentColor"
          strokeWidth="3"
          vectorEffect="non-scaling-stroke"
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      ))}
    </svg>
  );
}

/** 상태와 TM 경로 순번이 확인된 경우에만 상세 공정의 면 이름을 표시한다. */
function detailPhase(order: OrderDetail | null): string {
  if (!order) return "주문 미선택";
  if (order.status === "paused") return "복구 대기";
  if (order.status === "completed") return "제작 완료";
  const surface = dashboardSurface(order);
  return order.phase === "DRAWING" && surface
    ? `${surface === "top" ? "윗면" : "옆면"} 드로잉`
    : phaseLabel(order.phase);
}

/** 실제 주문·제어 계약을 유지하며 한 화면에 기록·상세·원본·로그를 함께 표시한다. */
export function Hmi() {
  const [page, setPage] = useState<OrderPage>({ items: [], total: 0 });
  // 빈 선택은 자동 모드다. 수동으로 고른 과거 주문은 1초 갱신에도 유지한다.
  const [selected, setSelected] = useState("");
  const [detail, setDetail] = useState<OrderDetail | null>(null);
  const [monitor, setMonitor] = useState<Monitor | null>(null);
  const [error, setError] = useState("");
  const [health, setHealth] = useState(false);
  const [updated, setUpdated] = useState("");
  const [stopState, setStopState] = useState<RobotStop | null>(null);
  const [stopError, setStopError] = useState("");
  const [recoveryError, setRecoveryError] = useState("");
  const [logs, setLogs] = useState<(OrderLog & { time: string })[]>([]);
  const lastError = useRef("");

  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    let pending: AbortController | null = null;
    /** 같은 시도의 목록·상세를 연결한다. 요청을 겹치지 않고 시작 간격을 1초로 맞춘다. */
    async function poll() {
      const started = performance.now();
      const controller = new AbortController();
      pending = controller;
      const timeout = setTimeout(() => controller.abort(), 10000);
      try {
        const [rawPage, server] = await Promise.all([
          readJson("/api/orders?limit=100", controller.signal),
          readJson("/api/health", controller.signal),
        ]);
        const next = parsePage(rawPage);
        if ((server as { status?: string })?.status !== "ok") {
          throw new Error("서버 상태 응답을 확인할 수 없습니다.");
        }
        const id = dashboardOrder(next.items, selected);
        let order: OrderDetail | null = null;
        let meta: Monitor | null = null;
        if (id) {
          order = parseDetail(
            await readJson(
              `/api/orders/${encodeURIComponent(id)}`,
              controller.signal,
            ),
          );
          try {
            const raw = (await readJson(
              `/api/orders/${encodeURIComponent(id)}/monitor`,
              controller.signal,
            )) as Monitor;
            if (
              typeof raw.updated_at === "string" &&
              typeof raw.error_code === "string" &&
              Number.isFinite(raw.elapsed_time_sec)
            )
              meta = raw;
          } catch {
            // 확장 계약이 없는 이전 서버에서도 주문과 원본은 계속 조회한다.
            if (controller.signal.aborted) throw new Error("조회 시간 초과");
          }
        }
        if (!active) return;
        if (order) {
          const { drawing: _drawing, ...summary } = order;
          next.items = next.items.map((item) =>
            item.order_id === summary.order_id ? summary : item,
          );
        }
        setPage(next);
        setDetail(order);
        setMonitor(meta);
        setHealth(true);
        setError("");
        setUpdated(dashboardTimestamp(new Date()));
        lastError.current = "";
      } catch (cause) {
        if (!active) return;
        const message = controller.signal.aborted
          ? "조회 시간 초과 · 자동으로 다시 확인합니다."
          : cause instanceof Error
            ? cause.message
            : "조회 오류";
        setError(message);
        setHealth(false);
        if (lastError.current !== message) {
          lastError.current = message;
          setLogs((previous) =>
            [
              {
                time: formatLogTime(new Date()),
                message,
                severity: "warning" as const,
              },
              ...previous,
            ].slice(0, 50),
          );
        }
      } finally {
        clearTimeout(timeout);
        if (active)
          timer = setTimeout(
            poll,
            Math.max(100, 1000 - (performance.now() - started)),
          );
      }
    }
    void poll();
    return () => {
      active = false;
      pending?.abort();
      clearTimeout(timer);
    };
  }, [selected]);

  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    let pending: AbortController | null = null;
    /** 정지 상태는 주문 선택과 독립적으로 조회해 어느 기록을 보더라도 최신 요청을 표시한다. */
    async function pollStop() {
      const controller = new AbortController();
      pending = controller;
      const timeout = setTimeout(() => controller.abort(), 3000);
      try {
        const next = parseRobotStop(
          await readJson("/api/robot/stop", controller.signal),
        );
        if (active) {
          setStopState(next);
          setStopError("");
        }
      } catch (reason) {
        if (active)
          setStopError(
            controller.signal.aborted
              ? "정지 상태 조회 시간 초과"
              : reason instanceof Error
                ? reason.message
                : "정지 상태 조회 실패",
          );
      } finally {
        clearTimeout(timeout);
        if (active) timer = setTimeout(pollStop, 1000);
      }
    }
    void pollStop();
    return () => {
      active = false;
      pending?.abort();
      clearTimeout(timer);
    };
  }, []);

  /** 확인창을 거치지 않고 즉시 정지 POST를 보낸다. 조회 실패 중에도 버튼은 사용 가능하다. */
  async function requestStop() {
    try {
      setStopState(await sendRobotStop());
      setStopError("");
    } catch (reason) {
      setStopError(
        `${reason instanceof Error ? reason.message : "정지 요청 실패"} · 물리 E-stop을 사용하세요.`,
      );
    }
  }

  const stopped = stopState?.stopped === true;
  const current = page.items.find(
    (order) => order.order_id === dashboardOrder(page.items),
  );
  const currentPhase =
    current?.order_id === detail?.order_id
      ? detailPhase(detail)
      : current
        ? phaseLabel(current.phase)
        : "주문 대기";
  const top = surfaceDrawing(detail?.drawing, "top");
  const side = surfaceDrawing(detail?.drawing, "side");
  const selectedSurface = dashboardSurface(detail);
  const step =
    detail?.phase === "COMPLETED"
      ? 4
      : detail?.phase === "FINISHING"
        ? 3
        : detail?.phase === "PREPARING"
          ? 0
          : detail?.phase === "DRAWING" && selectedSurface
            ? selectedSurface === "top"
              ? 1
              : 2
            : -1;
  const stopLabel = stopped
    ? stopState.applied
      ? "정지 중 · 적용됨"
      : "정지 요청 · 확인 대기"
    : "소프트웨어 정지";

  return (
    <div className="hmi-app">
      <main className="dashboard-app">
        <header className="topbar">
          <div className="brand">
            <span className="logo-symbol" aria-hidden="true">
              c.
            </span>
            <h1>
              케이크 드로잉<span>대시보드</span>
            </h1>
          </div>
          <div className="topbar-status">
            <time className="mono">{updated || "연결 확인 중"}</time>
          </div>
        </header>
        <div className="overview-row">
          <section className="summary-grid" aria-label="상태 요약">
            <article className="summary-card">
              <div>
                <span className="label">주문 API</span>
                <strong className="teal">
                  {health ? "응답 있음" : "미확인"}
                </strong>
                <p>마지막 조회 {updated.split(" ")[1] || "—"}</p>
              </div>
            </article>
            <article className="summary-card">
              <div>
                <span className="label">오늘 주문</span>
                <strong className="mono">
                  {dashboardToday(page)}
                  <span className="unit">건</span>
                </strong>
                <p>누적 {page.total}건</p>
              </div>
            </article>
            <article className="summary-card">
              <div>
                <span className="label">현재 공정</span>
                <strong className="phase-summary">{currentPhase}</strong>
                <p>{current?.order_id || "접수된 주문 없음"}</p>
              </div>
            </article>
          </section>
          <div className="stop-zone">
            <WorkcellRecoveryPanel
              stopped={stopped}
              onError={setRecoveryError}
            />
            <button
              type="button"
              className={`stop-button${stopped ? " is-stopped" : ""}`}
              onClick={() => void requestStop()}
              title="소프트웨어 정지 · 물리 E-stop 신호와 별개"
            >
              <span className="stop-square" />
              {stopLabel}
            </button>
          </div>
        </div>
        <div className="hmi-notices" aria-live="polite">
          {error && (
            <p role="alert">
              {error}
              {updated && " · 이전 조회 값을 표시 중입니다."}
            </p>
          )}
          {stopError && <p role="alert">{stopError}</p>}
          {recoveryError && <p role="alert">{recoveryError}</p>}
          {stopped && (
            <p role="status">
              {stopState.applied ? "로봇 정지 적용됨" : "로봇 적용 미확정"} ·{" "}
              {stopState.message} · 현장 정리 후 실행기를 재시작하세요.
            </p>
          )}
        </div>
        <div className="workspace">
          <section
            className="card history-card"
            aria-labelledby="hmi-history-title"
          >
            <div className="card-head">
              <h2 id="hmi-history-title">주문 기록</h2>
              <span className="muted">최근 100건</span>
            </div>
            <div className="history-caption">
              <span>주문 ID · 접수 일시</span>
              <span>상태</span>
            </div>
            <div
              className="order-list hmi-orders"
              tabIndex={0}
              aria-label="주문 기록"
            >
              {page.items.map((order) => (
                <button
                  type="button"
                  className="order"
                  key={order.order_id}
                  aria-pressed={detail?.order_id === order.order_id}
                  onClick={() => {
                    setSelected(order.order_id);
                    setDetail(null);
                    setMonitor(null);
                  }}
                >
                  <div>
                    <b>{order.order_id}</b>
                    <small className="mono">
                      {dashboardTimestamp(order.created_at)}
                    </small>
                  </div>
                  <span className={`status-tag ${order.status}`}>
                    {orderLabels[order.status]}
                  </span>
                </button>
              ))}
              {!page.items.length && (
                <p className="empty-state">주문 데이터가 없습니다.</p>
              )}
            </div>
            {selected && (
              <div className="history-footer">
                <span>선택한 주문 기록 표시 중</span>
                <button type="button" onClick={() => setSelected("")}>
                  현재 주문 보기 ↗
                </button>
              </div>
            )}
          </section>
          <section className="card process-card" aria-label="선택 주문 상세">
            <div className="card-head">
              <h2 className="mono">{detail?.order_id || "주문 미선택"}</h2>
              {detail && (
                <span className={`status-tag ${detail.status}`}>
                  {orderLabels[detail.status]}
                </span>
              )}
            </div>
            <div className="process-summary">
              <div className="ring">
                <svg viewBox="0 0 100 100" aria-hidden="true">
                  <circle className="track" cx="50" cy="50" r="40" />
                  <circle
                    className="value"
                    cx="50"
                    cy="50"
                    r="40"
                    style={{
                      strokeDashoffset:
                        251.33 * (1 - (detail?.progress_pct || 0) / 100),
                    }}
                  />
                </svg>
                <div className="ring-label">
                  <strong className="mono">
                    {detail ? Number(detail.progress_pct.toFixed(1)) : "—"}
                    <span>%</span>
                  </strong>
                  <small>전체 공정 진행률</small>
                </div>
              </div>
              <div className="process-meta">
                <h3>{detailPhase(detail)}</h3>
                <p>{detail?.message || "공정 메시지 대기"}</p>
                <dl className="command">
                  <dt>현재 명령</dt>
                  <dd>{monitor?.current_command || "미수신 / 작업 종료"}</dd>
                  <dt>오류 코드</dt>
                  <dd>
                    {monitor?.error_code || detail?.error_code || "기록 없음"}
                  </dd>
                  <dt>상태 갱신</dt>
                  <dd className="mono">
                    {monitor
                      ? dashboardTimestamp(monitor.updated_at)
                      : "미제공"}
                  </dd>
                </dl>
              </div>
            </div>
            <div className="steps" aria-label="선택 주문 공정 단계">
              {["케이크 준비", "윗면", "옆면", "회수·출고"].map(
                (label, index) => (
                  <span
                    className={`step${index < step ? " done" : index === step ? " current" : ""}`}
                    key={label}
                  >
                    <i>{index < step ? "✓" : index + 1}</i>
                    {label}
                  </span>
                ),
              )}
            </div>
          </section>
          <WorkcellLogs localLogs={logs} />
          <section
            className="card original-card"
            aria-labelledby="hmi-original-title"
          >
            <div className="card-head">
              <h2 id="hmi-original-title">고객 원본 그림</h2>
            </div>
            {detail ? (
              <HmiCake3D drawing={detail.drawing} />
            ) : (
              <div className="cake3d empty-state">주문을 선택하세요</div>
            )}
            <div className="original-note">
              <span className="mono">{detail?.order_id || "—"}</span>
            </div>
            <div className="original-faces">
              {(["top", "side"] as const).map((surface) => (
                <figure key={surface}>
                  <figcaption>
                    <b>{surface === "top" ? "윗면 원본" : "옆면 원본"}</b>
                    <span>
                      {(surface === "top" ? top : side)?.strokes.length || 0}획
                    </span>
                  </figcaption>
                  <div className="face-canvas">
                    <Drawing
                      drawing={surface === "top" ? top : side}
                      label={`${surface === "top" ? "윗면" : "옆면"} 고객 원본`}
                    />
                  </div>
                </figure>
              ))}
            </div>
          </section>
        </div>
      </main>
    </div>
  );
}
