import { DRAFT_CHANNEL, parseDraft } from "../api/draftPreview";
import type { DraftPreview } from "../api/draftPreview";
import { useEffect, useRef, useState } from "react";
import {
  phaseLabel,
  parseDetail,
  parseRuntime,
  isOrderFeedbackFresh,
  parsePage,
  readJson,
  collectOrderChanges,
  orderLabels as labels,
  parseRobotStop,
  sendRobotStop,
  surfaceDrawing,
} from "../api/hmi";
import type {
  Monitor,
  OrderDetail,
  OrderPage,
  OrderLog,
  RuntimeMonitor,
  RobotStop,
} from "../api/hmi";
import type { DrawingPathPayload } from "../api/drawing";
import { HmiCake3D } from "./HmiCake3D";
import "../styles/hmi.css";

/** 원본 픽셀 그대로 표시한다. 측면은 입력 캔버스이며 실제 원뿔대 전개도가 아니다. */
function Drawing({
  drawing,
  faded = false,
  label = "고객 원본 그림",
}: {
  drawing: DrawingPathPayload;
  faded?: boolean;
  label?: string;
}) {
  return (
    <svg
      viewBox={`0 0 ${drawing.canvas_width} ${drawing.canvas_height}`}
      role="img"
      aria-label={label}
      className={faded ? "hmi-drawing faded" : "hmi-drawing"}
    >
      {drawing.strokes.map((stroke, index) => (
        <polyline
          key={index}
          points={stroke.points.map((p) => `${p.x},${p.y}`).join(" ")}
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

/** 주문은 3초마다 순차 조회한다. 이전 요청 종료 뒤 예약하여 응답 역전·중복을 막는다. */
export function Hmi() {
  const [draft, setDraft] = useState<DraftPreview | null>(null);
  // 작성 중 그림은 주문과 분리해 보여준다. 수신만 하며 제작 요청을 만들지 않는다.
  useEffect(() => {
    if (typeof BroadcastChannel === "undefined") return;
    const channel = new BroadcastChannel(DRAFT_CHANNEL);
    channel.onmessage = (event) => {
      const preview = parseDraft(event.data);
      if (preview) setDraft(preview);
    };
    channel.postMessage({ kind: "request-draft" });
    return () => channel.close();
  }, []);
  const [page, setPage] = useState<OrderPage>({ items: [], total: 0 });
  const [selected, setSelected] = useState("");
  const [detail, setDetail] = useState<OrderDetail | null>(null);
  const [monitor, setMonitor] = useState<Monitor | null>(null);
  const [runtime, setRuntime] = useState<RuntimeMonitor | null>(null);
  const [surface, setSurface] = useState<"top" | "side">("top");
  // 고객 원본 카드의 보기 방식. true면 두 면을 입힌 3D 케이크, false면 선택 면(surface)의 2D 원본.
  const [view3d, setView3d] = useState(false);
  const [error, setError] = useState("");
  const [health, setHealth] = useState(false);
  const [updated, setUpdated] = useState("");
  const [refresh, setRefresh] = useState(0);
  const [guide, setGuide] = useState(false);
  // 소프트웨어 정지 상태. 주문 조회(3초)와 별도로 1초마다 갱신해 정지 반영을 빨리 보여준다.
  const [stopState, setStopState] = useState<RobotStop | null>(null);
  const [stopError, setStopError] = useState("");
  const [logs, setLogs] = useState<(OrderLog & { time: string })[]>([]);
  // 수동 선택은 유지하되 새 주문으로 자동 전환할 때만 원본의 면과 데모를 초기화한다.
  const displayedOrder = useRef("");
  const seenOrders = useRef(new Map<string, string>());
  const lastError = useRef("");

  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    let pendingController: AbortController | null = null;
    async function poll() {
      const controller = new AbortController();
      pendingController = controller;
      // 연결이 끊겨도 무한 대기하지 않으며 화면 종료 시 전체 요청을 중단한다.
      const timeout = setTimeout(() => controller.abort(), 10000);
      try {
        const next = parsePage(
          await readJson("/api/orders?limit=100", controller.signal),
        );
        const server = (await readJson("/api/health", controller.signal)) as {
          status?: string;
        };
        if (server?.status !== "ok")
          throw new Error("서버 상태 응답을 확인할 수 없습니다.");
        let connection: RuntimeMonitor | null = null;
        // 이전 백엔드에서도 주문 조회는 유지하고 연결 미관측을 명확히 표시한다.
        try {
          connection = parseRuntime(
            await readJson("/api/monitor", controller.signal),
          );
        } catch {
          if (controller.signal.aborted) throw new Error("조회 시간 초과");
        }
        const id = selected || next.items[0]?.order_id;
        let order: OrderDetail | null = null;
        let meta: Monitor | null = null;
        if (id) {
          order = parseDetail(
            await readJson(
              `/api/orders/${encodeURIComponent(id)}`,
              controller.signal,
            ),
          );
          // 이 확장 조회가 없는 이전 서버에서도 기존 주문 API와 원본 비교는 사용할 수 있다.
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
            // 확장 계약 미지원은 허용하되 시간 초과는 전체 조회 오류로 알린다.
            if (controller.signal.aborted) throw new Error("조회 시간 초과");
          }
        }
        if (!active) return;
        // 목록 뒤에 받은 상세가 더 최신일 수 있으므로 선택 행과 상세의 상태를 맞춘다.
        if (order) {
          const { drawing: _drawing, ...summary } = order;
          next.items = next.items.map((item) =>
            item.order_id === summary.order_id ? summary : item,
          );
        }
        if ((order?.order_id || "") !== displayedOrder.current) {
          displayedOrder.current = order?.order_id || "";
          setSurface(order?.drawing_surface || "top");
        }
        setPage(next);
        setDetail(order);
        setMonitor(meta);
        setRuntime(connection);
        setHealth(true);
        setError("");
        setUpdated(new Date().toLocaleTimeString("ko-KR"));
        lastError.current = "";
        const observed = collectOrderChanges(seenOrders.current, next.items);
        seenOrders.current = observed.signatures;
        if (observed.changes.length) {
          const time = new Date().toLocaleTimeString("ko-KR");
          setLogs((old) =>
            [
              ...observed.changes.map((change) => ({ ...change, time })),
              ...old,
            ].slice(0, 50),
          );
        }
      } catch (cause) {
        if (!active) return;
        const message = cause instanceof Error ? cause.message : "조회 오류";
        setError(
          controller.signal.aborted
            ? "조회 시간 초과 · 3초 뒤 다시 확인합니다."
            : message,
        );
        setHealth(false);
        if (lastError.current !== message) {
          lastError.current = message;
          setLogs((old) =>
            [
              {
                time: new Date().toLocaleTimeString("ko-KR"),
                message: `경고 · ${message}`,
                severity: "warning" as const,
              },
              ...old,
            ].slice(0, 50),
          );
        }
      } finally {
        clearTimeout(timeout);
        if (active) timer = setTimeout(poll, 3000);
      }
    }
    void poll();
    return () => {
      active = false;
      pendingController?.abort();
      clearTimeout(timer);
    };
  }, [selected, refresh]);

  // 정지 상태는 1초마다 순차 조회한다. 이전 요청이 끝난 뒤 다음 조회를 예약한다.
  useEffect(() => {
    let active = true;
    let timer: ReturnType<typeof setTimeout>;
    let pending: AbortController | null = null;
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
        // 조회 실패 시 마지막 상태를 정상으로 간주하지 않도록 오류를 함께 표시한다.
        if (active && !controller.signal.aborted)
          setStopError(
            reason instanceof Error ? reason.message : "정지 상태 조회 실패",
          );
        if (active && controller.signal.aborted)
          setStopError("정지 상태 조회 시간 초과");
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

  /** 정지 버튼: 확인 없이 즉시 전송한다. 여러 번 눌러도 매번 정지를 다시 보낸다. */
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

  // 헤더 문구: 정지 여부 → 로봇 적용 여부 → 브릿지 연결 순으로 가장 중요한 정보를 고른다.
  const stopped = stopState?.stopped === true;
  const stopTitle = stopped
    ? stopState.applied
      ? "소프트웨어 정지 중 · 로봇 정지 적용됨"
      : stopState.bridge_listening
        ? "소프트웨어 정지 중 · 로봇 적용 확인 대기"
        : "소프트웨어 정지 요청됨 · 브릿지 미연결"
    : "안전 상태: 확인 불가";
  // 정지 해제 기능은 두지 않는다. 다시 운용하려면 현장 정리 후 실행기를 재시작한다.
  const stopDetail =
    stopError ||
    (stopped
      ? stopState.bridge_listening
        ? `${stopState.message} · 재개: 현장 정리 후 실행기 재시작`
        : "정지 요청이 로봇에 전달되지 않았습니다. 물리 E-stop을 사용하세요."
      : stopState && !stopState.bridge_listening
        ? "물리 E-stop 신호 미수신 · 브릿지 미연결: 소프트웨어 정지가 로봇에 전달되지 않습니다."
        : "물리 E-stop 신호 미수신 · 소프트웨어 정지는 인증된 안전 기능이 아닙니다.");

  // 한 주문에는 윗면만, 옆면만, 또는 윗면+옆면(additional_paths)이 들어올 수 있다.
  // 선택 면의 원본을 주문 안에서만 찾고, 없는 면을 만들거나 다른 주문과 섞지 않는다.
  const drawing = surfaceDrawing(detail?.drawing, surface);
  const stale = !health && Boolean(updated);

  return (
    <div className="hmi-app">
      {/* 모든 화면에 안전 상태를 고정하며 HTTP 성공을 안전 확인으로 해석하지 않는다. */}
      {/* 정지 중에는 헤더 전체를 강조색으로 바꿔 어느 화면에서든 정지 상태를 알 수 있게 한다. */}
      <header
        className={stopped ? "hmi-safety stopped" : "hmi-safety"}
        aria-label="안전 상태"
        aria-live="assertive"
      >
        <div>
          <b>{stopTitle}</b>
          <span>{stopDetail}</span>
        </div>
        <div className="hmi-stop-actions">
          {/* 정지 버튼은 상태 조회 실패·정지 중에도 항상 누를 수 있다. */}
          <button type="button" className="hmi-stop" onClick={requestStop}>
            소프트웨어 정지
          </button>
          <button
            type="button"
            className="hmi-guide"
            onClick={() => setGuide(true)}
          >
            정지 안내
          </button>
        </div>
      </header>
      <div className="hmi-brand">
        <a href="/hmi">
          <b>CAKEBOT</b> <span>운영 모니터</span>
        </a>
        <a href="/">고객 화면 ↗</a>
      </div>
      <div className="hmi-shell">
        <aside>
          <h2>제작을 한눈에.</h2>
          <nav>
            <a href="#overview">주문·공정</a>
            <a href="#drawing">고객 원본</a>
            <a href="#logs">경고·로그</a>
          </nav>
          <p>M0609 / RG2</p>
          <small>
            주문·공정 조회 전용
            <br />
            로봇 제어는 소프트웨어 정지만
          </small>
        </aside>
        <main>
          {draft && (
            <section
              className="hmi-card hmi-live"
              aria-label="실시간 작성 미리보기"
            >
              <small>LIVE DRAFT · 작성 중 미리보기</small>
              <h2>고객이 그리고 있어요</h2>
              <p>
                미접수 초안 · 실행 안 함. 고객 화면에서 제작 요청을 전송하면
                주문으로 접수됩니다.
              </p>
              <div className="hmi-live-faces">
                {draft.drawings.map((drawing) => (
                  <figure key={drawing.drawing_surface}>
                    <figcaption>
                      {drawing.drawing_surface === "top" ? "상단" : "측면"} ·{" "}
                      {drawing.strokes.length}획
                    </figcaption>
                    <Drawing
                      drawing={drawing}
                      label={`작성 중 ${drawing.drawing_surface === "top" ? "상단" : "측면"}`}
                    />
                  </figure>
                ))}
              </div>
            </section>
          )}
          <div className="hmi-title" id="overview">
            <div>
              <small>OPERATIONS</small>
              <h1>제작 현황</h1>
            </div>
            <div className="hmi-controls">
              <button onClick={() => setRefresh((n) => n + 1)}>
                다시 조회
              </button>
              <button aria-pressed={!selected} onClick={() => setSelected("")}>
                최신 주문 따라가기
              </button>
            </div>
          </div>
          {/* 조회 오류가 있을 때만 경고 띠를 표시한다. 정상일 때는 안내 문구 없이 비워 둔다. */}
          {error && (
            <div className="hmi-banner warning" role="status">
              {`${error} ${stale ? "이전 조회 값을 표시 중 · 최신 상태가 아닙니다." : ""}`}
            </div>
          )}
          <section className="hmi-stats" aria-label="상태 요약">
            <article>
              <small>주문 API</small>
              <strong>{health ? "응답 있음" : "미확인"}</strong>
              <span>마지막 조회 {updated || "—"}</span>
            </article>
            <article>
              <small>저장 주문</small>
              <strong>{page.total}건</strong>
              <span>최신 100건 표시</span>
            </article>
            <article>
              <small>선택 공정</small>
              <strong>{detail ? phaseLabel(detail.phase) : "—"}</strong>
              <span>{detail ? labels[detail.status] : "주문 미선택"}</span>
            </article>
          </section>
          <section className="hmi-card hmi-runtime" aria-label="TM 연결 관측">
            <h2>TM 연결 관측</h2>
            <p>
              {runtime?.bridge_fresh && health
                ? runtime.task_server_ready
                  ? "TM 액션 서버 확인"
                  : "브릿지 응답 있음 · TM 서버 미확인"
                : "브릿지 갱신 없음 · 연결 상태 미확인"}
            </p>
            <p>
              최근 수신 {runtime?.updated_at || "없음"} ·{" "}
              {runtime?.active_order_id || "활성 주문 없음 / 미제공"}
            </p>
            <p>
              {runtime?.task_feedback_fresh && health
                ? "TM 작업 피드백 수신 중"
                : runtime?.active_order_id
                  ? "TM 작업 피드백 갱신 대기 / 지연"
                  : "작업 피드백 대기"}
            </p>
          </section>
          {/* 선택 주문 피드백: TM 연결 관측 바로 아래에 한 줄 요약 형태로 작게 둔다. */}
          <section
            className="hmi-card hmi-feedback"
            aria-label="선택 주문 피드백"
          >
            <div className="hmi-feedback-head">
              <h2>선택 주문 피드백</h2>
              <b>{detail?.order_id || "—"}</b>
              <span>{detail?.message || "공정 메시지가 없습니다."}</span>
            </div>
            <div className="hmi-feedback-progress">
              <label htmlFor="hmi-feedback-progress">
                TM 공정 진행률 {detail?.progress_pct.toFixed(1) ?? "—"}%
              </label>
              <progress
                id="hmi-feedback-progress"
                max="100"
                value={detail?.progress_pct || 0}
              />
            </div>
            <dl>
              <dt>상태 갱신</dt>
              <dd>{monitor?.updated_at || "미제공"}</dd>
              <dt>오류 코드</dt>
              <dd>{monitor?.error_code || "기록 없음"}</dd>
              <dt>현재 명령</dt>
              <dd>{monitor?.current_command || "미수신 / 작업 종료"}</dd>
              <dt>TM 피드백</dt>
              <dd>
                {isOrderFeedbackFresh(runtime, detail?.order_id) && health
                  ? "수신 중"
                  : detail &&
                      ["completed", "failed", "rejected", "resolved"].includes(
                        detail.status,
                      )
                    ? "종료된 주문"
                    : "대기 / 미확인"}
              </dd>
            </dl>
          </section>
          <div className="hmi-panels">
            <section className="hmi-card">
              <h2>주문·공정 상태 {stale && "· 이전 조회 값"}</h2>
              {page.items.length === 0 ? (
                <p>주문 데이터가 없습니다.</p>
              ) : (
                <div className="hmi-orders">
                  {page.items.map((order) => (
                    <button
                      key={order.order_id}
                      aria-pressed={detail?.order_id === order.order_id}
                      onClick={() => {
                        setSelected(order.order_id);
                        setDetail(null);
                        setMonitor(null);
                        setSurface(order.drawing_surface);
                      }}
                    >
                      <b>{order.order_id}</b>
                      <span>
                        {order.drawing_surface === "top" ? "상단" : "측면"} ·{" "}
                        {labels[order.status]}
                      </span>
                      <small>
                        {phaseLabel(order.phase)} ·{" "}
                        {order.progress_pct.toFixed(1)}%
                      </small>
                    </button>
                  ))}
                </div>
              )}
            </section>
            {/* 고객 원본 그림: 주문 목록 옆에서 선택 주문의 그림을 바로 확인한다.
                상단/측면은 해당 면의 2D 원본, 3D는 두 면을 한 케이크에 입혀 보여준다. */}
            <section className="hmi-card hmi-original" id="drawing">
              <div className="hmi-original-head">
                <h2>고객 원본 그림</h2>
                <div className="hmi-controls">
                  <button
                    aria-pressed={!view3d && surface === "top"}
                    onClick={() => {
                      setSurface("top");
                      setView3d(false);
                    }}
                  >
                    상단
                  </button>
                  <button
                    aria-pressed={!view3d && surface === "side"}
                    onClick={() => {
                      setSurface("side");
                      setView3d(false);
                    }}
                  >
                    측면
                  </button>
                  <button aria-pressed={view3d} onClick={() => setView3d(true)}>
                    3D
                  </button>
                </div>
              </div>
              {view3d ? (
                detail ? (
                  <HmiCake3D drawing={detail.drawing} />
                ) : (
                  <div className="hmi-placeholder">주문을 선택하세요</div>
                )
              ) : drawing ? (
                <Drawing drawing={drawing} />
              ) : (
                <div className="hmi-placeholder">
                  {detail
                    ? "이 주문에는 이 면의 그림이 없습니다"
                    : "주문을 선택하세요"}
                </div>
              )}
              <p className="hmi-muted">
                {view3d
                  ? "드래그해서 돌려 볼 수 있습니다. 원본 그림이며 실제 필기 결과가 아닙니다."
                  : `${surface === "top" ? "상단" : "측면"} 원본 · 고객이 제출한 좌표 그대로`}
              </p>
            </section>
          </div>
          <section className="hmi-card" id="logs">
            <h2>경고·오류 로그</h2>
            <p>
              이 브라우저에서 관측한 모든 표시 주문의 변화·조회 오류 최신
              50건입니다. 새로고침 전 이력과 3초 사이의 중간 전환은 보존하지
              않습니다.
            </p>
            {logs.map((log, i) => (
              <div
                className={`hmi-log ${log.severity}`}
                key={`${log.time}-${i}`}
              >
                <time>{log.time}</time>
                <span>
                  <b>
                    {log.severity === "error"
                      ? "오류"
                      : log.severity === "warning"
                        ? "경고"
                        : "상태"}
                  </b>{" "}
                  · {log.message}
                </span>
              </div>
            ))}
          </section>
          <footer>
            주문·공정 정보는 읽기 전용입니다. 소프트웨어 정지만 로봇에 전달되며
            물리적 비상정지는 장비의 E-stop을 사용하세요.
          </footer>
        </main>
      </div>
      {guide && (
        <div className="hmi-modal" onClick={() => setGuide(false)}>
          <section
            role="dialog"
            aria-modal="true"
            aria-labelledby="stop-title"
            onClick={(e) => e.stopPropagation()}
            onKeyDown={(e) => {
              if (e.key === "Escape") setGuide(false);
            }}
          >
            <h2 id="stop-title">정지 안내</h2>
            <p>
              위험 시 장비의 물리 E-stop을 조작하고 현장 안전 절차를 따르세요.
            </p>
            <p>
              소프트웨어 정지는 로봇의 현재 이동을 Quick stop으로 멈추고, 이후
              이동·그리퍼 명령과 새 주문을 차단합니다. 진행 중 주문은 실패로
              끝나며 이어서 실행되지 않습니다.
            </p>
            <p>
              정지 해제 기능은 없습니다. 펜던트로 들고 있는 오프너·마커와
              케이크를 정리한 뒤 실행기를 재시작해야 다시 주문을 받습니다.
            </p>
            <p>
              소프트웨어 정지는 인증된 안전 기능이 아니며 물리 E-stop을 대체하지
              않습니다. 브릿지가 연결되지 않았거나 통신이 끊기면 전달되지
              않습니다.
            </p>
            <button
              type="button"
              className="hmi-stop"
              onClick={() => {
                void requestStop();
                setGuide(false);
              }}
            >
              소프트웨어 정지
            </button>
            <button autoFocus onClick={() => setGuide(false)}>
              닫기
            </button>
          </section>
        </div>
      )}
    </div>
  );
}
