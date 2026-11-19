import { DRAFT_CHANNEL, parseDraft } from "../api/draftPreview";
import type { DraftPreview } from "../api/draftPreview";
import { useEffect, useRef, useState } from "react";
import {
  phaseLabel,
  parseDetail,
  parsePlan,
  parseRuntime,
  isOrderFeedbackFresh,
  parsePage,
  readJson,
  collectOrderChanges,
  orderLabels as labels,
} from "../api/hmi";
import type {
  Monitor,
  OrderDetail,
  OrderPage,
  OrderLog,
  TaskPlan,
  RuntimeMonitor,
} from "../api/hmi";
import type { DrawingPayload } from "../api/drawing";
import "../styles/hmi.css";

/** 원본 픽셀 그대로 표시한다. 측면은 입력 캔버스이며 실제 원뿔대 전개도가 아니다. */
function Drawing({
  drawing,
  faded = false,
  label = "고객 원본 그림",
}: {
  drawing: DrawingPayload;
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
  const [plan, setPlan] = useState<TaskPlan | null>(null);
  const [runtime, setRuntime] = useState<RuntimeMonitor | null>(null);
  const [trajectoryWarning, setTrajectoryWarning] = useState("");
  const [surface, setSurface] = useState<"top" | "side">("top");
  const [overlay, setOverlay] = useState(false);
  const [error, setError] = useState("");
  const [health, setHealth] = useState(false);
  const [updated, setUpdated] = useState("");
  const [refresh, setRefresh] = useState(0);
  const [guide, setGuide] = useState(false);
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
        let path: TaskPlan | null = null;
        let pathWarning = "";
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
        if (order) {
          try {
            path = parsePlan(
              await readJson(
                `/api/orders/${encodeURIComponent(order.order_id)}/plan`,
                controller.signal,
              ),
              order,
            );
          } catch (cause) {
            if (controller.signal.aborted) throw new Error("조회 시간 초과");
            // 이전 서버의 404는 미지원 상태로 취급하고, 손상된/실패한 경로는 별도 안내한다.
            const text =
              cause instanceof Error ? cause.message : "경로 조회 실패";
            if (!text.includes("HTTP 404")) pathWarning = text;
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
        setPlan(path);
        setRuntime(connection);
        setTrajectoryWarning(pathWarning);
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

  // 한 주문에 한 면만 존재한다. 없는 반대 면을 자동 생성하거나 다른 주문과 섞지 않는다.
  const drawing =
    detail?.drawing.drawing_surface === surface ? detail.drawing : null;
  const visiblePlan = plan?.drawing_surface === surface ? plan : null;
  const stale = !health && Boolean(updated);

  return (
    <div className="hmi-app">
      {/* 모든 화면에 안전 상태를 고정하며 HTTP 성공을 안전 확인으로 해석하지 않는다. */}
      <header className="hmi-safety" aria-label="안전 상태">
        <div>
          <b>안전 상태: 확인 불가</b>
          <span>물리 E-stop 신호 미수신 · TM 연결은 안전 확인이 아닙니다.</span>
        </div>
        <button type="button" onClick={() => setGuide(true)}>
          정지 안내
        </button>
      </header>
      <div className="hmi-brand">
        <a href="/hmi">
          <b>CAKEBOT</b> <span>운영 모니터</span>
        </a>
        <a href="/">고객 화면 ↗</a>
      </div>
      <div className="hmi-shell">
        <aside>
          <small>STATION 01</small>
          <h2>제작을 한눈에.</h2>
          <nav>
            <a href="#overview">주문·공정</a>
            <a href="#drawing">드로잉 비교</a>
            <a href="#logs">경고·로그</a>
          </nav>
          <p>M0609 / RG2</p>
          <small>
            읽기 전용 HMI
            <br />
            로봇 제어 없음
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
              <small>OPERATIONS / READ ONLY</small>
              <h1>제작 현황</h1>
              <p>저장된 주문과 TM 공정 피드백을 확인합니다.</p>
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
          <div
            className={error ? "hmi-banner warning" : "hmi-banner"}
            role="status"
          >
            {error
              ? `${error} ${stale ? "이전 조회 값을 표시 중 · 최신 상태가 아닙니다." : ""}`
              : "읽기 전용 · 3초 간격 갱신. API 응답은 로봇 연결·안전 확인을 의미하지 않습니다."}
          </div>
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
            <article>
              <small>실제 / 시뮬레이션</small>
              <strong>
                {monitor?.execution_mode === "real"
                  ? "실물 설정"
                  : monitor?.execution_mode === "simulation"
                    ? "시뮬레이션 설정"
                    : "미확인"}
              </strong>
              <span>해당 주문의 브릿지 설정 · 실제 연결/안전 신호 아님</span>
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
                        setPlan(null);
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
            <section className="hmi-card">
              <h2>선택 주문 피드백</h2>
              <h3>{detail?.order_id || "—"}</h3>
              <p>{detail?.message || "공정 메시지가 없습니다."}</p>
              <label>
                TM 공정 진행률 {detail?.progress_pct.toFixed(1) ?? "—"}%
              </label>
              <progress max="100" value={detail?.progress_pct || 0} />
              <p className="hmi-muted">
                공정 진행률은 드로잉 완료 실측값이 아닙니다.
              </p>
              <dl>
                <dt>상태 갱신 시각</dt>
                <dd>{monitor?.updated_at || "미제공"}</dd>
                <dt>마지막 오류 코드</dt>
                <dd>{monitor?.error_code || "기록 없음 / 미제공"}</dd>
                <dt>현재 작업 명령</dt>
                <dd>{monitor?.current_command || "미수신 / 작업 종료"}</dd>
                <dt>ROS 연결·heartbeat</dt>
                <dd>
                  {isOrderFeedbackFresh(runtime, detail?.order_id) && health
                    ? "선택 주문의 TM 피드백 수신 중"
                    : detail &&
                        [
                          "completed",
                          "failed",
                          "rejected",
                          "resolved",
                        ].includes(detail.status)
                      ? "종료된 주문 기록"
                      : "선택 주문 피드백 대기 / 미확인"}
                </dd>
              </dl>
            </section>
          </div>
          <section className="hmi-card hmi-monitor" id="drawing">
            <div className="hmi-title">
              <div>
                <small>DRAWING MONITOR</small>
                <h2>원본과 TM 계획 제어점 비교</h2>
              </div>
              <div className="hmi-controls">
                <button
                  aria-pressed={surface === "top"}
                  onClick={() => setSurface("top")}
                >
                  상단
                </button>
                <button
                  aria-pressed={surface === "side"}
                  onClick={() => setSurface("side")}
                >
                  측면
                </button>
                <button
                  aria-pressed={overlay}
                  onClick={() => setOverlay(!overlay)}
                >
                  {overlay ? "나란히 보기" : "겹쳐 보기"}
                </button>
              </div>
            </div>
            <p>
              TM이 받은 GenerateTrajectory 결과를 표시합니다. 실제 TCP·잉크 측정
              경로는 미연결입니다. 진행률이나 원본 그림으로 실행 궤적을 추정하지
              않습니다.
            </p>
            <div
              className={overlay ? "hmi-comparison overlay" : "hmi-comparison"}
            >
              <figure>
                <figcaption>
                  고객 원본 · {surface === "top" ? "상단" : "측면"}
                </figcaption>
                {drawing ? (
                  <Drawing drawing={drawing} />
                ) : (
                  <div className="hmi-placeholder">이 면의 원본이 없습니다</div>
                )}
              </figure>
              <figure>
                <figcaption>TM 계획 제어점 · 실측 아님</figcaption>
                {visiblePlan ? (
                  <Drawing drawing={visiblePlan} label="TM 계획 경로" />
                ) : (
                  <div className="hmi-placeholder">TM 계획 경로 미수신</div>
                )}
              </figure>
            </div>
            {trajectoryWarning && (
              <p role="status" className="hmi-banner warning">
                계획 조회 경고 · {trajectoryWarning}
              </p>
            )}
            {visiblePlan && (
              <p className="hmi-path-metrics">
                계획 {visiblePlan.strokes.length}획 ·{" "}
                {visiblePlan.waypoint_count}개 웨이포인트 ·{" "}
                {visiblePlan.updated_at}
              </p>
            )}
            <p className="hmi-muted">
              제작 완료 상태는 실제 필기 품질·물리 안전 확인을 뜻하지 않습니다.
            </p>
          </section>
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
            API → HMI 읽기 전용. 소프트웨어 정지 요청은 미연결입니다. 물리적
            비상정지는 장비의 E-stop을 사용하세요.
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
              이 버튼은 안내를 엽니다. 정지 명령을 보내지 않았습니다. 소프트웨어
              정지 요청과 물리 E-stop은 별도 기능입니다.
            </p>
            <button disabled>소프트웨어 정지 요청 (미연결)</button>
            <button autoFocus onClick={() => setGuide(false)}>
              닫기
            </button>
          </section>
        </div>
      )}
    </div>
  );
}
