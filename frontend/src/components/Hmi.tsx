import { useEffect, useState } from "react";
import {
  fetchHmi,
  isStale,
  latestStage,
  stages,
  statusLabels,
} from "../api/hmi";
import type { Snapshot } from "../api/hmi";
import "../styles/hmi.css";

/** 실제 주문·명시적 공정 응답만 표시하는 읽기 전용 운영 화면이다. */
export function Hmi() {
  const [data, setData] = useState<Snapshot | null>(null);
  const [selectedId, setSelectedId] = useState("");
  const [error, setError] = useState("");
  const [lastSuccess, setLastSuccess] = useState(0);
  const [now, setNow] = useState(Date.now());
  const [refresh, setRefresh] = useState(0);
  useEffect(() => {
    // 이전 주문의 느린 응답이 새 선택을 덮어쓰지 않도록 취소·해제 상태를 함께 검사한다.
    let stopped = false;
    let timer: ReturnType<typeof setTimeout>;
    const controller = new AbortController();
    async function poll() {
      try {
        const snapshot = await fetchHmi(selectedId, controller.signal);
        if (!stopped) {
          setData(snapshot);
          setError("");
          setLastSuccess(Date.now());
        }
      } catch (e) {
        if (!stopped)
          setError(e instanceof Error ? e.message : "서버 조회 오류");
      } finally {
        // 요청이 완료된 뒤 1초를 기다려 느린 서버에 조회가 겹치지 않게 한다.
        if (!stopped) timer = setTimeout(poll, 1000);
      }
    }
    void poll();
    return () => {
      stopped = true;
      controller.abort();
      clearTimeout(timer);
    };
  }, [selectedId, refresh]);
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);
  const stale = isStale(lastSuccess, now, error);
  // 선택이 바뀐 뒤 응답 전에는 이전 주문을 새 주문처럼 표시하지 않는다.
  const order =
    data?.selected && (!selectedId || data.selected.order_id === selectedId)
      ? data.selected
      : null;
  const events = order
    ? (data?.events ?? []).filter((e) => e.order_id === order.order_id)
    : [];
  const runtime = data?.runtime;
  const heartbeatFresh =
    Boolean(runtime?.fresh) &&
    Boolean(runtime?.updated_at) &&
    now - Date.parse(runtime!.updated_at!) <= 8000;
  const stageState = (key: string) =>
    latestStage(events, key)?.stage_state ?? "unknown";
  const mode = order?.process_mode ?? "unknown";
  const visibleStages = stages.filter(([key]) =>
    mode === "drawing"
      ? key === "drawing"
      : mode === "pen"
        ? ["pen_pick", "drawing", "pen_return"].includes(key)
        : true,
  );
  const lastCommand = events.find((e) => e.stage_state === "running");
  return (
    <div className="hmi-app">
      <header>
        <a href="/hmi">
          <b>CAKE DRAWING</b>
          <span>실제 주문 · 공정 모니터</span>
        </a>
        <a href="/">고객 화면 ↗</a>
      </header>
      <main className="hmi-dashboard">
        <div className="hmi-title">
          <div>
            <p className="eyebrow">OPERATIONS / READ ONLY</p>
            <h1>전체 공정 현황</h1>
            <p>주문과 실행 노드가 보낸 단계·오류를 1초 간격으로 조회합니다.</p>
          </div>
          <button onClick={() => setRefresh((v) => v + 1)}>
            지금 새로고침
          </button>
        </div>
        <div className="hmi-safety">
          물리 안전 상태: 확인 불가 · E-stop 신호 미연결. 이 화면은 로봇
          운전·정지 기능을 제공하지 않습니다.
        </div>
        {stale && (
          <div className="hmi-error" role="alert">
            {error || "주문 서버 응답 대기 중"} · 표시된 이전 데이터는 현재
            상태가 아닐 수 있습니다.
          </div>
        )}
        {runtime?.last_error_message && (
          <div className="hmi-error" role="alert">
            최근 실행 노드 오류 · {runtime.last_error_source} ·{" "}
            {runtime.last_error_at
              ? new Date(runtime.last_error_at).toLocaleTimeString("ko-KR")
              : "시각 미수신"}
            <p>{runtime.last_error_message}</p>
          </div>
        )}
        <section className="hmi-stats" aria-label="연결 및 주문 요약">
          <div>
            <label>주문 API</label>
            <strong>{stale ? "확인 필요" : "응답 정상"}</strong>
            <small>
              마지막 성공{" "}
              {lastSuccess
                ? new Date(lastSuccess).toLocaleTimeString("ko-KR")
                : "없음"}
            </small>
          </div>
          <div>
            <label>TM 연결 관측</label>
            <strong>
              {stale || !heartbeatFresh
                ? "갱신 없음"
                : runtime?.task_server_ready
                  ? "발견됨"
                  : "대기 중"}
            </strong>
            <small>
              브릿지{" "}
              {runtime?.updated_at
                ? new Date(runtime.updated_at).toLocaleTimeString("ko-KR")
                : "미수신"}{" "}
              · 로봇 안전 확인 아님
            </small>
          </div>
          <div>
            <label>실행 설정</label>
            <strong>
              {runtime?.execution_mode === "real"
                ? "실제 로봇"
                : runtime?.execution_mode === "simulation"
                  ? "시뮬레이션"
                  : "미확인"}
            </strong>
            <small>
              {mode === "full"
                ? "전체 공정"
                : mode === "pen"
                  ? "펜 순환"
                  : mode === "drawing"
                    ? "드로잉"
                    : "설정 미수신"}{" "}
              · 브릿지 보고값
            </small>
          </div>
          <div>
            <label>최근 주문</label>
            <strong>{data?.orders.length ?? 0}건</strong>
            <small>최신 100건 · 이력은 서버에 보존</small>
          </div>
        </section>
        <div className="hmi-grid">
          <section className="hmi-card">
            <h2>실제 주문 목록</h2>
            <label className="follow">
              <input
                type="checkbox"
                checked={!selectedId}
                onChange={(e) =>
                  setSelectedId(e.target.checked ? "" : (order?.order_id ?? ""))
                }
              />
              최신 주문 따라가기
            </label>
            <div className="table-wrap">
              <table>
                <thead>
                  <tr>
                    <th>주문</th>
                    <th>면</th>
                    <th>상태</th>
                  </tr>
                </thead>
                <tbody>
                  {data?.orders.map((item) => (
                    <tr
                      key={item.order_id}
                      className={
                        item.order_id === order?.order_id ? "selected" : ""
                      }
                    >
                      <td>
                        <button
                          className="order-link"
                          onClick={() => setSelectedId(item.order_id)}
                        >
                          {item.order_id}
                        </button>
                      </td>
                      <td>
                        {item.drawing_surface === "top" ? "윗면" : "옆면"}
                      </td>
                      <td>{statusLabels[item.state] ?? item.state}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
            {!data?.orders.length && <p>접수된 주문이 없습니다.</p>}
          </section>
          <section className="hmi-card">
            <h2>{order?.order_id ?? "주문을 선택하세요"}</h2>
            <div className={"order-state " + (order?.state ?? "")}>
              {order ? (statusLabels[order.state] ?? order.state) : "대기"}
            </div>
            <p>{order?.message || "현재 주문의 상태 응답을 기다립니다."}</p>
            <p>최근 작업: {lastCommand?.message || order?.phase || "미수신"}</p>
            <progress
              max={100}
              value={
                order?.state === "completed" ? 100 : (order?.progress_pct ?? 0)
              }
            />
            <small>
              진행률은 실행 노드 보고값이며 실제 필기 품질 측정값이 아닙니다.
            </small>
            <p>
              마지막 주문 갱신:{" "}
              {order
                ? new Date(order.updated_at).toLocaleTimeString("ko-KR")
                : "—"}{" "}
              · 소요 {order?.elapsed_time_sec.toFixed(1) ?? "—"}초
            </p>
            {order?.error_code && (
              <div className="hmi-error" role="alert">
                {order.error_code}:{" "}
                {order.message || "아래 오류 이력을 확인하세요."}
              </div>
            )}
            <ol className="process-steps">
              {visibleStages.map(([key, label]) => {
                const event = latestStage(events, key);
                const state = stageState(key);
                return (
                  <li key={key} className={state}>
                    <span className="stage-dot" />
                    <span>{label}</span>
                    <b>
                      {
                        (
                          {
                            running: "진행 중",
                            completed: "완료",
                            failed: "실패",
                            unknown: "기록 대기",
                          } as Record<string, string>
                        )[state]
                      }
                    </b>
                    {event?.stage_state === "failed" && (
                      <small>{event.message}</small>
                    )}
                  </li>
                );
              })}
            </ol>
            <small>
              시작·완료 응답이 없는 단계는 완료로 추정하지 않습니다. 통신 단절
              시 마지막 기록을 유지합니다.
            </small>
          </section>
        </div>
        <section className="hmi-card">
          <h2>선택 주문의 공정·오류 이력</h2>
          <p>서버에 저장된 최근 300개 이벤트 · 새로고침 후에도 보존됩니다.</p>
          {!events.length && <p>저장된 이벤트가 없습니다.</p>}
          <div className="event-list">
            {events.map((event) => (
              <div
                key={event.id}
                className={"hmi-log " + (event.error_code ? "error" : "")}
              >
                <time>
                  {new Date(event.created_at).toLocaleTimeString("ko-KR")}
                </time>
                <b>{event.error_code || event.stage_state || event.kind}</b>
                <span>{event.message || event.phase || event.kind}</span>
              </div>
            ))}
          </div>
        </section>
      </main>
    </div>
  );
}
