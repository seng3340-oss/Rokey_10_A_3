import { useEffect, useState } from "react";
import { readHmi } from "../api/hmi";
import "../styles/hmi.css";
export function Hmi() {
  const [data, setData] = useState<Awaited<ReturnType<typeof readHmi>> | null>(
    null,
  );
  const [error, setError] = useState("");
  const [refresh, setRefresh] = useState(0);
  const [updated, setUpdated] = useState(0);
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    let disposed = false;
    let poll: ReturnType<typeof setTimeout>;
    let controller: AbortController;
    async function load() {
      controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 5000);
      try {
        const result = await readHmi(controller.signal);
        if (!disposed) {
          setData(result);
          setUpdated(Date.now());
          setError("");
        }
      } catch {
        if (!disposed)
          setError(
            "서버 연결을 확인할 수 없습니다. 표시된 값은 현재 장비 상태가 아닐 수 있습니다.",
          );
      } finally {
        clearTimeout(timeout);
        if (!disposed) poll = setTimeout(load, 3000);
      }
    }
    void load();
    return () => {
      disposed = true;
      clearTimeout(poll);
      controller?.abort();
    };
  }, [refresh]);
  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);
  const stale =
    !data || !!error || now - Date.parse(data.state.updated_at) > 10000;
  return (
    <div className="hmi-app">
      <header>
        <a href="/hmi">
          <b>CAKE DRAWING</b>
          <span>운영 콘솔</span>
        </a>
        <a href="/">고객 화면 ↗</a>
      </header>
      <div className="hmi-shell">
        <aside>
          <b>STATION 01</b>
          <p className="chosen">제작 현황</p>
          <p>M0609 / RG2</p>
          <small>윗면·옆면 드로잉 MVP</small>
        </aside>
        <main>
          <div className="hmi-title">
            <div>
              <h1>제작 현황</h1>
              <p>주문 대기열과 장비 상태를 확인합니다.</p>
            </div>
            <button onClick={() => setRefresh((n) => n + 1)}>새로고침</button>
          </div>
          {(error || !data) && (
            <div className="hmi-error" role="status">
              {error || "운영 서버에 연결하는 중입니다."}
            </div>
          )}
          <section className="hmi-stats">
            <div>
              <label>로봇 연결</label>
              <strong>
                {stale
                  ? "확인 불가"
                  : data?.state.robot_connected
                    ? "연결됨"
                    : "미연결"}
              </strong>
              <small>
                {stale ? "실시간 상태를 확인해주세요" : "ROS 브리지 응답 기준"}
              </small>
            </div>
            <div>
              <label>현재 작업 단계</label>
              <strong>{stale ? "—" : data?.state.stage}</strong>
              <small>컨트롤러 작업 상태</small>
            </div>
            <div>
              <label>조회된 주문</label>
              <strong>{data ? data.orders.length : "—"}</strong>
              <small>{stale ? "현재 수치 확인 불가" : "서버 응답 기준"}</small>
            </div>
            <div>
              <label>마지막 화면 갱신</label>
              <strong>
                {updated ? new Date(updated).toLocaleTimeString("ko-KR") : "—"}
              </strong>
              <small>
                {stale ? "미연결 또는 데이터 지연" : "3초 간격 조회"}
              </small>
            </div>
          </section>
          <div className="hmi-panels">
            <section>
              <h2>주문 대기열</h2>
              <div className="table-wrap">
                <table>
                  <thead>
                    <tr>
                      <th>주문 ID</th>
                      <th>그리기 면</th>
                      <th>상태</th>
                    </tr>
                  </thead>
                  <tbody>
                    {data?.orders.map((o) => (
                      <tr key={o.order_id}>
                        <td>{o.order_id}</td>
                        <td>{o.drawing_surface === "top" ? "윗면" : "옆면"}</td>
                        <td>{o.status}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
              {!data?.orders.length && (
                <p className="empty">
                  {data
                    ? "조회된 주문이 없습니다."
                    : "서버 연결 후 주문이 표시됩니다."}
                </p>
              )}
            </section>
            <section>
              <h2>작업·오류 로그</h2>
              {!data?.logs.length && (
                <p className="empty">
                  {data
                    ? "조회된 로그가 없습니다."
                    : "서버 연결 후 로그가 표시됩니다."}
                </p>
              )}
              {data?.logs.map((l, i) => (
                <div className="hmi-log" key={`${l.timestamp}-${i}`}>
                  <time>{l.timestamp}</time>
                  <b>{l.level}</b>
                  <span>{l.message}</span>
                </div>
              ))}
            </section>
          </div>
          <footer>
            현재 HMI는 조회 기능을 준비한 상태입니다. 작업 시작·취소와 운영자
            권한은 백엔드 계약 확정 후 연결합니다. 웹 중단은 물리적 비상정지를
            대신하지 않습니다.
          </footer>
        </main>
      </div>
    </div>
  );
}
