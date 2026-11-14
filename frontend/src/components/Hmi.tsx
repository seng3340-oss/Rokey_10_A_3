import "../styles/hmi.css";

/**
 * 대기열의 배치와 문구를 확인하기 위한 화면 전용 예시다.
 * 서버 응답 타입이나 향후 HMI 데이터 계약을 정의하지 않는다.
 */
const exampleOrders = [
  { id: "예시-001", surface: "윗면", status: "대기 (예시)" },
  { id: "예시-002", surface: "옆면", status: "접수 (예시)" },
];

// 고정 시각과 예시 문구를 사용해 실제 장비에서 수신한 로그와 구분한다.
const exampleLogs = [
  { time: "예시 10:00", level: "안내", message: "주문 접수 표시 예시입니다." },
  { time: "예시 10:01", level: "안내", message: "작업 대기 표시 예시입니다." },
];

/**
 * 승인된 운영 콘솔 배치를 보여주는 미연결 정적 화면이다.
 * 데이터 확정 전에는 네트워크 조회·자동 갱신·장비 제어를 수행하지 않는다.
 */
export function Hmi() {
  return (
    <div className="hmi-app">
      {/* 고객 화면으로 이동하는 링크는 정적 화면에서도 사용할 수 있다. */}
      <header>
        <a href="/hmi">
          <b>CAKE DRAWING</b>
          <span>운영 콘솔 · 예시 화면</span>
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
              <p>주문 대기열과 장비 상태의 화면 구성 예시입니다.</p>
            </div>
            {/* 연결 기능을 암시하지 않도록 동작 없이 비활성 상태를 유지한다. */}
            <button type="button" disabled>
              새로고침 (미연결)
            </button>
          </div>

          <div className="hmi-error" role="status">
            미연결 · 정적 예시 화면입니다. 아래 주문과 로그는 예시 데이터이며,
            실제 장비 상태가 아닙니다. HMI 데이터는 아직 확정되지 않았습니다.
          </div>

          {/* 실제 연결 상태·시각을 생성하지 않고 각 카드의 표시 범위를 명시한다. */}
          <section className="hmi-stats" aria-label="미연결 상태 및 예시 정보">
            <div>
              <label>로봇 연결</label>
              <strong>미연결</strong>
              <small>실제 연결 상태를 조회하지 않습니다</small>
            </div>
            <div>
              <label>현재 작업 단계</label>
              <strong>—</strong>
              <small>장비 데이터 미확정</small>
            </div>
            <div>
              <label>예시 주문</label>
              <strong>{exampleOrders.length}건</strong>
              <small>화면 확인용 고정 데이터</small>
            </div>
            <div>
              <label>마지막 화면 갱신</label>
              <strong>—</strong>
              <small>자동 갱신 없음 · 정적 화면</small>
            </div>
          </section>

          <div className="hmi-panels">
            {/* 식별자와 상태에도 예시임을 남겨 표만 보아도 실제 주문과 구분한다. */}
            <section>
              <h2>주문 대기열 · 예시 데이터</h2>
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
                    {exampleOrders.map((order) => (
                      <tr key={order.id}>
                        <td>{order.id}</td>
                        <td>{order.surface}</td>
                        <td>{order.status}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </section>

            <section>
              <h2>작업·오류 로그 · 예시 데이터</h2>
              {exampleLogs.map((log) => (
                <div className="hmi-log" key={log.time}>
                  <span>{log.time}</span>
                  <b>{log.level}</b>
                  <span>{log.message}</span>
                </div>
              ))}
            </section>
          </div>

          <footer>
            화면 구성 확인을 위한 예시입니다. HMI 조회·작업 시작·취소 기능은
            연결되어 있지 않습니다. 웹 중단은 물리적 비상정지를 대신하지
            않습니다.
          </footer>
        </main>
      </div>
    </div>
  );
}
