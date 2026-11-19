import "../styles/workspace.css";

/** 고객 작성 화면과 읽기 전용 운영 화면을 나란히 연다. 장비나 가상 환경을 시작하지 않는다. */
export function OperationWorkspace() {
  return (
    <div className="operation-workspace">
      <header>
        <b>케이크 드로잉 · 운영 화면</b>
        <span>그림을 제출하면 오른쪽에서 작업 진행을 확인할 수 있습니다.</span>
      </header>
      <main>
        <section>
          <h1>고객 화면</h1>
          <iframe title="사용자 UI" src="/" />
        </section>
        <section>
          <h1>관리자 HMI</h1>
          <iframe title="관리자 HMI" src="/hmi" />
        </section>
      </main>
    </div>
  );
}
