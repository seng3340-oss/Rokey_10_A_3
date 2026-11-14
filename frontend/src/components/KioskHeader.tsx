export function KioskHeader() {
  return (
    <header className="masthead">
      <a href="/" className="logo">
        <span className="logo-symbol">c.</span>
        <span>
          케이크 드로잉<small>로봇 레터링 체험</small>
        </span>
      </a>
      <div className="header-right">
        <span className="preview-label">레터링 이벤트</span>
        <a href="/hmi">운영 화면</a>
      </div>
    </header>
  );
}
