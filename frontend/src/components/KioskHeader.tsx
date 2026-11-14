/** 고객용 브랜드 표시와 운영 화면 진입 링크를 제공하는 공통 머리글이다. */
export function KioskHeader() {
  return (
    <header className="masthead">
      {/* 브랜드 링크는 새 고객 화면으로 돌아가는 진입점이다. */}
      <a href="/" className="logo">
        <span className="logo-symbol">c.</span>
        <span>
          케이크 드로잉<small>로봇 레터링 체험</small>
        </span>
      </a>
      {/* 화면 폭에 따른 운영 링크 표시는 기존 CSS 반응형 규칙을 따른다. */}
      <div className="header-right">
        <span className="preview-label">레터링 이벤트</span>
        <a href="/hmi">운영 화면</a>
      </div>
    </header>
  );
}
