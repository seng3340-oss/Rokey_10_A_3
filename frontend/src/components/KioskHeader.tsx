/** 고객용 브랜드를 표시하는 머리글이다. */
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
    </header>
  );
}
