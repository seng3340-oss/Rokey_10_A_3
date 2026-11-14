export function ReviewDialog() {
  return (
    <dialog id="modal">
      <header className="review-heading">
        <div>
          <span className="overline">제출 전 미리보기</span>
          <h2>내 케이크, 한 바퀴 확인해볼까요?</h2>
        </div>
        <button id="back" className="quiet">
          수정하기
        </button>
      </header>
      <div className="idle-notice" role="status">
        <div id="idle-ring" className="idle-ring">
          <span id="idle-seconds">60</span>
        </div>
        <span>
          조작이 없으면 <b>수정 화면으로 돌아갑니다.</b>
          <small>그린 그림은 그대로 남아 있어요.</small>
        </span>
      </div>
      <div
        id="review3d"
        className="model-view review-model"
        role="img"
        aria-label="제출 전 전체 케이크 3D 확인"
      ></div>
      <p className="rotate-note">
        그림을 확인하며 손가락으로 케이크를 돌려보세요.
      </p>
      <div className="review-controls">
        <button data-review-view="angle">전체 보기</button>
        <button data-review-view="top">윗면 보기</button>
        <button data-review-view="side">옆면 보기</button>
      </div>
      <button id="confirm" className="primary">
        선택한 면 제작 요청
      </button>
      <div id="payload-panel" hidden={true}>
        <p id="result" role="status"></p>
        <pre id="payload"></pre>
        <button id="download" className="quiet">
          선택한 면 JSON 내려받기
        </button>
      </div>
    </dialog>
  );
}
