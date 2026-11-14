/** 두 면의 전체 모습을 확인하되, 제출은 선택한 한 면으로 제한하는 확인 창이다. */
export function ReviewDialog() {
  return (
    // 네이티브 dialog의 열기·닫기와 무조작 복귀는 필기 컨트롤러에서 관리한다.
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
      {/* 남은 초와 원형 진행률은 같은 마감 시각을 기준으로 함께 갱신한다. */}
      <div className="idle-notice" role="status">
        <div id="idle-ring" className="idle-ring">
          <span id="idle-seconds">60</span>
        </div>
        <span>
          조작이 없으면 <b>수정 화면으로 돌아갑니다.</b>
          <small>그린 그림은 그대로 남아 있어요.</small>
        </span>
      </div>
      {/* 처음 열 때 별도 3D 뷰를 만든다. 원본 텍스처는 일반 미리보기와 공유한다. */}
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
      {/* 실제 전송은 이 버튼에서만 시작한다. 응답 대기 중에는 중복 제출을 막는다. */}
      <button id="confirm" className="primary">
        선택한 면 제작 요청
      </button>
      {/* 제출을 시도한 뒤 요청 내용·접수 결과와 동일 데이터의 내려받기를 공개한다. */}
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
