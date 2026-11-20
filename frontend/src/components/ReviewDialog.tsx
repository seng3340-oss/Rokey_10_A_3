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
      <div className="review-stage">
        <div
          id="review3d"
          className="model-view review-model"
          role="img"
          aria-label="케이크 3D 미리보기"
        ></div>
        {/* 안내를 미리보기 안에 겹쳐 버튼 영역과 떨어뜨린다. */}
        <div id="payload-panel" hidden={true}>
          <p id="result" role="status"></p>
        </div>
        {/* 제작 요청 후 미리보기 오른쪽에 공정 상태를 펼친다. */}
        <section
          id="order-status"
          className="order-status"
          aria-live="polite"
          aria-hidden="true"
        >
          <span className="overline">MY CAKE</span>
          <h2 id="order-title">케이크를 준비하고 있어요</h2>
          <p id="order-message">작업을 기다리고 있어요.</p>
          <progress
            id="order-progress"
            aria-label="전체 공정 진행률"
            max="100"
            value="0"
          />
          <strong id="order-percent">진행률 확인 중</strong>
          <p id="order-number"></p>
        </section>
      </div>
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
      <button id="pickup" className="pickup-button" hidden>
        <span id="pickup-ring" className="pickup-ring" aria-hidden="true">
          <span id="pickup-seconds">60</span>
        </span>
        <span>픽업을 해주세요</span>
      </button>
    </dialog>
  );
}
