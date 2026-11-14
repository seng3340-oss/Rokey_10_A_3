/**
 * 필기와 3D 미리보기를 나란히 배치한다. 동작은 mountDrawing이 고정 ID로 연결한다.
 * ID와 data 속성은 입력 면·시점·버튼의 연결 계약이므로 변경 시 컨트롤러도 확인한다.
 */
export function DrawingWorkspace() {
  return (
    <main>
      <div className="intro">
        <div>
          <span className="overline">내가 그리면, 로봇이 완성하는 케이크</span>
          <h1>어떤 그림을 올려볼까요?</h1>
        </div>
      </div>
      {/* 고객은 왼쪽에서 그리고 오른쪽에서 두 면의 배치를 확인한다. */}
      <div className="workbench">
        <section className="drawing-panel">
          <div className="drawing-toolbar">
            {/* 선택 면은 컨트롤러가 aria-pressed와 selected 클래스를 함께 갱신한다. */}
            <div className="surface-tabs" role="group" aria-label="그리기 면">
              <button
                data-surface="top"
                className="selected"
                aria-pressed="true"
              >
                윗면 그리기
              </button>
              <button data-surface="side" aria-pressed="false">
                옆면 그리기
              </button>
            </div>
            <span id="stroke-count">0획</span>
          </div>
          <div className="drawing-stage">
            <div className="stage-label">
              <span>
                펜 색상 <i className="ink" />
              </span>
            </div>
            {/* 표시 크기는 반응형이지만 저장 좌표계는 1024×768로 고정한다. */}
            <div className="canvas-wrap">
              <canvas
                id="draw"
                width="1024"
                height="768"
                aria-label="윗면에 그림 그리기"
              ></canvas>
              {/* 첫 유효 획이 생기면 숨긴다. 안내가 포인터 입력을 가로채지 않도록 CSS로 처리한다. */}
              <div id="empty-hint">
                여기에 그려주세요<span>손가락 또는 펜으로 그릴 수 있어요</span>
              </div>
            </div>
            <p id="surface-help">
              점선 안쪽에 그려주세요. 케이크 윗면에 배치됩니다.
            </p>
          </div>
          {/* 상태 문구와 현재 면에만 적용되는 편집 도구를 모은다. */}
          <div className="drawing-bottom">
            <p id="hint" role="status"></p>
            <div>
              <button id="undo">한 획 취소</button>
              <button id="clear">이 면 지우기</button>
            </div>
          </div>
        </section>
        {/* 3D 렌더러가 live3d 안에 캔버스를 붙인다. 회전해도 입력 그림은 변하지 않는다. */}
        <aside className="preview-panel">
          <div className="preview-heading">
            <div>
              <span className="overline">MY CAKE</span>
              <h2>이렇게 완성돼요</h2>
            </div>
            <span className="live-tag">3D 미리보기</span>
          </div>
          <div
            id="live3d"
            className="model-view"
            role="img"
            aria-label="드래그하여 돌려볼 수 있는 케이크 3D 미리보기"
          ></div>
          <p className="rotate-note">케이크를 좌우로 밀어 돌려보세요</p>
          {/* 시점 버튼은 카메라만 바꾼다. 제출할 면은 왼쪽 탭에서 선택한다. */}
          <div className="view-controls">
            <button data-view="angle">전체</button>
            <button data-view="top">윗면</button>
            <button data-view="side">옆면</button>
          </div>
          <div className="cake-spec">
            <span>화이트 케이크</span>
            <span>지름 15cm · 높이 8cm</span>
          </div>
        </aside>
      </div>
      {/* 이 단계는 제출이 아니라 확인 창을 여는 동작이다. */}
      <footer className="bottom-bar">
        <div>
          <b>마지막으로 전체 모습을 확인해요.</b>
          <p>확인 후에도 돌아와서 수정할 수 있어요.</p>
        </div>
        <button id="review" className="primary">
          케이크 전체 확인 <span>→</span>
        </button>
      </footer>
      <p className="prototype-note">
        3D는 배치 참고용이며 실제 필기 결과와 차이가 있습니다. 주문 접수 후 제작
        상태는 운영자가 확인합니다.
      </p>
    </main>
  );
}
