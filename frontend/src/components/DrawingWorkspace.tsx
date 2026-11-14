export function DrawingWorkspace() {
  return (
    <main>
      <div className="intro">
        <div>
          <span className="overline">내가 그리면, 로봇이 완성하는 케이크</span>
          <h1>어떤 그림을 올려볼까요?</h1>
        </div>
      </div>
      <div className="workbench">
        <section className="drawing-panel">
          <div className="drawing-toolbar">
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
            <div className="canvas-wrap">
              <canvas
                id="draw"
                width="1024"
                height="768"
                aria-label="윗면에 그림 그리기"
              ></canvas>
              <div id="empty-hint">
                여기에 그려주세요<span>손가락 또는 펜으로 그릴 수 있어요</span>
              </div>
            </div>
            <p id="surface-help">
              점선 안쪽에 그려주세요. 케이크 윗면에 배치됩니다.
            </p>
          </div>
          <div className="drawing-bottom">
            <p id="hint" role="status"></p>
            <div>
              <button id="undo">한 획 취소</button>
              <button id="clear">이 면 지우기</button>
            </div>
          </div>
        </section>
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
