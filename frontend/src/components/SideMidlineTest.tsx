import { useRef, useState, type FormEvent } from "react";
import {
  SIDE_MIDLINE_TEST,
  submitSideMidlineTest,
  type SideMidlineReceipt,
} from "../api/sideMidlineTest";

/** 독립 테스트 입력과 접수 안내다. 일반 캔버스 컨트롤러와 상태·DOM 식별자를 공유하지 않는다. */
export function SideMidlineTest() {
  const [length, setLength] = useState(
    String(SIDE_MIDLINE_TEST.default_line_length_mm),
  );
  const [submitting, setSubmitting] = useState(false);
  const [receipt, setReceipt] = useState<SideMidlineReceipt | null>(null);
  const [error, setError] = useState("");
  // React 재렌더 전 연속 이벤트도 막아 한 번의 조작이 여러 요청으로 나뉘지 않게 한다.
  const inFlight = useRef(false);

  /** 길이만 제출한다. 오류나 접수 시에도 일반 그림·확인 창·제작 요청에는 접근하지 않는다. */
  async function submit(event: FormEvent<HTMLFormElement>) {
    event.preventDefault();
    if (inFlight.current) return;
    inFlight.current = true;
    setSubmitting(true);
    setReceipt(null);
    setError("");
    try {
      // 빈 입력은 Number("")의 0 변환 대신 명시적인 유효성 오류로 전달한다.
      setReceipt(
        await submitSideMidlineTest(length.trim() ? Number(length) : NaN),
      );
    } catch (cause) {
      setError(
        cause instanceof Error
          ? cause.message
          : "테스트 요청을 확인하지 못했습니다.",
      );
    } finally {
      inFlight.current = false;
      setSubmitting(false);
    }
  }

  return (
    <section
      className="side-midline-test"
      aria-labelledby="side-midline-test-title"
    >
      {/* 이 치수는 별도 실측 테스트용이며 기존 일반 3D 미리보기 치수와 구분한다. */}
      <h2 id="side-midline-test-title">옆면 수평선 테스트</h2>
      <p>
        독립 테스트 치수: 윗지름 {SIDE_MIDLINE_TEST.top_diameter_mm / 10}cm ·
        아랫지름 {SIDE_MIDLINE_TEST.bottom_diameter_mm / 10}cm · 높이{" "}
        {SIDE_MIDLINE_TEST.height_mm / 10}cm
      </p>
      <p>
        테스트 위치: 윗면에서 {SIDE_MIDLINE_TEST.height_mm / 2}mm 아래 (옆면
        중간)
      </p>
      <p>
        YES 기준점은 케이크 윗면 중심입니다. real 모드에서는 실행 시 실제 로봇이
        움직입니다.
      </p>
      <p>
        일반 드로잉·3D와 별개인 테스트입니다. 기존 그림과 주문을 변경하거나
        취소하지 않습니다.
      </p>
      {/* 네이티브 폼 검증 대신 API 검증 오류를 동일한 alert 영역으로 전달한다. */}
      <form onSubmit={submit} noValidate aria-busy={submitting}>
        <label htmlFor="side-midline-test-length">
          선 길이 (mm, {SIDE_MIDLINE_TEST.min_line_length_mm}~
          {SIDE_MIDLINE_TEST.max_line_length_mm})
        </label>
        <input
          id="side-midline-test-length"
          type="number"
          min={SIDE_MIDLINE_TEST.min_line_length_mm}
          max={SIDE_MIDLINE_TEST.max_line_length_mm}
          step="any"
          required
          value={length}
          disabled={submitting}
          onChange={(event) => {
            setLength(event.target.value);
            setReceipt(null);
            setError("");
          }}
        />
        <button type="submit" disabled={submitting}>
          {submitting ? "접수 요청 중…" : "옆면 테스트 실행"}
        </button>
      </form>
      {/* 접수는 대기열 등록이며 로봇 동작 완료를 의미하지 않는다. */}
      {receipt && (
        <p role="status">
          테스트 대기열에 접수되었습니다. 주문 번호: {receipt.order_id} · 동작
          완료는 운영자가 확인합니다.
        </p>
      )}
      {error && (
        <p className="side-midline-test-error" role="alert">
          {error}
        </p>
      )}
    </section>
  );
}
