import type { DrawingPayload } from "./drawing.ts";

/** 기존 주문 API의 상태 값이다. 로봇의 안전 상태나 운전 모드를 뜻하지 않는다. */
export const orderStates = [
  "received",
  "claimed",
  "dispatching",
  "running",
  "completed",
  "failed",
  "rejected",
  "uncertain",
  "resolved",
] as const;
export type OrderState = (typeof orderStates)[number];
/** progress_pct는 TM이 보낸 공정 값이며 TCP 궤적의 측정 비율이 아니다. */
export interface Order {
  order_id: string;
  status: OrderState;
  drawing_surface: "top" | "side";
  created_at: string;
  phase: string;
  progress_pct: number;
  message: string;
  /** 이전 서버에서는 생략되며 새 서버는 저장된 마지막 오류 코드만 반환한다. */
  error_code?: string;
}
/** 목록과 상세는 별도로 조회한다. 최신 100건만 메모리에 유지한다. */
export interface OrderPage {
  items: Order[];
  total: number;
}
export interface OrderDetail extends Order {
  drawing: DrawingPayload;
}
/** 서버가 저장한 마지막 이벤트 메타데이터다. 실시간 heartbeat 계약은 아니다. */
export interface Monitor {
  updated_at: string;
  error_code: string;
  elapsed_time_sec: number;
  execution_mode: "unknown" | "real" | "simulation";
  current_command?: string;
  safety_status: "unknown";
  trajectory_available: false;
}

/** 외부 JSON은 화면에 넣기 전에 검사해 NaN·범위 밖 진행률을 차단한다. */
export function parseOrder(value: unknown): Order {
  const row = value as Order;
  if (
    !row ||
    typeof row !== "object" ||
    typeof row.order_id !== "string" ||
    !row.order_id ||
    !orderStates.includes(row.status) ||
    !["top", "side"].includes(row.drawing_surface) ||
    typeof row.created_at !== "string" ||
    typeof row.phase !== "string" ||
    typeof row.message !== "string" ||
    (row.error_code !== undefined && typeof row.error_code !== "string") ||
    !Number.isFinite(row.progress_pct) ||
    row.progress_pct < 0 ||
    row.progress_pct > 100
  ) {
    throw new Error("주문 응답 형식을 확인할 수 없습니다.");
  }
  return row;
}

/** 실제 원본 크기·유한 좌표를 검사한다. SVG는 원본 픽셀 좌표계를 그대로 쓴다. */
export function parseDetail(value: unknown): OrderDetail {
  const row = parseOrder(value) as OrderDetail;
  const d = row.drawing;
  if (
    !d ||
    !Number.isInteger(d.canvas_width) ||
    !Number.isInteger(d.canvas_height) ||
    d.canvas_width <= 0 ||
    d.canvas_width > 8192 ||
    d.canvas_height <= 0 ||
    d.canvas_height > 8192 ||
    d.drawing_surface !== row.drawing_surface ||
    !Array.isArray(d.strokes) ||
    d.strokes.length > 100 ||
    d.strokes.some(
      (s) =>
        !Array.isArray(s.points) ||
        s.points.length < 2 ||
        s.points.length > 5000 ||
        s.points.some(
          (p) =>
            !Number.isFinite(p.x) ||
            !Number.isFinite(p.y) ||
            p.x < 0 ||
            p.y < 0 ||
            p.x > d.canvas_width ||
            p.y > d.canvas_height,
        ),
    )
  ) {
    throw new Error("원본 그림 응답 형식을 확인할 수 없습니다.");
  }
  return row;
}

/** GET만 허용하는 HMI 전용 조회다. 인증 토큰이나 내부 제어 주소를 사용하지 않는다. */
export async function readJson(
  path: string,
  signal: AbortSignal,
): Promise<unknown> {
  const response = await fetch(path, {
    method: "GET",
    signal,
    cache: "no-store",
  });
  if (!response.ok) throw new Error(`조회 실패 (HTTP ${response.status})`);
  return response.json();
}

/** 페이지 크기·총 건수도 검사하여 잘못된 목록을 정상 빈 목록으로 표시하지 않는다. */
export function parsePage(value: unknown): OrderPage {
  const page = value as OrderPage;
  if (
    !page ||
    !Array.isArray(page.items) ||
    page.items.length > 100 ||
    !Number.isInteger(page.total) ||
    page.total < page.items.length
  ) {
    throw new Error("주문 목록 응답 형식을 확인할 수 없습니다.");
  }
  return { items: page.items.map(parseOrder), total: page.total };
}

/** 기존 API 상태를 운영자에게 보여주는 명칭이다. 로봇 안전 판정과는 분리한다. */
export const orderLabels: Record<OrderState, string> = {
  received: "접수",
  claimed: "브릿지 인수",
  dispatching: "전송 확인 중",
  running: "공정 진행",
  completed: "완료",
  failed: "실패",
  rejected: "거절",
  uncertain: "수동 확인 필요",
  resolved: "수동 확인 종료",
};

/** 화면에서 관측한 주문 상태 변화 한 건이다. 서버의 전체 이벤트 이력은 아니다. */
export interface OrderLog {
  message: string;
  severity: "info" | "warning" | "error";
}

/** 선택한 주문과 무관하게 목록 전체의 변화를 수집해 다른 주문의 오류도 놓치지 않는다. */
export function collectOrderChanges(
  previous: Map<string, string>,
  orders: Order[],
) {
  const signatures = new Map<string, string>();
  const changes: OrderLog[] = [];
  for (const order of orders) {
    // 진행률만 바뀌어도 로그에 남기고, 같은 응답의 반복 조회는 중복 기록하지 않는다.
    const signature = JSON.stringify([
      order.status,
      order.phase,
      order.progress_pct,
      order.message,
      order.error_code || "",
    ]);
    signatures.set(order.order_id, signature);
    if (previous.get(order.order_id) === signature) continue;
    const severity = ["failed", "rejected"].includes(order.status)
      ? "error"
      : order.status === "uncertain"
        ? "warning"
        : "info";
    changes.push({
      severity,
      message: `${order.order_id} · ${orderLabels[order.status]} · ${order.phase || "공정 미수신"} · ${order.progress_pct.toFixed(1)}% · ${order.message}${order.error_code ? ` · 오류 코드 ${order.error_code}` : ""}`,
    });
  }
  return { signatures, changes };
}

/** 운영자가 선택한 운전 설정이다. 물리 장비 연결 신호로 해석하지 않는다. */
export type ExecutionMode = "unknown" | "real" | "simulation";
export interface RuntimeMonitor {
  observed: boolean;
  bridge_fresh: boolean;
  task_server_ready: boolean;
  task_feedback_fresh: boolean;
  execution_mode: ExecutionMode;
  active_order_id: string | null;
  feedback_age_sec: number | null;
  updated_at: string | null;
  robot_connected: null;
  safety_status: "unknown";
}
/** TM이 실제 DP 응답을 투영한 계획이다. 센서·TCP 실측을 포함하지 않는다. */
export interface TaskPlan extends DrawingPayload {
  version: 1;
  source: "task_manager_node/GenerateTrajectory";
  order_id: string;
  waypoint_count: number;
  execution_mode: ExecutionMode;
  updated_at: string;
  measurement_available: false;
}
const modes = ["unknown", "real", "simulation"];

/** 연결 관측은 서버가 판정한 최신성까지 검사한다. 장비 안전 여부는 추정하지 않는다. */
export function parseRuntime(value: unknown): RuntimeMonitor {
  const data = value as RuntimeMonitor;
  if (
    !data ||
    ![
      data.observed,
      data.bridge_fresh,
      data.task_server_ready,
      data.task_feedback_fresh,
    ].every((v) => typeof v === "boolean") ||
    !modes.includes(data.execution_mode) ||
    data.robot_connected !== null ||
    data.safety_status !== "unknown" ||
    !(
      data.active_order_id === null || typeof data.active_order_id === "string"
    ) ||
    !(data.updated_at === null || typeof data.updated_at === "string") ||
    !(
      data.feedback_age_sec === null ||
      (Number.isFinite(data.feedback_age_sec) && data.feedback_age_sec >= 0)
    )
  ) {
    throw new Error("TM 연결 관측 응답을 확인할 수 없습니다.");
  }
  return data;
}

/** 주문·면·해상도·생산자와 점 수를 대조한다. 원본으로 계획을 대신 만들지 않는다. */
export function parsePlan(value: unknown, order: OrderDetail): TaskPlan {
  const data = value as TaskPlan;
  if (
    !data ||
    data.version !== 1 ||
    data.source !== "task_manager_node/GenerateTrajectory" ||
    data.order_id !== order.order_id ||
    data.drawing_surface !== order.drawing_surface ||
    data.canvas_width !== order.drawing.canvas_width ||
    data.canvas_height !== order.drawing.canvas_height ||
    !modes.includes(data.execution_mode) ||
    data.measurement_available !== false ||
    !Number.isInteger(data.waypoint_count) ||
    data.waypoint_count < 2 ||
    data.waypoint_count > 30000 ||
    typeof data.updated_at !== "string"
  )
    throw new Error("TM 계획의 주문·면·출처를 확인할 수 없습니다.");
  parseDetail({ ...order, drawing: data });
  if (
    data.strokes.length === 0 ||
    data.strokes.some((stroke, index) => stroke.stroke_id !== index) ||
    data.strokes.reduce((n, stroke) => n + stroke.points.length, 0) > 20000
  ) {
    throw new Error("TM 계획의 획·점 수를 확인할 수 없습니다.");
  }
  return data;
}

/** 다른 주문의 최신 피드백을 선택 주문의 연결 상태로 잘못 표시하지 않게 한다. */
export function isOrderFeedbackFresh(
  runtime: RuntimeMonitor | null,
  orderId: string | undefined,
): boolean {
  return Boolean(
    orderId &&
    runtime?.active_order_id === orderId &&
    runtime.bridge_fresh &&
    runtime.task_feedback_fresh,
  );
}
