import type { DrawingPathPayload, DrawingPayload } from "./drawing.ts";

/** 기존 주문 API의 상태 값이다. 로봇의 안전 상태나 운전 모드를 뜻하지 않는다. */
export const orderStates = [
  "received",
  "paused",
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

/**
 * 한 면의 원본 그림 형식을 검사한다. 캔버스 크기·획 수·점 수와 좌표 범위를 확인해
 * 깨진 그림이 실행 경로처럼 보이지 않게 한다. 주 그림과 추가 면(additional_paths)에 공통 적용한다.
 */
function isValidPath(
  d: DrawingPathPayload | undefined,
): d is DrawingPathPayload {
  return Boolean(
    d &&
    Number.isInteger(d.canvas_width) &&
    Number.isInteger(d.canvas_height) &&
    d.canvas_width > 0 &&
    d.canvas_width <= 8192 &&
    d.canvas_height > 0 &&
    d.canvas_height <= 8192 &&
    (d.drawing_surface === "top" || d.drawing_surface === "side") &&
    Array.isArray(d.strokes) &&
    d.strokes.length <= 100 &&
    d.strokes.every(
      (s) =>
        Array.isArray(s.points) &&
        s.points.length >= 2 &&
        s.points.length <= 5000 &&
        s.points.every(
          (p) =>
            Number.isFinite(p.x) &&
            Number.isFinite(p.y) &&
            p.x >= 0 &&
            p.y >= 0 &&
            p.x <= d.canvas_width &&
            p.y <= d.canvas_height,
        ),
    ),
  );
}

/**
 * 주문 상세를 검사한다. 주 그림의 면은 주문 면과 같아야 하고,
 * 추가 면은 백엔드 계약대로 최대 1개의 옆면(side)만 허용한다(윗면+옆면 동시 주문).
 */
export function parseDetail(value: unknown): OrderDetail {
  const row = parseOrder(value) as OrderDetail;
  const d = row.drawing;
  const extra = d?.additional_paths ?? [];
  if (
    !isValidPath(d) ||
    d.drawing_surface !== row.drawing_surface ||
    !Array.isArray(extra) ||
    extra.length > 1 ||
    extra.some((path) => !isValidPath(path) || path.drawing_surface !== "side")
  ) {
    throw new Error("원본 그림 응답 형식을 확인할 수 없습니다.");
  }
  return row;
}

/**
 * 주문 그림에서 요청한 면의 원본을 찾는다. 주 그림 → 추가 면 순서로 확인하며
 * 없는 면을 만들거나 다른 주문과 섞지 않는다. 없으면 null.
 */
export function surfaceDrawing(
  drawing: DrawingPayload | null | undefined,
  surface: "top" | "side",
): DrawingPathPayload | null {
  if (!drawing) return null;
  if (drawing.drawing_surface === surface) return drawing;
  return (
    drawing.additional_paths?.find(
      (path) => path.drawing_surface === surface,
    ) ?? null
  );
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
  paused: "복구 대기",
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

/** HMI에서 관측한 시각을 운영자의 현지 시간으로 밀리초까지 표시한다. */
export function formatLogTime(date: Date): string {
  const pad = (value: number, length = 2) =>
    String(value).padStart(length, "0");
  return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())} ${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}.${pad(date.getMilliseconds(), 3)}`;
}

/** 공정 로그에 포함된 경로·동작 이름을 번역하고 오류코드는 보존한다. */
export function koreanLogMessage(message: string, errorCode = ""): string {
  const motions: Record<string, string> = {
    APPROACH: "접근",
    PEN_DOWN: "펜 내림",
    DRAWING: "그리기",
    PEN_UP: "펜 올림",
    REPOSITIONING: "다음 획으로 이동",
  };
  const words: Record<string, string> = {
    TM: "작업 관리 노드",
    Goal: "작업 요청",
    DrawOnCake: "그리기",
    HTTP: "응답 코드",
    TOP: "윗면",
    SIDE: "옆면",
  };
  return message
    .replace(/Failed to fetch/gi, "서버에 연결할 수 없습니다")
    .replace(/NetworkError/gi, "네트워크 연결 오류")
    .replace(/DrawingPath\[(\d+)\/(\d+)\]/g, "그림 경로 [$1/$2]")
    .replace(/[A-Za-z][A-Za-z0-9_-]*/g, (word) => {
      if (
        word === errorCode ||
        (/^[A-Z]+(?:_[A-Z0-9]+)+$/.test(word) && !(word in motions))
      )
        return word;
      return motions[word] || words[word] || "상세 정보";
    });
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
    const severity = ["failed", "paused", "rejected"].includes(order.status)
      ? "error"
      : order.status === "uncertain"
        ? "warning"
        : "info";
    changes.push({
      severity,
      message: `${order.order_id} · ${orderLabels[order.status]} · ${order.phase && phaseLabel(order.phase) === order.phase ? "알 수 없는 공정" : phaseLabel(order.phase)} · ${order.progress_pct.toFixed(1)}% · ${koreanLogMessage(order.message, order.error_code)}${order.error_code ? ` · 오류 코드 ${order.error_code}` : ""}`,
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

/** 최신 TM 공정 문자열만 번역한다. 알려지지 않은 값도 숨기지 않는다. */
export function phaseLabel(phase: string): string {
  const labels: Record<string, string> = {
    PREPARING: "플레이트 준비",
    DRAWING: "드로잉",
    FINISHING: "회수·출고",
    COMPLETED: "완료",
    FAILED: "실패",
    CANCELED: "취소",
  };
  return labels[phase] ?? (phase || "공정 미수신");
}

/**
 * HMI 소프트웨어 정지 상태(백엔드 /api/robot/stop 응답).
 * 물리 E-stop 신호가 아니며, 브릿지가 로봇에 적용했는지를 applied로 구분한다.
 * - seq: 정지 요청마다 1씩 증가하는 순번(해제 없음, 재시작 시 0)
 * - applied: 현재 순번을 브릿지가 로봇 쪽(move_stop·래치)에 적용했는지
 * - bridge_listening: 브릿지가 최근(약 8초 이내) 정지 요청을 수신 대기했는지
 */
export interface RobotStop {
  seq: number;
  stopped: boolean;
  requested_at: string | null;
  applied: boolean;
  applied_at: string | null;
  message: string;
  bridge_listening: boolean;
}

/** 정지 상태 응답을 검사한다. 형식이 다르면 정지 여부를 추측하지 않고 오류로 처리한다. */
export function parseRobotStop(value: unknown): RobotStop {
  const row = value as RobotStop;
  const optionalText = (v: unknown) => v === null || typeof v === "string";
  if (
    !row ||
    typeof row !== "object" ||
    !Number.isInteger(row.seq) ||
    row.seq < 0 ||
    typeof row.stopped !== "boolean" ||
    typeof row.applied !== "boolean" ||
    typeof row.bridge_listening !== "boolean" ||
    typeof row.message !== "string" ||
    !optionalText(row.requested_at) ||
    !optionalText(row.applied_at)
  ) {
    throw new Error("정지 상태 응답 형식을 확인할 수 없습니다.");
  }
  return row;
}

/**
 * HMI의 소프트웨어 정지를 요청한다. 예외 복구·주문 재개와는 별도 경로다.
 * 소프트웨어 정지 해제 API는 없으며 현장 정리 후 실행기 재시작으로 해제한다.
 * 본문 없이 POST하며 응답으로 갱신된 정지 상태를 돌려받는다.
 * 정지 요청은 즉시성이 중요하므로 3초 안에 응답이 없으면 실패로 알린다.
 */
export async function sendRobotStop(): Promise<RobotStop> {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 3000);
  try {
    const response = await fetch("/api/robot/stop", {
      method: "POST",
      signal: controller.signal,
      cache: "no-store",
    });
    if (!response.ok) {
      throw new Error(`정지 요청 실패 (HTTP ${response.status})`);
    }
    return parseRobotStop(await response.json());
  } finally {
    clearTimeout(timeout);
  }
}

/** 제작 실패 후 서버에 영속화된 접수 잠금이다. 로봇 운전·안전 상태와는 별개다. */
export interface WorkcellRecovery {
  revision: number;
  requires_reset: boolean;
  can_reset: boolean;
  can_resume: boolean;
  resume_requested: boolean;
  stopped_stage: string;
  stopped_label: string;
  order_id: string;
  error_code: string;
  message: string;
  updated_at: string;
  reset_at: string | null;
}

/** 알 수 없는 잠금 응답을 접수 가능으로 해석하지 않도록 계약을 검사한다. */
export function parseWorkcellRecovery(value: unknown): WorkcellRecovery {
  const row = value as WorkcellRecovery;
  if (
    !row ||
    typeof row !== "object" ||
    !Number.isSafeInteger(row.revision) ||
    row.revision < 0 ||
    typeof row.requires_reset !== "boolean" ||
    typeof row.can_reset !== "boolean" ||
    typeof row.can_resume !== "boolean" ||
    typeof row.resume_requested !== "boolean" ||
    typeof row.stopped_stage !== "string" ||
    typeof row.stopped_label !== "string" ||
    typeof row.order_id !== "string" ||
    typeof row.error_code !== "string" ||
    typeof row.message !== "string" ||
    typeof row.updated_at !== "string" ||
    !(row.reset_at === null || typeof row.reset_at === "string") ||
    (row.requires_reset && (!row.order_id || row.revision === 0)) ||
    (row.can_reset && !row.requires_reset) ||
    (row.can_resume && !row.can_reset)
  )
    throw new Error("재설정 상태 응답을 확인할 수 없습니다.");
  return row;
}

/** 관리자 확인 버튼에서만 호출한다. 재개는 동일 주문을 예약하고 초기 확인은 최종 실패로 종료한다. */
export async function sendWorkcellDecision(
  decision: "resume" | "initialize",
  revision: number,
  signal: AbortSignal,
): Promise<WorkcellRecovery> {
  const response = await fetch(`/api/workcell/${decision}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ revision, confirmed_reset: true }),
    signal,
    cache: "no-store",
  });
  if (!response.ok) {
    const problem = await response.json();
    throw new Error(
      typeof problem.detail === "string"
        ? problem.detail
        : "재설정 완료 요청 실패",
    );
  }
  return parseWorkcellRecovery(await response.json());
}

/** 서버에 기록한 제작 이벤트다. id는 페이지 이동용 증가 커서이며 시간은 UTC ISO다. */
export interface StoredWorkcellLog {
  id: number;
  created_at: string;
  order_id: string;
  severity: "info" | "warning" | "error";
  message: string;
}
export interface WorkcellLogPage {
  items: StoredWorkcellLog[];
  has_more: boolean;
}

/** 깨진 로그·커서 응답은 빈 이력으로 숨기지 않고 재조회 가능한 오류로 표시한다. */
export function parseWorkcellLogs(value: unknown): WorkcellLogPage {
  const page = value as WorkcellLogPage;
  if (
    !page ||
    !Array.isArray(page.items) ||
    page.items.length > 100 ||
    typeof page.has_more !== "boolean" ||
    page.items.some(
      (row) =>
        !row ||
        !Number.isSafeInteger(row.id) ||
        row.id < 1 ||
        typeof row.created_at !== "string" ||
        !Number.isFinite(Date.parse(row.created_at)) ||
        typeof row.order_id !== "string" ||
        typeof row.message !== "string" ||
        !["info", "warning", "error"].includes(row.severity),
    )
  )
    throw new Error("로그 응답을 확인할 수 없습니다.");
  return page;
}

/** 새 로그·이전 페이지가 겹쳐 도착해도 ID로 합치고 최신순을 유지한다. */
export function mergeWorkcellLogs(
  previous: StoredWorkcellLog[],
  incoming: StoredWorkcellLog[],
): StoredWorkcellLog[] {
  return [
    ...new Map([...previous, ...incoming].map((row) => [row.id, row])).values(),
  ].sort((a, b) => b.id - a.id);
}
