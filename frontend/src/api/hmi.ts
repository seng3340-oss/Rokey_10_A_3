/** HMI는 조회만 수행한다. 이 타입은 모터 명령이나 안전 신호를 제공하지 않는다. */
export type Order = {
  order_id: string;
  drawing_surface: string;
  state: string;
  phase: string;
  progress_pct: number;
  message: string;
  error_code: string;
  elapsed_time_sec: number;
  created_at: string;
  updated_at: string;
  process_mode: string;
};
export type ProcessEvent = {
  id: number;
  order_id: string;
  kind: string;
  phase: string;
  stage_key: string;
  stage_state: string;
  progress_pct: number;
  message: string;
  error_code: string;
  created_at: string;
};
export type Snapshot = {
  orders: Order[];
  selected: Order | null;
  events: ProcessEvent[];
  server_time: string;
  runtime: {
    last_error_message?: string;
    last_error_source?: string;
    last_error_at?: string;
    fresh: boolean;
    task_server_ready: boolean;
    updated_at: string | null;
    age_sec: number | null;
    execution_mode: string;
    process_mode: string;
    active_order_id: string | null;
    safety_status: string;
    robot_connected: boolean | null;
  };
};

/** 순서는 실행 코드의 실제 단계 계약과 같다. 기록이 없는 단계를 완료로 추정하지 않는다. */
export const stages = [
  ["opener_pick", "오프너 집기"],
  ["cake_to_station", "케이크 틀 스테이션 배치"],
  ["opener_return_before", "오프너 반납"],
  ["pen_pick", "펜 집기"],
  ["drawing", "윗면 드로잉"],
  ["pen_return", "펜 반납"],
  ["opener_repick", "오프너 다시 집기"],
  ["cake_to_outlet", "케이크 틀 반출구 배치"],
  ["opener_return_after", "오프너 최종 반납"],
  ["safe_position", "마지막 안전 위치"],
] as const;
export const statusLabels: Record<string, string> = {
  received: "접수",
  claimed: "인수",
  dispatching: "전달 중",
  running: "실행 중",
  completed: "완료",
  failed: "실패",
  rejected: "거절",
  uncertain: "확인 필요",
  resolved: "수동 확인 종료",
};

/** 이벤트가 최신순으로 내려오더라도 명시된 ID로 마지막 단계 응답을 찾는다. */
export function latestStage(
  events: ProcessEvent[],
  key: string,
): ProcessEvent | undefined {
  return events
    .filter((e) => e.stage_key === key)
    .reduce<ProcessEvent | undefined>(
      (last, e) => (!last || e.id > last.id ? e : last),
      undefined,
    );
}

/** 마지막 성공 뒤 5초가 지나거나 조회에 실패하면 이전 화면을 최신 상태로 표시하지 않는다. */
export function isStale(
  lastSuccess: number,
  now: number,
  error: string,
): boolean {
  return Boolean(error) || !lastSuccess || now - lastSuccess > 5000;
}

export async function fetchHmi(
  orderId: string,
  signal: AbortSignal,
): Promise<Snapshot> {
  const response = await fetch(
    "/api/hmi" + (orderId ? "?order_id=" + encodeURIComponent(orderId) : ""),
    { signal, cache: "no-store" },
  );
  if (!response.ok) throw new Error(`조회 실패 HTTP ${response.status}`);
  const data = await response.json();
  if (
    !Array.isArray(data.orders) ||
    !Array.isArray(data.events) ||
    !data.runtime
  )
    throw new Error("잘못된 HMI 응답");
  return data as Snapshot;
}
