export interface HmiState {
  robot_connected: boolean;
  stage: string;
  updated_at: string;
}
export interface OrderRow {
  order_id: string;
  drawing_surface: "top" | "side";
  status: string;
}
export interface LogRow {
  timestamp: string;
  level: string;
  message: string;
}
async function getJson(path: string, signal: AbortSignal): Promise<unknown> {
  const response = await fetch(path, { signal });
  if (!response.ok)
    throw new Error(
      "HMI API에 연결되지 않았습니다. 상태를 확인할 수 없습니다.",
    );
  return response.json();
}
// 아래 GET 응답은 백엔드와 합의할 제안 계약이며 아직 구현된 서버가 아니다.
export async function readHmi(signal: AbortSignal) {
  const [state, orders, logs] = await Promise.all([
    getJson("/api/hmi/state", signal),
    getJson("/api/hmi/orders", signal),
    getJson("/api/hmi/logs", signal),
  ]);
  if (
    !state ||
    typeof state !== "object" ||
    !("robot_connected" in state) ||
    typeof state.robot_connected !== "boolean" ||
    !("stage" in state) ||
    typeof state.stage !== "string" ||
    !("updated_at" in state) ||
    typeof state.updated_at !== "string" ||
    !Number.isFinite(Date.parse(state.updated_at)) ||
    !Array.isArray(orders) ||
    !Array.isArray(logs)
  )
    throw new Error("HMI 응답 형식을 확인할 수 없습니다.");
  if (
    !orders.every(
      (o) =>
        o &&
        typeof o.order_id === "string" &&
        ["top", "side"].includes(o.drawing_surface) &&
        typeof o.status === "string",
    ) ||
    !logs.every(
      (l) =>
        l &&
        typeof l.timestamp === "string" &&
        typeof l.level === "string" &&
        typeof l.message === "string",
    )
  )
    throw new Error("주문 또는 로그 응답 형식을 확인할 수 없습니다.");
  return {
    state: state as HmiState,
    orders: orders as OrderRow[],
    logs: logs as LogRow[],
  };
}
