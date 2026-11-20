import type { Order, OrderDetail, OrderPage } from "./hmi";

/** 운영 화면의 일시를 서버 접수 기준과 같은 한국 시간으로 표시한다. 잘못된 값은 숨기지 않는다. */
export function dashboardTimestamp(value: string | Date): string {
  const date = value instanceof Date ? value : new Date(value);
  if (!Number.isFinite(date.getTime())) return "—";
  return new Intl.DateTimeFormat("sv-SE", {
    timeZone: "Asia/Seoul",
    year: "numeric",
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hourCycle: "h23",
  })
    .format(date)
    .replaceAll("-", ".");
}

/** 수동 선택은 자동 갱신으로 바꾸지 않는다. 자동 모드에서는 운용 중인 주문을 먼저 고른다. */
export function dashboardOrder(orders: Order[], selected = ""): string {
  if (selected) return selected;
  for (const status of [
    "running",
    "paused",
    "dispatching",
    "claimed",
    "received",
    "uncertain",
  ]) {
    const active = orders.find((order) => order.status === status);
    if (active) return active.order_id;
  }
  return orders[0]?.order_id || "";
}

/** 서버의 오늘 총계를 우선 사용한다. 구버전 서버의 잘린 페이지는 하한값임을 +로 표시한다. */
export function dashboardToday(page: OrderPage, now = new Date()): string {
  if (page.today_total !== undefined) return String(page.today_total);
  const day = dashboardTimestamp(now).slice(0, 10);
  const count = page.items.filter((order) =>
    dashboardTimestamp(order.created_at).startsWith(day),
  ).length;
  return `${count}${page.total > page.items.length && count === page.items.length ? "+" : ""}`;
}

/** TM의 실제 경로 순번이 있을 때만 윗면/옆면을 구분한다. 기록에 없는 면은 추정하지 않는다. */
export function dashboardSurface(
  order: OrderDetail | null,
): "top" | "side" | null {
  if (!order) return null;
  const paths = [order.drawing, ...(order.drawing.additional_paths || [])];
  const match = /DrawingPath\[(\d+)\/(\d+)\]/.exec(order.message);
  return match
    ? paths[Number(match[1]) - 1]?.drawing_surface || null
    : paths.length === 1
      ? paths[0].drawing_surface
      : null;
}
