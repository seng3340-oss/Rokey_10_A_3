import { useEffect, useLayoutEffect, useRef, useState } from "react";
import {
  formatLogTime,
  koreanLogMessage,
  mergeWorkcellLogs,
  parseWorkcellLogs,
  readJson,
} from "../api/hmi";
import type { OrderLog, StoredWorkcellLog } from "../api/hmi";

/** 제한 높이의 로그창에서 최신 이벤트를 갱신하고 아래 스크롤로 이전 기록을 조회한다. */
export function WorkcellLogs({
  localLogs,
}: {
  localLogs: (OrderLog & { time: string })[];
}) {
  const [rows, setRows] = useState<StoredWorkcellLog[]>([]);
  const [hasOlder, setHasOlder] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");
  const viewport = useRef<HTMLDivElement>(null);
  const stored = useRef<StoredWorkcellLog[]>([]);
  const initialized = useRef(false);
  const mounted = useRef(false);
  const olderRequest = useRef<AbortController | null>(null);
  const anchor = useRef<{ top: number; height: number } | null>(null);

  // 과거를 읽는 중 새 로그가 앞에 추가되어도 보고 있던 행의 화면 위치를 보존한다.
  useLayoutEffect(() => {
    if (anchor.current && viewport.current) {
      const previous = anchor.current;
      viewport.current.scrollTop =
        previous.top + viewport.current.scrollHeight - previous.height;
      anchor.current = null;
    }
  }, [rows]);

  useEffect(() => {
    mounted.current = true;
    let timer: ReturnType<typeof setTimeout>;
    let pending: AbortController | null = null;
    /** 최초 30건 뒤에는 마지막 ID 이후만 읽어 빠른 로그 발생에도 중간 기록을 누락하지 않는다. */
    async function poll() {
      const controller = new AbortController();
      pending = controller;
      const timeout = setTimeout(() => controller.abort(), 10000);
      let backlog = false;
      try {
        const query = initialized.current
          ? `after_id=${stored.current[0]?.id || 0}&limit=100`
          : "limit=30";
        const page = parseWorkcellLogs(
          await readJson(`/api/workcell/logs?${query}`, controller.signal),
        );
        if (!mounted.current) return;
        if (!initialized.current) {
          initialized.current = true;
          setHasOlder(page.has_more);
        } else {
          backlog = page.has_more;
          if (
            page.items.length &&
            viewport.current &&
            viewport.current.scrollTop > 8
          ) {
            anchor.current = {
              top: viewport.current.scrollTop,
              height: viewport.current.scrollHeight,
            };
          }
        }
        stored.current = mergeWorkcellLogs(stored.current, page.items);
        setRows(stored.current);
        setError("");
      } catch {
        if (mounted.current)
          setError("로그를 불러오지 못했습니다. 자동으로 다시 확인합니다.");
      } finally {
        clearTimeout(timeout);
        if (mounted.current) timer = setTimeout(poll, backlog ? 0 : 3000);
      }
    }
    void poll();
    return () => {
      mounted.current = false;
      pending?.abort();
      olderRequest.current?.abort();
      clearTimeout(timer);
    };
  }, []);

  /** 가장 오래된 표시 ID 이전 페이지를 한 번만 요청한다. 반복 스크롤은 중복 요청하지 않는다. */
  async function loadOlder() {
    const oldest = stored.current.at(-1)?.id;
    if (!hasOlder || !oldest || olderRequest.current) return;
    const controller = new AbortController();
    olderRequest.current = controller;
    setLoading(true);
    const timeout = setTimeout(() => controller.abort(), 10000);
    try {
      const page = parseWorkcellLogs(
        await readJson(
          `/api/workcell/logs?before_id=${oldest}&limit=30`,
          controller.signal,
        ),
      );
      if (!mounted.current) return;
      stored.current = mergeWorkcellLogs(stored.current, page.items);
      setRows(stored.current);
      setHasOlder(page.has_more);
      setError("");
    } catch {
      if (mounted.current)
        setError("이전 로그를 불러오지 못했습니다. 다시 시도하세요.");
    } finally {
      clearTimeout(timeout);
      olderRequest.current = null;
      if (mounted.current) setLoading(false);
    }
  }

  return (
    <section className="hmi-card" id="logs">
      <h2>공정·경고·오류 로그</h2>
      <p>최신 기록부터 표시합니다. 아래로 스크롤하면 이전 기록을 불러옵니다.</p>
      <div
        ref={viewport}
        className="hmi-log-scroll"
        tabIndex={0}
        aria-label="공정 로그, 아래로 스크롤해 이전 기록 보기"
        onScroll={(event) => {
          const node = event.currentTarget;
          if (node.scrollHeight - node.scrollTop - node.clientHeight < 80)
            void loadOlder();
        }}
      >
        {localLogs.map((log, index) => (
          <div
            className={`hmi-log ${log.severity}`}
            key={`local-${log.time}-${index}`}
          >
            <time>{log.time}</time>
            <span>{log.message}</span>
          </div>
        ))}
        {rows.map((log) => (
          <div className={`hmi-log ${log.severity}`} key={log.id}>
            <time>{formatLogTime(new Date(log.created_at))}</time>
            <span>
              <b>
                {log.severity === "error"
                  ? "오류"
                  : log.severity === "warning"
                    ? "경고"
                    : "정상"}
              </b>
              {" · "}
              {log.order_id}
              {" · "}
              {koreanLogMessage(log.message)}
            </span>
          </div>
        ))}
        {!rows.length && !error && <p>저장된 공정 로그가 없습니다.</p>}
        {error && <p role="status">{error}</p>}
        {hasOlder && (
          <button
            type="button"
            disabled={loading}
            onClick={() => void loadOlder()}
          >
            {loading ? "이전 로그 불러오는 중…" : "이전 로그 더 보기"}
          </button>
        )}
        {!!rows.length && !hasOlder && <p>저장된 로그를 모두 표시했습니다.</p>}
      </div>
    </section>
  );
}
