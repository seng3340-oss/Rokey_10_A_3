import { defineConfig, loadEnv } from "vite";
/** 개발 브라우저의 동일 출처 /api 요청을 별도 백엔드 개발 서버로 중계한다. */
export default defineConfig(({ mode }) => {
  // 브라우저 공개 변수가 아닌 개발 서버 설정이므로 접두사 없이 환경 파일을 읽는다.
  const env = loadEnv(mode, process.cwd(), "");
  return {
    server: {
      proxy: {
        // 기존 드로잉 요청용 프록시다. 정적 HMI는 API를 호출하지 않는다.
        "/api": {
          target: env.API_PROXY_TARGET || "http://127.0.0.1:5000",
          changeOrigin: true,
        },
      },
    },
  };
});
