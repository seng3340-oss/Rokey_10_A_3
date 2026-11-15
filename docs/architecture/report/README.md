# 시스템 구조와 데이터 필드

[PDF 보기](케이크_시스템설계_통신규격_v0.2.pdf) · [HTML 보기](system-contract.html)

세로 A4 3쪽으로 간단히 정리했습니다.

1. 시스템 연결 구조와 각 부분의 역할
2. 웹 그림 입력과 ROS 주문 필드
3. 로봇 경로와 작업 상태 필드

`build_report.py`에서 본문을, `report.css`에서 인쇄 스타일을 수정합니다.
저장소 루트에서 다음 명령으로 HTML을 갱신합니다.

```bash
python3 docs/architecture/report/build_report.py
```

PDF는 Chrome에서 A4 세로·배율 100%·여백 없음·배경 그래픽 켜기·머리글/바닥글 끄기로 출력합니다.
`diagrams/`에는 이전 상세 보고서에 썼던 도식 원본을 보관했습니다. 간단한 보고서에서는 사용하지 않습니다.
상세 정의는 [INTERFACES.md](../INTERFACES.md), 확정 기록은 [DECISIONS.md](../DECISIONS.md)에 있습니다.
