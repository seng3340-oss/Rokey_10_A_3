# 시스템 구조와 ROS 통신 안내서

[PDF 보기](케이크_시스템설계_통신규격_v0.3.pdf) · [HTML 보기](system-contract.html)

팀원이 처음 읽는 상황을 기준으로 세로 A4 4쪽으로 정리했습니다.

1. 전체 구조도와 노드별 역할, 현재 구현 상태
2. Action·Service·Topic의 차이와 보내는 쪽·받는 쪽 연결표
3. 웹 원본과 ROS 메시지 6종의 필드·사용 위치
4. Action 4종·Service 3종의 요청·진행·결과 필드

문서의 구조도는 목표 설계입니다. 현재 ROS 업무 노드와 백엔드 연결은 구현 전이며,
HMI는 정적 화면입니다. 미확정 주소와 정책은 확정된 것처럼 표시하지 않습니다.

`build_report.py`에서 본문을, `report.css`에서 인쇄 스타일을 수정합니다.
저장소 루트에서 다음 명령으로 HTML을 갱신합니다.

```bash
python3 docs/architecture/report/build_report.py
```

PDF는 Chrome에서 A4 세로·배율 100%·여백 없음·배경 그래픽 켜기·머리글/바닥글 끄기로 출력합니다.
본문과 바닥글이 겹치지 않는지, 4쪽으로 출력되는지 확인합니다.
`diagrams/`에는 이전 상세 보고서에 썼던 도식 원본을 보관했습니다.
현재 보고서의 구조도는 HTML/CSS로 만들며 외부 렌더러가 필요하지 않습니다.
상세 정의는 [INTERFACES.md](../INTERFACES.md), 확정 기록은 [DECISIONS.md](../DECISIONS.md)에 있습니다.
