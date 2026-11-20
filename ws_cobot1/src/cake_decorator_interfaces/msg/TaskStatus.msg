# task_manager_node가 /task_status로 발행하는 작업 상태. 작업 중 마지막 값을 1초마다 갱신한다.
string order_id
# 공정 상태 문자열. 허용 상태 집합은 작업 관리 구현과 함께 확정한다.
string phase
# 0~100 범위의 진행률. 타입 자체로 범위를 강제하지 않으므로 발행자가 검사해야 한다.
float32 progress_pct
string current_command
string message
builtin_interfaces/Time timestamp
