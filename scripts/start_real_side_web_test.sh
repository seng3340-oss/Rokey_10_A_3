#!/usr/bin/env bash
# 45도 펜을 이미 잡은 상태에서, 기존 실물 드라이버와 웹 주문을 연결한다.
set -eo pipefail
# 기본은 양쪽 각각 1mm. 뒤에 --side-y-inset-mm 값을 주면 해당 값으로 덮어쓴다.
source "$(dirname "${BASH_SOURCE[0]}")/_system_environment.sh" real --side-pen-45 --reuse-driver --skip-center-confirm --side-y-inset-mm 1 "$@"
