#!/usr/bin/env bash
# 실제 로봇 연결용이다. 영점 확인 전에는 주문 브릿지를 시작하지 않는다.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_system_environment.sh" real "$@"
