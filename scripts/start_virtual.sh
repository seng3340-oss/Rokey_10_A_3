#!/usr/bin/env bash
# 웹·백엔드·가상 로봇을 함께 켠다. 펜 길이는 공통 실행기에서 입력받는다.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_system_environment.sh" virtual "$@"
