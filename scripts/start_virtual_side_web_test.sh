#!/usr/bin/env bash
# 실물 도메인30과 분리한 45도 펜 옆면 검증. 웹·DB·로그도 별도로 사용한다.
set -eo pipefail
export ROS_DOMAIN_ID=187
source "$(dirname "${BASH_SOURCE[0]}")/_system_environment.sh" virtual \
  --side-pen-45 --pen-extension-cm 4 --frontend-port 8081 --backend-port 5001 "$@"
