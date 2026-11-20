#!/usr/bin/env bash
# 복제 위치·사용자 이름에 관계없이 프로젝트에서 생성한 도구를 선택한다.
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$project_root/.runtime"
if [[ -x "$project_root/.runtime/node/bin/node" ]]; then
  export PATH="$project_root/.runtime/node/bin:$PATH"
fi
if [[ -d "$project_root/.runtime/ros-venv" ]]; then
  ros_python_site="$("$project_root/.runtime/ros-venv/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
  export PYTHONPATH="$ros_python_site:${PYTHONPATH:-}"
elif [[ -d "$project_root/.runtime/python_deps" ]]; then
  # 이전 노트북의 설치본만 호환한다. 새 PC는 setup_project.sh가 ros-venv를 만든다.
  export PYTHONPATH="$project_root/.runtime/python_deps:${PYTHONPATH:-}"
fi