#!/usr/bin/env bash
# 仅获取底盘源码，不启用硬件。已有目录一律不覆盖。
set -eo pipefail
SCOUT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCOUT_ROOT/src"
fetch() {
  local url="$1" branch="$2" dest="$3"
  if [[ -e "$dest" ]]; then
    echo "保留已有目录: $dest"
    test -d "$dest/.git" || test -f "$dest/.upstream-revision"
    return
  fi
  local stage="${dest}.download"
  if [[ -e "$stage" ]]; then
    echo "发现未完成下载: $stage；请检查后重命名或移走，再重试。" >&2
    return 1
  fi
  GIT_TERMINAL_PROMPT=0 timeout 180 git clone --depth 1 --branch "$branch" \
    "$url" "$stage"
  mv "$stage" "$dest"
}
fetch https://github.com/agilexrobotics/scout_ros2.git humble scout_ros2
fetch https://github.com/agilexrobotics/ugv_sdk.git main ugv_sdk
if [[ -d ugv_sdk/.git ]]; then
  git -C ugv_sdk submodule update --init --recursive
fi
revision() {
  if [[ -d "$1/.git" ]]; then git -C "$1" rev-parse HEAD
  else cat "$1/.upstream-revision"; fi
}
{
  printf 'scout_ros2: '; revision scout_ros2
  printf 'ugv_sdk: '; revision ugv_sdk
} > "$SCOUT_ROOT/BASE_SOURCE_REVISIONS.txt"
