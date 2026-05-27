#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="$(basename "${SCRIPT_DIR}")"
OUTPUT="${1:-/tmp/${PROJECT_NAME}_source_$(date +%Y%m%d_%H%M%S).tar.gz}"

mkdir -p "$(dirname "${OUTPUT}")"

tar -czf "${OUTPUT}" \
    -C "${SCRIPT_DIR}/.." \
    --exclude="${PROJECT_NAME}/.git" \
    --exclude="${PROJECT_NAME}/.codex" \
    --exclude="${PROJECT_NAME}/.vscode" \
    --exclude="${PROJECT_NAME}/.idea" \
    --exclude="*/.git" \
    --exclude="*/.codex" \
    --exclude="*/.vscode" \
    --exclude="*/.idea" \
    --exclude="*/build" \
    --exclude="*/build/*" \
    --exclude="*/log" \
    --exclude="*/log/*" \
    --exclude="*/install" \
    --exclude="*/install/*" \
    --exclude="${PROJECT_NAME}/*/.git" \
    --exclude="${PROJECT_NAME}/*/*/.git" \
    --exclude="${PROJECT_NAME}/build" \
    --exclude="${PROJECT_NAME}/log" \
    --exclude="${PROJECT_NAME}/mj/build" \
    --exclude="${PROJECT_NAME}/mj/log" \
    --exclude="${PROJECT_NAME}/mj/install" \
    --exclude="${PROJECT_NAME}/mj/src/build" \
    --exclude="${PROJECT_NAME}/mj/src/log" \
    --exclude="${PROJECT_NAME}/mj/src/install" \
    --exclude="${PROJECT_NAME}/thirdparty/concurrentqueue/build" \
    --exclude="${PROJECT_NAME}/thirdparty/Fast-DDS-Gen/build" \
    "${PROJECT_NAME}"

echo "源码包已生成: ${OUTPUT}"
echo "已排除: .git、.codex、IDE 配置、build、log、install 等生成目录"
