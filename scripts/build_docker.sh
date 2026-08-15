#!/usr/bin/env bash
# 构建开发容器镜像
set -e
cd "$(dirname "$0")/../docker"
docker compose build "$@"
echo "构建完成。用 ./scripts/run_container.sh 启动容器"
