#!/usr/bin/env bash
# 启动并进入开发容器
set -e
cd "$(dirname "$0")/../docker"

# 允许容器访问 X（rviz）
xhost +local:root >/dev/null 2>&1 || true

docker compose up -d
docker exec -it semantic_vins_fusion bash
