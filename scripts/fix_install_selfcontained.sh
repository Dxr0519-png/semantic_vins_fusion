#!/usr/bin/env bash
# =====================================================================
# 将 install/ 下指向绝对路径(通常是 build 树 /workspace/build)的软链接
# 原地固化为真实文件，使 install 自包含、不依赖特定挂载点。
#
# 背景: ament_cmake 的 colcon build 会把 install/<pkg>/... 的 Config.cmake、
#       可执行文件、launch 等做成指向 build 树的绝对路径软链接。
#       在 compose 容器里 repo 挂在 /workspace 所以能解析;
#       但 CLion Docker 工具链把 repo 挂在 /tmp/<工程名>, /workspace 不存在,
#       软链接全部变死链 -> find_package 找不到工作区包。
#
# 用法: 在能解析 /workspace 的容器里运行(colcon build 之后重跑一次):
#       ./scripts/fix_install_selfcontained.sh
# 参数: INSTALL_DIR 默认 /workspace/install
# =====================================================================
set -euo pipefail

INSTALL_DIR="${1:-/workspace/install}"
[ -d "$INSTALL_DIR" ] || { echo "找不到 $INSTALL_DIR"; exit 1; }

count=0
skipped=0
while IFS= read -r -d '' link; do
  target="$(readlink "$link")"
  [[ "$target" == /* ]] || continue            # 只处理绝对路径软链接
  if [ -e "$target" ]; then                     # 目标存在(本容器能解析)才固化
    cp --remove-destination "$target" "$link"
    count=$((count+1))
  else
    skipped=$((skipped+1))
    echo "警告: 跳过死链 $link -> $target (目标不存在)"
  fi
done < <(find "$INSTALL_DIR" -type l -print0)

echo "已将 $count 个软链接固化为真实文件${skipped:+, 跳过 $skipped 个死链}"
