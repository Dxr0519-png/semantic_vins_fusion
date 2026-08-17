#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
cd /workspace
echo "==> colcon build (带 compile_commands 导出)"
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
echo "==> 合并各包 compile_commands.json -> /workspace/compile_commands.json"
python3 - <<'PYEOF'
import json, glob
cc = []
for f in glob.glob('/workspace/build/*/compile_commands.json'):
    cc += json.load(open(f))
json.dump(cc, open('/workspace/compile_commands.json', 'w'))
print(f"merged {len(cc)} entries")
PYEOF
