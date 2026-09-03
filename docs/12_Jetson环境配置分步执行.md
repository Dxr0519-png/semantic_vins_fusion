# 12 · Jetson 环境配置 · 分步执行清单（先验证，缺了才装）

> 配套设计文档：[docs/11_Jetson原生部署.md](11_Jetson原生部署.md)
> 本文是**实操版**：每步先【验证】有没有，没有才【执行】，装完【复验】。
> 已按 **2026-08-22 本板实测状态**标注每步是否需要执行。

## 0. 使用说明

- **原则**：每步三步——① 验证（探测有没有）→ ② 执行（仅当验证未通过）→ ③ 复验（确认装好）。验证已通过就整步跳过。
- **本板**：reComputer Super J401（Jetson Orin NX 16GB）· JetPack 6.2 (R36.4.3) · Ubuntu 22.04 · aarch64 · Python 3.10
- **工作区路径**（与 docs/11 不同，注意替换）：`/home/zc/semantic_ws/semantic_vins_fusion`，下文用 `$WS` 表示
- **sudo 需要输密码**：每步用到 sudo 时会交互提示
- **耗时预估**：第 1、2 步 apt 下载 5–15 分钟；第 3 步 g2o 编译 ~10 分钟；第 4 步 librealsense 编译 ~30 分钟；第 5 步 torch wheel 下载 ~10 分钟；第 6 步 colcon 构建 10–30 分钟。可随时中断，验证通过的部分下次自动跳过。
- **断点续跑**：任意一步没做完，下次从该步开始即可（前面各步已验证会跳过）。

## 1. 本板现状总览（2026-08-22 实测）

| 步骤 | 内容 | 本板状态 |
|---|---|---|
| 第 0 步 | 前置系统检查（架构/版本/TRT） | ✅ 已就绪，仅确认 |
| 第 1 步 | apt 基础依赖 | ⚠️ 部分缺（8 个包 + 部分小工具） |
| 第 2 步 | ROS2 Humble | ❌ 全部未装 |
| 第 3 步 | g2o 源码编译 | ❌ 未装 |
| 第 4 步 | librealsense2 + realsense-ros2 | ❌ 未装 |
| 第 5 步 | Python aarch64 wheels | ⚠️ numpy/cv2/tensorrt 已就绪；torch 等未装 |
| 第 6 步 | colcon 构建 | ❌ 未构建 |
| 第 7 步 | TRT engine 板载导出 | ❌ 需先拷入 .pt 权重 |
| 第 8 步 | 输出目录 + bashrc 收尾 | ❌ 未建 |

---

## 第 0 步 · 前置系统检查（本板已全部通过，仅确认）

```bash
uname -m                                # 必须输出 aarch64
cat /etc/nv_tegra_release               # 必须含 R36（JetPack 6.x）；R35 = JetPack5 无 Humble，直接放弃
python3 --version                       # 必须 3.10
python3 -c "import tensorrt; print(tensorrt.__version__)"   # 必须 10.x
```

- 本板 2026-08-22：✅ `aarch64` · `R36 (release) REVISION 4.3` · `Python 3.10.12` · `tensorrt 10.3.0`，**整步跳过**。
- 可选：`sudo nvpmodel -m 0`（MAXN 最高性能档，散热自负）。
- ⚠️ **tensorrt 是 JetPack 系统自带，永远禁止 `pip install tensorrt`**。将来 `import tensorrt` 失败的唯一正确修法是 `sudo apt-get install python3-libnvinfer`。

---

## 第 1 步 · apt 基础依赖

**① 验证**（逐包探测，全 OK 则整步跳过）：

```bash
for p in build-essential cmake pkg-config python3-pip python3-dev python3-opencv \
         libeigen3-dev libopencv-dev libboost-all-dev libceres-dev libsuitesparse-dev \
         libcholmod3 liblapack-dev libblas-dev \
         git curl wget vim tmux htop unzip libssl-dev libusb-1.0-0-dev libgtk-3-dev \
         libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev mesa-utils; do
  dpkg -s $p >/dev/null 2>&1 && echo "OK   $p" || echo "MISS $p"
done
```

**② 执行**（有 MISS 才做；apt 自身幂等，已装的自动跳过）：

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  git curl wget vim tmux htop unzip \
  build-essential cmake pkg-config \
  python3-pip python3-dev python3-opencv \
  libeigen3-dev libopencv-dev libboost-all-dev \
  libceres-dev libsuitesparse-dev libcholmod3 liblapack-dev libblas-dev \
  libssl-dev libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
  mesa-utils
```

- 本板 2026-08-22：已有 `build-essential pkg-config python3-pip python3-dev libopencv-dev git curl wget vim unzip libgl1-mesa-dev`；**缺** `cmake python3-opencv libeigen3-dev libboost-all-dev libceres-dev libsuitesparse-dev libcholmod3 liblapack-dev libblas-dev tmux htop libssl-dev libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libglu1-mesa-dev mesa-utils` → **需要执行**。已模拟过：无冲突（升级 8 / 新装 146 / 卸载 0）。

**③ 复验**：

```bash
python3 -c "import cv2; print(cv2.__version__, cv2.__file__)"
```

> 🚩 **OpenCV 红线**：`cv2.__file__` 必须包含 `dist-packages`（apt 安装），**绝不能是 `/usr/local`**（那是 pip 的 opencv-python）。本板原装 JetPack 的 nvidia-opencv 4.8.0 也是 apt 管理，同样合规；装完 `python3-opencv` 后 cv2 变成 apt 的 4.5.4，与 cv_bridge 的 ABI 一致，正常。**任何情况下不要 `pip install opencv-python`**，否则 cv_bridge import 即崩（docs/11 §5.1 注释、FAQ）。

---

## 第 2 步 · ROS2 Humble（arm64）

**① 验证**（三个探测点依次检查，第三个存在即说明 ROS 本体已装）：

```bash
ls /usr/share/keyrings/ros-archive-keyring.gpg    # ① ROS GPG key
ls /etc/apt/sources.list.d/ros2.list              # ② apt 源
ls /opt/ros/humble/setup.bash                     # ③ ROS 本体
```

**② 执行**（本板 2026-08-22：①②③ 全无 → **需要执行**）：

```bash
# 2.1 ROS GPG key + apt 源（不存在才添加）
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt-get update

# 2.2 ROS2 本体 + 本项目运行依赖（不装 desktop-full，省空间）
sudo apt-get install -y --no-install-recommends \
  ros-humble-ros-base \
  ros-humble-rviz2 ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-image-transport-plugins \
  ros-humble-tf2 ros-humble-tf2-ros \
  ros-humble-camera-info-manager ros-humble-diagnostic-updater \
  ros-humble-dynamic-reconfigure ros-humble-message-filters \
  ros-humble-rosgraph-msgs \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  python3-colcon-common-extensions

# 2.3 bashrc 注入（已存在则自动跳过）
grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc 2>/dev/null || \
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

**③ 复验**：

```bash
source /opt/ros/humble/setup.bash && ros2 -h     # 应列出 Commands: action/topic/pkg/run/... 等
ros2 pkg list 2>/dev/null | wc -l                # 应输出几百个 ros-humble 包
```

> ⚠️ **ros2cli 没有 `--version` 参数**：`ros2 --version` 报 `unrecognized arguments: --version` 是正常现象，不代表装坏了。版本验证用 `ros2 -h` 列出命令即可（或 `ros2 doctor` 体检）。
> 注意：arm64 **没有** `ros-humble-realsense2-camera` 的 deb 包，RealSense 留给第 4 步源码编译。

---

## 第 3 步 · g2o 源码编译（Ubuntu 22.04 无 apt 包）

**① 验证**：

```bash
ls /usr/local/lib/cmake/g2o/g2oConfig.cmake     # 编译安装过的标志文件
grep "export g2o_DIR" ~/.bashrc                 # 环境变量是否已注入
```

**② 执行**（本板 2026-08-22：未装 → **需要执行**；耗时 ~10 分钟）：

```bash
git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git /tmp/g2o
cd /tmp/g2o && mkdir build && cd build
cmake .. -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install
grep -q "export g2o_DIR" ~/.bashrc 2>/dev/null || \
  echo 'export g2o_DIR=/usr/local/lib/cmake/g2o' >> ~/.bashrc
cd ~ && rm -rf /tmp/g2o
```

**③ 复验**：

```bash
ls /usr/local/lib/cmake/g2o/g2oConfig.cmake
```

> 🚩 **`-DBUILD_WITH_MARCH_NATIVE=OFF` 必须保留**，否则在 Jetson 上生成非法指令（`object_slam` 崩溃）。`object_slam` 的 `find_package(g2o REQUIRED)` 靠 `g2o_DIR` 环境变量定位，别漏 bashrc 注入。

---

## 第 4 步 · librealsense2 + realsense-ros2 源码 ★ 全流程最高风险

**① 验证**：

```bash
pkg-config --exists realsense2 && echo "SDK 已装" || echo "SDK 未装"
ls /etc/udev/rules.d/99-realsense-libusb.rules
ls -d $WS/ws_src/realsense-ros2
```

**② 执行**（本板 2026-08-22：三个全无 → **需要执行**；SDK 编译 ~30 分钟）：

```bash
# 4.1 librealsense2 SDK（RSUSB 后端，绕开 Jetson 内核 UVC 驱动缺陷）
git clone --depth 1 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense
sudo cp /tmp/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
cd /tmp/librealsense && mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=false -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install && sudo ldconfig
cd ~ && rm -rf /tmp/librealsense

# 4.2 realsense-ros2 包装层（master 分支即 ROS2）
cd $WS
git clone --depth 1 https://github.com/IntelRealSense/realsense-ros.git \
  ws_src/realsense-ros2
```

**③ 复验 —— 务必先单独验证相机，再接 VINS**（D435i 的 IMU metadata / 硬件时间戳在 RSUSB 链路已知易出问题）：

```bash
# 4.3.1 SDK 层：枚举到相机，IMU 模块在
rs-enumerate-devices

# 4.3.2 话题层：插上 D435i，另开一个终端
#   ★ rs_launch.py 默认不开 IMU/infra；项目已封装好配置
#     （vins/launch/rs_camera.launch.py，内含 IMU 融合 + 双目参数）
source /opt/ros/humble/setup.bash && source $WS/install/setup.bash
ros2 launch vins rs_camera.launch.py

# 4.3.3 IMU 频率与时间戳（再开一个终端）
source /opt/ros/humble/setup.bash && source $WS/install/setup.bash
ros2 topic hz /camera/imu         # 融合话题，板载实测 ~199Hz（文档 11 写 ~400Hz 是理论值）
ros2 topic delay /camera/imu      # 抖动可控
```

> 话题为单层前缀 `/camera/...`：rs_camera.launch.py 设 `camera_namespace: '/'` + 节点名 `camera`。
> （默认 namespace 也是 "camera"，话题会变成双层 `/camera/camera/...`，与 VINS/YOLO 配置不匹配，故显式覆盖。）
> 本板实测：accel ~101Hz、gyro ~200Hz、融合 imu ~199Hz、infra ~30Hz，数据全部正常。
> VINS 配置文件 `ws_src/vins_fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml`
> 的 imu_topic / image0_topic / image1_topic 均为 `/camera/` 前缀。

- IMU 无数据 → 检查 `rs_launch.py` 的 `enable_accel/enable_gyro`，必要时 `unite_imu_method=2`。
- 这步不过，**不要**继续接 VINS（docs/11 §5.3、FAQ 第一条）。

---

## 第 5 步 · Python aarch64 wheels

**① 验证**：

```bash
python3 -c "import torch; print('torch', torch.__version__, 'cuda:', torch.cuda.is_available())" 2>&1 | tail -1
python3 -c "import numpy; print('numpy', numpy.__version__)"                 # 必须 1.x（<2）
python3 -c "import cv2; print('cv2', cv2.__version__, cv2.__file__)"         # dist-packages，非 /usr/local
python3 -c "import tensorrt; print('tensorrt', tensorrt.__version__)"        # 系统自带，只验证
python3 -m pip show ultralytics onnxruntime-gpu 2>/dev/null | grep -E "^(Name|Version)"
```

- 本板 2026-08-22：`numpy 1.21.5` ✅（已满足 <2）· `cv2 4.8.0`（dist-packages，apt）✅ · `tensorrt 10.3.0` ✅；**torch / torchvision / ultralytics / onnx / onnxruntime-gpu 未装** → **需要执行**。

**② 执行**（PyPI 没有 arm64 CUDA torch，用 NVIDIA 官方 JetPack 6 索引）：

```bash
# 5.1 pip 本体 + 覆盖 apt 的 distutils 老包（--ignore-installed 强制覆盖到 /usr/local）
sudo -H python3 -m pip install --upgrade pip setuptools wheel
sudo -H python3 -m pip install --ignore-installed sympy mpmath "numpy<2"

# 5.2 torch / torchvision：jetson-ai-lab jp6/cu126 索引（CUDA 12.6 编译，匹配本板）
#   ⚠️ 勿用 NVIDIA jp62 官方索引的"最新" wheel：2026 年起其最新版已改为 cu13x 编译，
#      装上后 torch.version.cuda=13.0，与板载 CUDA 12.6 驱动不匹配，
#      报 "CUDA initialization: The NVIDIA driver ... too old (found version 12060)"。
#   （12060 = CUDA 12.6 驱动；cu13x 的 wheel 要求 ≥ 13.0）
sudo -H python3 -m pip uninstall -y torch torchvision   # 若之前误装 cu13x 版，先卸
sudo -H python3 -m pip install torch torchvision \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
  --extra-index-url https://pypi.org/simple
#   （2026-08 实测该索引版本：torch 2.11.0 + torchvision 0.26.0）

# 5.3 ultralytics + onnx
sudo -H python3 -m pip install ultralytics onnx

# 5.4 卸载 pip 的 opencv，确保 apt 版生效（本板没有，|| true 防报错）
sudo -H python3 -m pip uninstall -y opencv-python opencv-contrib-python || true

# 5.5 onnxruntime-gpu：PyPI 无 aarch64，用 Jetson 专用索引（CUDA 12.6）
sudo -H python3 -m pip install onnxruntime-gpu \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 --extra-index-url https://pypi.org/simple

# 5.6 兜底锁 numpy<2（torch/ort 可能拉回 numpy 2.x）
sudo -H python3 -m pip install --ignore-installed "numpy<2"
```

**③ 复验**（全过才算成功）：

```bash
python3 -c "import torch; print('torch', torch.__version__, '| cuda build:', torch.version.cuda); assert torch.cuda.is_available(); print('cuda OK')"   # cuda build 必须 12.6，且输出 cuda OK
python3 -c "import numpy; assert numpy.__version__<'2'; print('numpy', numpy.__version__)"
python3 -c "import cv2; print('cv2', cv2.__version__)"                            # 4.5.4
python3 -c "import tensorrt; print('tensorrt', tensorrt.__version__)"             # 10.3.0
python3 -c "import onnxruntime; print('ort', onnxruntime.__version__)"
python3 -c "import ultralytics; print('ultralytics', ultralytics.__version__)"
```

> - `torch.cuda.is_available()` 为 False = 装错成 CPU/x86 版 → 重装 5.2 的 NVIDIA aarch64 wheel。
> - tensorrt **禁止 pip**，本板系统自带，仅验证。

---

## 第 6 步 · colcon 构建

**① 验证**：

```bash
ls $WS/install/setup.bash
```

**② 执行**（本板 2026-08-22：未构建 → **需要执行**；10–30 分钟）：

```bash
cd $WS
source /opt/ros/humble/setup.bash
export g2o_DIR=/usr/local/lib/cmake/g2o
colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release
grep -q "source $WS/install/setup.bash" ~/.bashrc 2>/dev/null || \
  echo "source $WS/install/setup.bash 2>/dev/null" >> ~/.bashrc
```

**③ 复验**：

```bash
source $WS/install/setup.bash
ros2 pkg list 2>/dev/null | grep -E "vins|yolo_seg|object_slam"   # 应列出项目包
```

> - **`--parallel-workers 4` 必须保留**：Orin NX 16GB 并行 make 易 OOM（docs/11 §6、§8）。
> - 若 `global_fusion` 报错（遗留依赖）：`--packages-select` 跳过，非主流程（docs/11 §6、FAQ）。

---

## 第 7 步 · TRT engine 板载导出

**① 验证**：

```bash
ls $WS/ws_src/yolo_seg_ros/*.pt       # 权重（.pt 被 .gitignore 排除，不入库，需从 PC 拷）
ls $WS/ws_src/yolo_seg_ros/*.engine   # engine 是否已导出
```

**② 执行**（本板 2026-08-22：两者都无 → **先把 `.pt` 从 PC 拷进来**，然后导出）：

```bash
# 7.1 从 PC 拷贝权重到板子（PC 上执行；USB 或 scp 均可）：
#   scp yolov8n-seg.pt zc@<板子IP>:/home/zc/semantic_ws/semantic_vins_fusion/ws_src/yolo_seg_ros/

# 7.2 板载导出（.engine 与 GPU 架构 + TRT 版本绑定，必须在板子上就地导出）
cd $WS
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run yolo_seg_ros export_trt --weights yolov8n-seg.pt --imgsz 640 --half

# 7.3 改配置：把 ws_src/yolo_seg_ros/config/yolo.yaml 的 model_path 改为 yolov8n-seg.engine
```

**③ 复验**：

```bash
ls $WS/ws_src/yolo_seg_ros/yolov8n-seg.engine    # engine 文件生成
```

---

## 第 8 步 · 输出目录 + bashrc 收尾

**① 验证**：

```bash
ls -d /tmp/vins_output /tmp/vins_output/pose_graph
```

**② 执行**（`mkdir -p` 本身幂等，已存在则无操作；本板 2026-08-22：未建 → 执行）：

```bash
mkdir -p /tmp/vins_output /tmp/vins_output/pose_graph
```

**③ 复验**（新开终端后应无报错、环境已就绪）：

```bash
bash -c "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; ros2 -h >/dev/null && echo 'ROS2 环境 OK'"
grep -E "source /opt/ros/humble|source $WS/install" ~/.bashrc
```

---

## 第 9 步 · 端到端启动验证（全流程装完才做）

> 详细的分层启动 + 逐级验证清单（含 PC 远程 rviz）见 [docs/13_分阶段启动与逐级验证.md](13_分阶段启动与逐级验证.md)。

插上 D435i，按序在四个终端启动（**从 workspace 根运行**，calib 相对配置目录解析）：

```bash
cd $WS && source /opt/ros/humble/setup.bash && source install/setup.bash
# 终端 1：相机（项目封装 launch：IMU 融合 ~199Hz + 双目 640x480）
ros2 launch vins rs_camera.launch.py
# 终端 2：VINS（VINS 不自动建目录，第 8 步已建）
ros2 run vins vins_node ws_src/vins_fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
# 终端 3：YOLO 分割（yolo.yaml 的 model_path 已指向 .engine）
ros2 launch yolo_seg_ros yolo_seg.launch.py
# 终端 4：物体级 SLAM
ros2 launch object_slam object_slam.launch.py
# 可视化（可选，板载 headless 时放 PC 远程连）
rviz2 -d ws_src/vins_fusion/config/vins_rviz_d435i.rviz
```

验证通过 → 完整运行流程与验收指标见 [docs/10_运行与验证.md](10_运行与验证.md)。

---

## 附录 A · 备用路线：一键脚本

不想手工逐条时，脚本内置了与本文相同的探测逻辑，重复执行自动跳过已完成步骤：

```bash
cd $WS
./scripts/setup_jetson.sh              # 全量（等价于本文第 1–8 步 + TRT 导出）
```

| 开关 | 作用 |
|---|---|
| `--skip-ros` / `--skip-g2o` / `--skip-realsense` / `--skip-python` / `--skip-colcon` / `--skip-trt` | 跳过对应分节 |
| `--force` | 强制重跑对应分节（默认幂等跳过） |

注意：脚本 §9 只在 `ws_src/yolo_seg_ros/yolov8n-seg.pt` 存在时才导出 TRT（第 7 步需先拷入权重）。

## 附录 B · 常见问题速查（详见 docs/11 §9）

| 现象 | 处理 |
|---|---|
| `/camera/imu` 无数据 | 检查 `enable_accel/enable_gyro`，必要时 `unite_imu_method=2`（第 4 步） |
| `import tensorrt` 失败 | `sudo apt-get install python3-libnvinfer`（禁止 pip） |
| `torch.cuda.is_available()` False | 装错 CPU/x86 版，重装第 5 步 5.2 的 NVIDIA aarch64 wheel |
| cv_bridge import 崩 | pip 的 opencv-python 覆盖了 apt 版：`pip uninstall opencv-python opencv-contrib-python` |
| `global_fusion` configure 报 rclpy 错 | `--packages-select` 跳过非主流程包（第 6 步） |
| `vins_node` 写不了 `/tmp/vins_output/vio.csv` | 忘建目录：`mkdir -p /tmp/vins_output`（第 8 步） |
| 离线无网 | 先装系统部分（第 0–4 步），手工拷贝 wheel 安装（docs/11 §5.4） |
