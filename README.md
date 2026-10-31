# 第三周 · EKF 装甲板整车位姿估计

在**第二周**（传统 + 神经网络装甲板检测 + PnP 单目位姿解算）的基础上，叠加 **EKF（扩展卡尔曼滤波）**，对装甲板小车的**整车运动状态**做实时估计：平面位置 `(cx,cy)`、平面速度 `(vx,vy)`、板面高度 `za`、高度速度 `vz`、车体朝向 `θ`、转向角速度 `ω`、旋转半径 `r`，共 **9 维状态**。同时利用 **IMU 内录四元数**做帧间姿态时间同步，并对外提供带右上角实时状态 HUD + 四块板重投影的可视化视频。

> **本周（第三周）贡献范围**：本工程真正新增的内容是 `ekf/` 目录下的 EKF 估计器（状态定义、IMU 时间同步、观测模型与雅可比、预测/更新/匹配/门控），以及 `main_ekf.cpp` 的总装与可视化。`traditional/`、`neural/`、`models/` 中的装甲板检测与数字分类为**前两周已有成果**，仅在本工程中被复用以保证可独立构建运行，**不属于本周新增**，请勿当作第三周工作量。

## 目录结构

```
third_train/
├── 视觉第三周作业.md              # 作业要求原文
├── 第三周_EKF报告.md              # 本报告
├── 素材/                         # 输入数据（大视频已被 .gitignore 忽略，不进仓库）
│   ├── 2026-05-20_23-57-31 (1).avi   # 含装甲板的录制视频
│   └── 2026-05-20_23-57-31.txt       # IMU 内录数据（25 Hz 四元数）
├── code/                         # 源码
│   ├── CMakeLists.txt            # 构建脚本（MSYS2 UCRT64 + Ninja）
│   ├── main_ekf.cpp              # 第三周主程序：EKF 总装 + 视频绘制 + CSV 导出
│   ├── common/                   # 公共模块：相机参数与 PnP 位姿求解
│   │   ├── include/armor_types.h #   装甲板尺寸/类型定义
│   │   ├── include/camera_config.h   # 相机内参
│   │   └── src/pnp_solver.cpp    #   PnP 求解（SOLVEPNP_IPPE）
│   ├── ekf/                      # ★第三周 EKF 核心
│   │   ├── linalg.h              #   迷你矩阵库（乘/转置/求逆/单位阵）
│   │   ├── state.h               #   9 维状态定义与板方位常量
│   │   ├── imu_sync.{h,cpp}      #   IMU 四元数→R→Rt 与帧间时间同步
│   │   ├── observer_model.{h,cpp}#   观测模型 h(x) 与雅可比 H(3×9)
│   │   ├── ekf.{h,cpp}           #   EKF 引擎（predict/update/match）
│   │   └── test_ekf.cpp          #   纯数学单元测试（无相机）
│   ├── traditional/              # 复用·前两周上游输入（非本周新增）：传统装甲板检测 + 数字分类
│   │   ├── include/armor_detector.h
│   │   ├── include/number_classifier.h
│   │   └── src/{armor_detector,number_classifier}.cpp
│   ├── neural/                   # 复用·前两周上游输入（非本周新增）：神经网络检测（USE_NN 定义时启用）
│   │   ├── include/neural_detector.h
│   │   └── src/neural_detector.cpp
│   ├── models/                   # 推理模型
│   │   ├── Infantry-v8n-fp32.onnx    # 装甲板检测（NN）
│   │   ├── Infantry-v8n-fp16-*.onnx  # 同上 fp16 版（可选）
│   │   ├── mlp.onnx                  # 数字分类
│   │   └── label.txt
│   └── tools/
│       ├── plot_states.py        # 由 CSV 生成状态曲线图
│       └── gen_example_csv.py    # 示例 CSV 生成
└── results/                      # 运行产物（视频已被 .gitignore 忽略）
    ├── ekf_output.avi / .mp4     # 带右上角 HUD + 四板重投影的输出视频
    └── ekf_state_curves.png      # 所有状态量实时估计曲线图
```

## 环境依赖

| 依赖 | 版本 / 说明 |
|------|------------|
| C++ 编译器 | 支持 C++17（如 MSYS2 UCRT64 的 GCC 13+） |
| 构建工具 | CMake ≥ 3.16、Ninja（或 Make） |
| OpenCV | 通过 `find_package(OpenCV REQUIRED)` 自动查找 |
| ONNX Runtime | **必需**（NN 检测）。CMake 通过 `-DONNXRUNTIME_DIR=<根目录>` 或环境变量 `ONNXRUNTIME_DIR` 定位（`include/` 与 `lib/` 在其下） |

> MSYS2 用户推荐：`pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-opencv`，ONNX Runtime 用官方预编译包并设 `ONNXRUNTIME_DIR`。

## 构建

```bash
cd code
cmake -B build -G Ninja -DONNXRUNTIME_DIR=/path/to/onnxruntime   # 改成你的 ORT 根目录
cmake --build build
```

- 若想顺便构建 EKF 纯数学自测（不依赖相机），加 `-DRM_BUILD_TESTS=ON`，产物为 `build/test_ekf`。
- 找不到 ONNX Runtime 时，CMake 会直接报错并提示如何指定路径，按提示设置即可。

## 运行

```bash
# 方式一：传参（推荐，避免硬编码路径问题）
./build/rm_ekf_third <输入视频> <输出视频> <IMU.txt> <状态CSV>

# 例如
./build/rm_ekf_third ../素材/2026-05-20_23-57-31\ \(1\).avi \
                     ../results/ekf_output.mp4 \
                     ../素材/2026-05-20_23-57-31.txt \
                     ../results/ekf_states.csv

# 方式二：不带参数，则使用 main_ekf.cpp 内硬编码的默认素材路径（仅本机可用）
./build/rm_ekf_third
```

- 程序会逐帧检测装甲板 → PnP 解算 → IMU 时间同步 → EKF 预测/更新（含新息门控）→ 绘制四块板重投影 + 右上角实时状态 HUD。
- 结束后在 `results/` 输出：带 HUD 的视频、所有状态量实时估计曲线图（`ekf_state_curves.png`，由 `tools/plot_states.py` 从 CSV 生成）。

## 输出说明

- **视频**：右上角 HUD 实时显示 9 维状态估计值；画面中四块装甲板以重投影框标出。
- **曲线图**：各状态量随帧变化的估计曲线，并标注每帧观测是否被 EKF 接受（门控）。
- **CSV**：每帧的完整状态向量与新息接受标记，供离线分析。

## 备注

- 装甲板检测（`#define USE_NN` 默认开启神经网络检测器，注释该行可切回传统检测器）为前两周复用模块，CMake 已同时编译两套检测器源码；本周核心为 `ekf/` 估计器，不是检测器本身。
- 大视频文件（`*.avi`/`*.mp4`）与 `code/build/` 已被 `.gitignore` 忽略，不会纳入版本库。
