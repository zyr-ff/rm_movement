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
