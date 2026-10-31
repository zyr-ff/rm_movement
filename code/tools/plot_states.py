#!/usr/bin/env python
# ============================================================================
#  plot_states.py —— 把 EKF 输出的 ekf_states.csv 画成“状态量实时曲线图”
#  用法:  python tools/plot_states.py [ekf_states.csv] [输出png]
#  依赖:  pip install matplotlib numpy
# ============================================================================
import sys
import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")            # 无显示环境也能保存
import matplotlib.pyplot as plt

CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else "ekf_states.csv"
PNG_PATH = sys.argv[2] if len(sys.argv) > 2 else "ekf_state_curves.png"

# 列名 -> 标题/单位/是否角度(弧度->度)
COLS = [
    ("cx",   "c_x (m)",       False),
    ("cy",   "c_y (m)",       False),
    ("vx",   "v_x (m/s)",     False),
    ("vy",   "v_y (m/s)",     False),
    ("za",   "z_a (m)",       False),
    ("vz",   "v_z (m/s)",     False),
    ("th",   "theta (deg)",   True),
    ("om",   "omega (deg/s)", True),
    ("r",    "r (m)",         False),
]

# 读取
t, data = [], {k: [] for k, _, _ in COLS}
acc = []
with open(CSV_PATH, newline="") as f:
    r = csv.DictReader(f)
    for row in r:
        try:
            t.append(float(row["t"]))
            for k, _, ang in COLS:
                v = row.get(k, "")
                val = float(v) if v not in ("", None) else np.nan
                if ang:
                    val = val * 180.0 / math.pi
                data[k].append(val)
            a = row.get("accepted", "")
            acc.append(float(a) if a not in ("", None) else np.nan)
        except (ValueError, KeyError):
            continue

t = np.array(t)
for k in data:
    data[k] = np.array(data[k], dtype=float)

# 绘图：4x3 网格，前 9 格画状态，第 10 格画 accepted
fig, axes = plt.subplots(4, 3, figsize=(15, 10))
axes = axes.ravel()
for i, (k, title, _) in enumerate(COLS):
    ax = axes[i]
    ax.plot(t, data[k], color="#1f77b4", linewidth=1.2)
    ax.set_title(title, fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("time (s)")
# 第 10 格：accepted
ax = axes[9]
ax.plot(t, acc, color="#d62728", linewidth=1.0)
ax.set_title("accepted obs / frame", fontsize=11)
ax.grid(True, alpha=0.3)
ax.set_xlabel("time (s)")
# 后两格留空
for j in (10, 11):
    axes[j].axis("off")

fig.suptitle("EKF Third-Week: Vehicle State Estimation Curves", fontsize=14)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(PNG_PATH, dpi=130)
print("[plot_states] 已保存:", PNG_PATH, "  帧数:", len(t))
