#!/usr/bin/env python
# ============================================================================
#  gen_example_csv.py —— 用 numpy 复现 EKF 逻辑，生成一份“示例” ekf_states.csv
#  仅用于演示 plot_states.py 的输出效果；真实数据由 C++ 主程序跑出来。
#  用法:  python tools/gen_example_csv.py [输出csv]
# ============================================================================
import sys, math, csv
import numpy as np

np.random.seed(7)
NX = 9
IX_CX, IX_CY, IX_VX, IX_VY, IX_ZA, IX_VZ, IX_TH, IX_OM, IX_R = range(9)
N_PLATES = 4
PHI = np.array([0.0, math.pi/2, math.pi, 3*math.pi/2])
R_ROBOT = 0.25
Z_HEIGHT = 0.30

def plate_in_base(x, j):
    ang = x[IX_TH] + PHI[j]
    return np.array([x[IX_CX] + R_ROBOT*math.cos(ang),
                     x[IX_CY] + R_ROBOT*math.sin(ang),
                     x[IX_ZA]])

# 真实状态
truth = np.zeros(NX)
truth[IX_CX]=0.0; truth[IX_CY]=0.0
truth[IX_VX]=0.30; truth[IX_VY]=0.12
truth[IX_ZA]=Z_HEIGHT; truth[IX_VZ]=0.0
truth[IX_TH]=0.4; truth[IX_OM]=0.10
truth[IX_R]=R_ROBOT

# EKF 状态/协方差
x = np.zeros(NX)
P = np.eye(NX)
P[IX_CX,IX_CX]=P[IX_CY,IX_CY]=0.1
P[IX_VX,IX_VX]=P[IX_VY,IX_VY]=1.0
P[IX_ZA,IX_ZA]=0.01; P[IX_VZ,IX_VZ]=1.0
P[IX_TH,IX_TH]=0.1; P[IX_OM,IX_OM]=1.0; P[IX_R,IX_R]=0.01
Q = np.eye(NX); Q[0,0]=Q[1,1]=Q[4,4]=1e-3; Q[2,2]=Q[3,3]=Q[5,5]=1e-2; Q[6,6]=1e-3; Q[7,7]=1e-2; Q[8,8]=1e-4
R = np.diag([0.01,0.01,0.04])
init_ = False
dt = 0.04
out = open(sys.argv[1] if len(sys.argv)>1 else "ekf_states.csv", "w", newline="")
w = csv.writer(out)
w.writerow(["frame","t","cx","cy","vx","vy","za","vz","th","om","r","accepted"])

for k in range(1, 401):
    # 真值推进
    truth[IX_CX]+=truth[IX_VX]*dt; truth[IX_CY]+=truth[IX_VY]*dt
    truth[IX_ZA]+=truth[IX_VZ]*dt; truth[IX_TH]+=truth[IX_OM]*dt
    t = k/fps if False else k*dt
    # 观测（4 块板，相机系=Rt=I），加噪声
    zs=[]; idx=[]
    for j in range(N_PLATES):
        p = plate_in_base(truth, j)
        p = p + np.random.normal(0, [0.03,0.03,0.2])
        zs.append(p); idx.append(j)
    if not init_:
        pb = zs[0]
        r0 = math.hypot(pb[0],pb[1]); th0 = math.atan2(pb[1],pb[0]) - PHI[0]
        x[IX_ZA]=pb[2]; x[IX_TH]=th0; x[IX_R]=r0; x[IX_CX]=x[IX_CY]=0.0
        init_=True
        w.writerow([k, f"{t:.4f}", "","","","","","","","","","0"])
        continue
    # 预测
    x[IX_CX]+=x[IX_VX]*dt; x[IX_CY]+=x[IX_VY]*dt; x[IX_ZA]+=x[IX_VZ]*dt; x[IX_TH]+=x[IX_OM]*dt
    F = np.eye(NX); F[0,2]=dt; F[1,3]=dt; F[4,5]=dt; F[6,7]=dt
    P = F@P@F.T + Q
    # 更新（批量，4 块板）
    m=len(zs); H=np.zeros((3*m,NX)); Y=np.zeros(3*m); Rb=np.zeros((3*m,3*m))
    for q,(j,z) in enumerate(zip(idx,zs)):
        ang=x[IX_TH]+PHI[j]
        hx=np.array([x[IX_CX]+x[IX_R]*math.cos(ang), x[IX_CY]+x[IX_R]*math.sin(ang), x[IX_ZA]])
        Hj=np.zeros((3,NX))
        Hj[0,IX_CX]=1; Hj[1,IX_CY]=1; Hj[2,IX_ZA]=1
        Hj[0,IX_TH]=-x[IX_R]*math.sin(ang); Hj[1,IX_TH]=x[IX_R]*math.cos(ang)
        Hj[0,IX_R]=math.cos(ang); Hj[1,IX_R]=math.sin(ang)
        H[3*q:3*q+3,:]=Hj; Y[3*q:3*q+3]=z-hx; Rb[3*q:3*q+3,3*q:3*q+3]=R
    S=H@P@H.T+Rb; K=P@H.T@np.linalg.inv(S)
    x=x+K@Y; I=np.eye(NX); IKH=I-K@H; P=IKH@P@IKH.T+K@Rb@K.T
    w.writerow([k, f"{t:.4f}", f"{x[IX_CX]:.4f}", f"{x[IX_CY]:.4f}", f"{x[IX_VX]:.4f}",
                f"{x[IX_VY]:.4f}", f"{x[IX_ZA]:.4f}", f"{x[IX_VZ]:.4f}", f"{x[IX_TH]:.4f}",
                f"{x[IX_OM]:.4f}", f"{x[IX_R]:.4f}", f"{m}"])
out.close()
print("示例 CSV 已生成")
