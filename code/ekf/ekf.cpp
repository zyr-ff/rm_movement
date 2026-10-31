// ============================================================================
//  ekf.cpp —— EKF 预测/更新/匹配/门控 实现
// ============================================================================
#include "ekf.h"
#include <cmath>
#include <algorithm>
#include <iostream>

EKFEstimator::EKFEstimator() {
    x_.assign(NX, 0.0);
    P_ = Mat::eye(NX) * 1.0;          // 初始协方差给一个保守的大值
    Q_ = Mat::eye(NX) * 1e-4;
    R_ = Mat::eye(3) * 0.02;          // 默认观测噪声
    gateChi2_ = 7.815;                // df=3, 95%
}

void EKFEstimator::setProcessNoise(double qPos, double qVel, double qAng) {
    Q_ = Mat(NX, NX, 0.0);
    Q_(IX_CX, IX_CX) = qPos;  Q_(IX_CY, IX_CY) = qPos;
    Q_(IX_VX, IX_VX) = qVel;  Q_(IX_VY, IX_VY) = qVel;
    Q_(IX_ZA, IX_ZA) = qPos;  Q_(IX_VZ, IX_VZ) = qVel;
    Q_(IX_TH, IX_TH) = qAng;  Q_(IX_OM, IX_OM) = qVel;   // ω 用速度级噪声
    Q_(IX_R,  IX_R)  = qPos * 0.1;                        // 半径基本不动
}

void EKFEstimator::setObservationNoise(double rx, double ry, double rz) {
    R_ = Mat(3, 3, 0.0);
    R_(0, 0) = rx; R_(1, 1) = ry; R_(2, 2) = rz;
}

void EKFEstimator::init(const Mat& zCamFirst, int plateIdx, const Mat& Rt) {
    // 相机系观测 → 基坐标系：P_base = R_observer · z_cam = Rtᵀ · z_cam
    Mat Robserver = Rt.T();
    Mat pBase = Robserver * zCamFirst;   // 3×1
    double px = pBase(0, 0), py = pBase(1, 0), pz = pBase(2, 0);

    double r0   = std::hypot(px, py);
    double th0  = std::atan2(py, px) - PLATE_PHI[plateIdx];   // 反推车头朝向

    x_.assign(NX, 0.0);
    x_[IX_CX] = 0.0;            // 以观察者初始位置为原点
    x_[IX_CY] = 0.0;
    x_[IX_VX] = 0.0; x_[IX_VY] = 0.0;
    x_[IX_ZA] = pz;             // 板高度直接取观测
    x_[IX_VZ] = 0.0;
    x_[IX_TH] = th0;
    x_[IX_OM] = 0.0;
    x_[IX_R]  = r0;

    // 初始协方差：位置/朝向给中等不确定，速度给较大不确定（初值=0 很不准）
    P_ = Mat(NX, NX, 0.0);
    P_(IX_CX, IX_CX) = 0.1;  P_(IX_CY, IX_CY) = 0.1;
    P_(IX_VX, IX_VX) = 1.0;  P_(IX_VY, IX_VY) = 1.0;
    P_(IX_ZA, IX_ZA) = 0.01;
    P_(IX_VZ, IX_VZ) = 1.0;
    P_(IX_TH, IX_TH) = 0.1;
    P_(IX_OM, IX_OM) = 1.0;
    P_(IX_R,  IX_R)  = 0.01;
    init_ = true;
}

// ---- 预测步：恒速(地面) + 恒转速(θ) ----
//   c_x += v_x·dt ;  c_y += v_y·dt
//   z_a += v_z·dt
//   θ   += ω·dt
//   其余(v_x,v_y,v_z,ω,r)保持不变
//  F 为线性：除对角线 1 外，F[CX][VX]=F[CY][VY]=F[ZA][VZ]=F[TH][OM]=dt
void EKFEstimator::predict(double dt) {
    if (!init_) return;
    // 状态转移
    x_[IX_CX] += x_[IX_VX] * dt;
    x_[IX_CY] += x_[IX_VY] * dt;
    x_[IX_ZA] += x_[IX_VZ] * dt;
    x_[IX_TH] += x_[IX_OM] * dt;
    // θ 角度回卷到 [-π, π)，避免数值漂移
    while (x_[IX_TH] >  M_PI) x_[IX_TH] -= 2 * M_PI;
    while (x_[IX_TH] < -M_PI) x_[IX_TH] += 2 * M_PI;

    // 雅可比 F（线性，直接构造）
    Mat F = Mat::eye(NX);
    F(IX_CX, IX_VX) = dt;
    F(IX_CY, IX_VY) = dt;
    F(IX_ZA, IX_VZ) = dt;
    F(IX_TH, IX_OM) = dt;

    // P⁻ = F·P·Fᵀ + Q
    P_ = F * P_ * F.T() + Q_;
}

// ---- 更新步：匹配后的观测逐个门控，通过的做批量更新 ----
int EKFEstimator::update(const std::vector<int>& plateIdx,
                         const std::vector<Mat>& z) {
    if (!init_ || z.empty()) return 0;

    // 收集被门控“通过”的观测
    std::vector<int>   accJ;
    std::vector<Mat>   accY;     // 残差 y = z - h(x⁻)
    std::vector<Mat>   accH;     // 对应雅可比 H_j

    for (size_t k = 0; k < z.size(); ++k) {
        int j = plateIdx[k];
        Mat hx = ObserverModel::predictPlate(j, x_, Rt_);
        Mat y  = z[k] - hx;                 // 新息（3×1）
        Mat Hj = ObserverModel::jacobianPlate(j, x_, Rt_);  // 3×9
        // 新息协方差 S_j = Hj·P·Hjᵀ + R
        Mat Sj = Hj * P_ * Hj.T() + R_;
        Mat SjInv;
        try { SjInv = Sj.inv(); }
        catch (...) { continue; }            // 奇异则跳过该观测
        // 马氏距离² d² = yᵀ·S⁻¹·y
        Mat d2m = y.T() * SjInv * y;
        double d2 = d2m(0, 0);
        if (d2 > gateChi2_) {
            // —— 跳变特殊处理：该观测偏离模型太远，判为野值/错配/跳变，丢弃不更新 ——
            continue;
        }
        accJ.push_back(j);
        accY.push_back(y);
        accH.push_back(Hj);
    }

    int m = (int)accJ.size();
    if (m == 0) return 0;   // 本帧全部被门控拒绝，仅做了预测

    // 拼成批量 H (3m×9)、y (3m×1)、R (3m×3m)
    Mat H(3 * m, NX, 0.0);
    Mat Y(3 * m, 1, 0.0);
    Mat Rblk(3 * m, 3 * m, 0.0);
    for (int k = 0; k < m; ++k) {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < NX; ++c)
                H(3 * k + r, c) = accH[k](r, c);
        for (int r = 0; r < 3; ++r)
            Y(3 * k + r, 0) = accY[k](r, 0);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Rblk(3 * k + r, 3 * k + c) = R_(r, c);
    }

    // 卡尔曼增益 K = P·Hᵀ·S⁻¹,  S = H·P·Hᵀ + Rblk
    Mat S = H * P_ * H.T() + Rblk;
    Mat K, Sinv;
    try { Sinv = S.inv(); }
    catch (...) { return 0; }
    K = P_ * H.T() * Sinv;

    // 状态更新
    Mat dx = K * Y;
    for (int i = 0; i < NX; ++i) x_[i] += dx(i, 0);

    // 协方差更新（Joseph 形式，数值更稳）
    Mat I = Mat::eye(NX);
    Mat IKH = I - K * H;
    P_ = IKH * P_ * IKH.T() + K * Rblk * K.T();

    return m;
}

// ---- 匹配机制：检测板 ↔ 模型板 最近邻贪心配对 ----
std::vector<std::pair<int,int>>
EKFEstimator::match(const std::vector<Mat>& predCam, const std::vector<Mat>& detCam) {
    std::vector<std::pair<int,int>> pairs;
    int nP = (int)predCam.size(), nD = (int)detCam.size();
    if (nP == 0 || nD == 0) return pairs;

    std::vector<bool> usedP(nP, false), usedD(nD, false);
    // 收集所有 (i,j,dist) 候选，按距离升序贪心
    struct Cand { double d; int i, j; };
    std::vector<Cand> cands;
    for (int i = 0; i < nD; ++i)
        for (int j = 0; j < nP; ++j) {
            double d = 0;
            for (int r = 0; r < 3; ++r) {
                double v = detCam[i](r, 0) - predCam[j](r, 0);
                d += v * v;
            }
            cands.push_back({ std::sqrt(d), i, j });
        }
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.d < b.d; });

    for (const auto& c : cands) {
        if (usedD[c.i] || usedP[c.j]) continue;   // 已被占用则跳过
        usedD[c.i] = true; usedP[c.j] = true;
        pairs.push_back({ c.i, c.j });
    }
    return pairs;
}
