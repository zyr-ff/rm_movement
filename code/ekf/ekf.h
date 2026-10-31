#pragma once
// ============================================================================
//  ekf.h —— 扩展卡尔曼滤波器（整车状态估计）
//
//  设计对应你学的 EKF 循环：
//      Predict : x⁻ = f(x),   P⁻ = F·P·Fᵀ + Q
//      Update  : y = z − h(x⁻)
//                S = H·P⁻·Hᵀ + R
//                K = P⁻·Hᵀ·S⁻¹
//                x = x⁻ + K·y
//                P = (I − K·H)·P⁻·(I − K·H)ᵀ + K·R·Kᵀ   （Joseph 形式，数值稳定）
//
//  并包含作业要求的“匹配机制”与“跳变特殊处理（新息门控）”。
// ============================================================================
#include <vector>
#include <utility>
#include "linalg.h"
#include "state.h"
#include "observer_model.h"

class EKFEstimator {
public:
    EKFEstimator();

    bool initialized() const { return init_; }

    // 冷启动初始化：用第一帧某个观测(相机系3D点) + 该帧 Rt(=R_observerᵀ) 反推初始状态
    void init(const Mat& zCamFirst, int plateIdx, const Mat& Rt);

    // 过程噪声 Q（9×9，对角）。dt 越大、对模型信任越低则适当放大。
    void setProcessNoise(double qPos, double qVel, double qAng);

    // 观测噪声 R（3×3，对角）。注意单目深度(z)比横/纵向(x,y)更不准 → rz 更大。
    void setObservationNoise(double rx, double ry, double rz);

    // 新息门控阈值（马氏距离² 上限）。df=3 时 95%≈7.815，99%≈11.35。
    void setGate(double chi2) { gateChi2_ = chi2; }

    // 每帧把当前 R_observerᵀ（Rt）set 进来，update 内的 h/H 要用它做旋转
    void setCurrentRt(const Mat& Rt) { Rt_ = Rt; }

    // 预测步：dt 秒，使用 CTRV/恒速恒转速 运动模型
    void predict(double dt);

    // 更新步：传入“匹配好的”观测——plateIdx[k] 对应 z[k](相机系3D点)
    //  内部对每个观测做新息门控，拒绝野值/错配/跳变，仅用通过的观测做批量更新。
    //  返回：本帧接受的观测数（0 表示全被门控拒绝，仅做了预测）。
    int update(const std::vector<int>& plateIdx, const std::vector<Mat>& z);

    const std::vector<double>& state() const { return x_; }
    const Mat& cov() const { return P_; }

    // 匹配机制：把“模型预测的4块板相机系位置 predCam[j]” 与
    //           “本帧检测到的装甲板相机系位置 detCam[i]” 做最近邻配对。
    //  返回配对 (检测下标 i, 板下标 j)。采用贪心最近优先、避免重复占用同一块板。
    static std::vector<std::pair<int,int>>
    match(const std::vector<Mat>& predCam, const std::vector<Mat>& detCam);

private:
    std::vector<double> x_;   // 9 维状态
    Mat P_;                  // 9×9 协方差
    Mat Q_;                  // 9×9 过程噪声
    Mat R_;                  // 3×3 观测噪声
    double gateChi2_;
    Mat Rt_;                 // 当前帧 R_observerᵀ（update 内的 h/H 旋转用）
    bool init_ = false;
};
