// ============================================================================
//  observer_model.cpp —— h(x) 与 H 的具体实现（逐行对应教学推导）
// ============================================================================
#include "observer_model.h"
#include <cmath>

// 第 j 块板在基坐标系下的位置（与教学①完全一致）
//   P_base[j] = ( c_x + r·cos(θ+φ_j),
//                 c_y + r·sin(θ+φ_j),
//                 z_a )
static Mat plateInBase(int j, const std::vector<double>& x) {
    double cx = x[IX_CX], cy = x[IX_CY], za = x[IX_ZA], th = x[IX_TH],
           r  = (x[IX_R] > 1e-3) ? x[IX_R] : ROBOT_RADIUS;   // 防止 r 退化为 0
    double ang = th + PLATE_PHI[j];
    Mat p(3, 1);
    p(0, 0) = cx + r * std::cos(ang);
    p(1, 0) = cy + r * std::sin(ang);
    p(2, 0) = za;
    return p;
}

// P_base 对状态 x 的偏导（3×9）—— 教学里推导过的那段：
//   ∂/∂c_x=[1,0,0], ∂/∂c_y=[0,1,0], ∂/∂z_a=[0,0,1]
//   ∂/∂θ =[-r·sin(θ+φ_j), r·cos(θ+φ_j), 0]
//   ∂/∂r =[ cos(θ+φ_j),     sin(θ+φ_j), 0]
//   其余(速度 v_x,v_y,v_z, ω) = 0
static Mat dPbase_dx(int j, const std::vector<double>& x) {
    double r  = (x[IX_R] > 1e-3) ? x[IX_R] : ROBOT_RADIUS;
    double ang = x[IX_TH] + PLATE_PHI[j];
    Mat J(3, NX, 0.0);
    J(0, IX_CX) = 1.0;            // ∂x_base/∂c_x
    J(1, IX_CY) = 1.0;            // ∂y_base/∂c_y
    J(2, IX_ZA) = 1.0;            // ∂z_base/∂z_a
    J(0, IX_TH) = -r * std::sin(ang);   // cos 对 θ 求导得 -sin
    J(1, IX_TH) =  r * std::cos(ang);   // sin 对 θ 求导得 +cos
    J(0, IX_R)  =  std::cos(ang);
    J(1, IX_R)  =  std::sin(ang);
    return J;
}

Mat ObserverModel::predictPlate(int j, const std::vector<double>& x, const Mat& Rt) {
    Mat pBase = plateInBase(j, x);
    // ② 相机坐标 = Rt · P_base
    return Rt * pBase;
}

Mat ObserverModel::jacobianPlate(int j, const std::vector<double>& x, const Mat& Rt) {
    Mat dP = dPbase_dx(j, x);
    // H_j = Rt · ∂P_base/∂x   （Rt 在给定帧是常数，不参与对状态求偏导）
    return Rt * dP;
}
