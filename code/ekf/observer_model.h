#pragma once
// ============================================================================
//  observer_model.h —— 观测模型 h(x) 与 雅可比 H
//
//  观测是什么：第二周 PnP 解出的 armor.position 是“装甲板中心在相机系的 3D 坐标”，
//              且 solvePnP 已带畸变系数求解，故它已是“去畸变后的理想相机系点”。
//              因此本作业的观测直接取这个 3D 点，h(x) 也只需输出“模型预测的
//              同一块板在相机系的 3D 坐标”，无需再做像素投影、没有 z_c² 项。
//
//  坐标系链路（对应你学的 h(x) 三段）：
//      ① 世界(基)坐标 P_base[j] = 车中心 + 沿(θ+φ_j)方向的半径向量
//      ② 相机坐标 P_cam[j]  = R_observerᵀ · P_base[j]
//         （R_observer 由 IMU 四元数得到，是把基系旋到机体/相机系的旋转；
//          取转置即把基系点表达到相机系，等价于“外参”）
//      ③ 本作业观测是 3D 点，③ 不需要再投影到像素（画重投影视频时才投影成 2D）
//
//  h(x)[j] 输出 P_cam[j]（3 维）；H = ∂h/∂x 是 3×9，只有 c_x,c_y,z_a,θ,r 四列非零。
// ============================================================================
#include <vector>
#include "linalg.h"
#include "state.h"

class ObserverModel {
public:
    // 第 j 块板在相机系下的预测 3D 位置（3×1）
    //  Rt : R_observer 的转置（3×3），把基系点旋到相机系（每帧由 IMU 给出，常数）
    static Mat predictPlate(int j, const std::vector<double>& x, const Mat& Rt);

    // 第 j 块板观测雅可比 H_j = ∂h_j/∂x （3×9）
    static Mat jacobianPlate(int j, const std::vector<double>& x, const Mat& Rt);
};
