#pragma once
// ============================================================================
//  imu_sync.h —— IMU 四元数读取、归一化、四元数→旋转矩阵、时间同步
//  数据格式（你提供的 txt）：  时间戳(s)  qw qx qy qz
//  约定：Hamilton（w 在前），与公式/大多数库一致；IMU 约 25Hz。
// ============================================================================
#include <string>
#include <vector>
#include "linalg.h"

struct ImuSample {
    double t;        // 时间戳（秒）
    double qw, qx, qy, qz;  // 四元数（已归一化）
};

class ImuSync {
public:
    // 从 txt 载入全部样本
    bool load(const std::string& path);

    // 给定视频帧曝光时刻 t_frame（秒），返回“时间上最接近”的 IMU 样本
    // （线性扫描即可：数据量小；返回最近邻，保证外参与曝光时刻对齐）
    ImuSample nearest(double tFrame) const;

    // 四元数（Hamilton, w 在前）→ 3x3 旋转矩阵 R
    //   R 把“基坐标系(世界)”下的向量旋到“IMU/机体坐标系”下。
    //   公式：R = [[1-2(y²+z²),  2(xy-wz),   2(xz+wy)],
    //              [2(xy+wz),    1-2(x²+z²), 2(yz-wx)],
    //              [2(xz-wy),    2(yz+wx),   1-2(x²+y²)]]
    //   其中 (w,x,y,z) = (qw,qx,qy,qz)，且已归一化。
    static Mat quatToRot(double qw, double qx, double qy, double qz);

    size_t size() const { return samples_.size(); }

private:
    std::vector<ImuSample> samples_;
};
