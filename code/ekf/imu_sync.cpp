// ============================================================================
//  imu_sync.cpp —— IMU 读取与时间同步实现
// ============================================================================
#include "imu_sync.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <filesystem>
#ifdef _WIN32
#include <windows.h>
#endif

// 把 UTF-8 路径转成 std::filesystem::path（Windows 下走宽字符 API，
// 可直接打开含中文的目录/文件名；不依赖 8.3 短文件名）。
static std::filesystem::path utf8ToFsPath(const std::string& u8) {
#ifdef _WIN32
    int n = MultiByteToWideChar(CP_UTF8, 0, u8.c_str(), -1, nullptr, 0);
    if (n > 0) {
        std::wstring w(n, L'\0');
        MultiByteToWideChar(CP_UTF8, 0, u8.c_str(), -1, &w[0], n);
        return std::filesystem::path(w);   // 由宽字符串构造，native 即 UTF-16
    }
#endif
    return std::filesystem::path(u8);
}

bool ImuSync::load(const std::string& path) {
    std::ifstream f(utf8ToFsPath(path));
    if (!f.is_open()) {
        std::cerr << "[IMU] 无法打开文件: " << path << std::endl;
        return false;
    }
    samples_.clear();
    std::string line;
    while (std::getline(f, line)) {
        // 跳过空行/注释
        if (line.empty() || line[0] == '#') continue;
        ImuSample s;
        // 格式: 时间戳(s) qw qx qy qz
        if (std::sscanf(line.c_str(), "%lf %lf %lf %lf %lf",
                        &s.t, &s.qw, &s.qx, &s.qy, &s.qz) == 5) {
            // —— 归一化：保证是单位四元数，否则由它算出的 R 不严格正交 ——
            double n = std::sqrt(s.qw * s.qw + s.qx * s.qx + s.qy * s.qy + s.qz * s.qz);
            if (n > 1e-9) { s.qw /= n; s.qx /= n; s.qy /= n; s.qz /= n; }
            samples_.push_back(s);
        }
    }
    std::cout << "[IMU] 载入 " << samples_.size() << " 条样本" << std::endl;
    return !samples_.empty();
}

ImuSample ImuSync::nearest(double tFrame) const {
    ImuSample best = samples_.front();
    double bestDt = std::fabs(best.t - tFrame);
    for (const auto& s : samples_) {
        double dt = std::fabs(s.t - tFrame);
        if (dt < bestDt) { bestDt = dt; best = s; }
    }
    return best;
}

Mat ImuSync::quatToRot(double qw, double qx, double qy, double qz) {
    // 归一化（双保险，即使 load 已做）
    double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (n < 1e-9) n = 1.0;
    qw /= n; qx /= n; qy /= n; qz /= n;

    double x = qx, y = qy, z = qz, w = qw;
    Mat R(3, 3);
    R(0, 0) = 1 - 2 * (y * y + z * z);
    R(0, 1) = 2 * (x * y - w * z);
    R(0, 2) = 2 * (x * z + w * y);
    R(1, 0) = 2 * (x * y + w * z);
    R(1, 1) = 1 - 2 * (x * x + z * z);
    R(1, 2) = 2 * (y * z - w * x);
    R(2, 0) = 2 * (x * z - w * y);
    R(2, 1) = 2 * (y * z + w * x);
    R(2, 2) = 1 - 2 * (x * x + y * y);
    return R;
}
