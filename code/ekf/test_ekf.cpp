// 纯数学自测：不依赖 OpenCV，验证 EKF 预测/更新/门控逻辑正确。
// 编译: g++ -std=c++17 -I. test_ekf.cpp observer_model.cpp ekf.cpp -o test_ekf
#include "ekf.h"
#include "observer_model.h"
#include "state.h"
#include <iostream>
#include <cmath>
#include <cstdlib>

static double rnd() { // 粗略正态 N(0,1)：多个均匀平均
    double s = 0; for (int i=0;i<6;++i) s += ((double)rand()/RAND_MAX*2-1);
    return s/2.0;
}

int main() {
    srand(12345);
    EKFEstimator ekf;
    ekf.setProcessNoise(1e-3, 1e-2, 1e-3);
    ekf.setObservationNoise(0.01, 0.01, 0.04);  // z 噪声更大（单目深度不准）
    ekf.setGate(7.815);

    // 真实状态
    std::vector<double> truth(NX, 0.0);
    truth[IX_CX]=0.0; truth[IX_CY]=0.0;
    truth[IX_VX]=0.30; truth[IX_VY]=0.10;
    truth[IX_ZA]=0.30; truth[IX_VZ]=0.0;
    truth[IX_TH]=0.4;  truth[IX_OM]=0.08;
    truth[IX_R]=0.25;

    Mat Rt = Mat::eye(3);   // 测试用：观察者不旋转，Rt=I
    ekf.setCurrentRt(Rt);

    double dt = 0.04;
    bool first = true;
    const int N = 200;
    double errSum = 0;

    for (int k = 0; k < N; ++k) {
        // —— 真值按与 EKF 相同的运动模型推进 —— //
        truth[IX_CX] += truth[IX_VX]*dt;
        truth[IX_CY] += truth[IX_VY]*dt;
        truth[IX_ZA] += truth[IX_VZ]*dt;
        truth[IX_TH] += truth[IX_OM]*dt;

        // —— 生成 4 块板的观测（相机系，Rt=I 时即基系）并加噪声 —— //
        std::vector<Mat> z;
        std::vector<int> pidx;
        for (int j = 0; j < N_PLATES; ++j) {
            Mat p = ObserverModel::predictPlate(j, truth, Rt);
            for (int r=0;r<3;++r){
                double n = rnd() * (r==2 ? 0.2 : 0.03); // z 噪声大
                p(r,0) += n;
            }
            z.push_back(p); pidx.push_back(j);
        }
        // 故意在第 100 帧塞一个野值（第 0 块板坐标猛跳），测试门控
        if (k == 100) { z[0](0,0) += 5.0; }

        if (first) {
            ekf.init(z[0], 0, Rt);
            first = false;
            continue;   // 初始化帧不更新
        }

        ekf.predict(dt);
        int accepted = ekf.update(pidx, z);
        if (k % 40 == 0 || k == 100) {
            const auto& x = ekf.state();
            std::cout << "step " << k
                      << " accepted=" << accepted
                      << " cx=" << x[IX_CX] << " cy=" << x[IX_CY]
                      << " vx=" << x[IX_VX] << " th=" << x[IX_TH]
                      << " r="  << x[IX_R] << std::endl;
        }
        if (k > 150) { // 收敛后统计位置误差
            const auto& x = ekf.state();
            errSum += std::hypot(x[IX_CX]-truth[IX_CX], x[IX_CY]-truth[IX_CY]);
        }
    }
    double avgErr = errSum / (N-150);
    std::cout << "[结果] 收敛后平均位置误差 = " << avgErr << " m" << std::endl;
    std::cout << (avgErr < 0.1 ? "PASS ✅" : "FAIL ❌") << std::endl;
    return avgErr < 0.1 ? 0 : 1;
}
