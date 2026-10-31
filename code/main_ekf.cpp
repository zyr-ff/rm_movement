// ============================================================================
//  main_ekf.cpp —— 第三周主程序：在第二周 PnP 基础上加 EKF 整车状态估计
//
//  流水线：  视频帧 → 检测装甲板 → PnP 解算(相机系3D点) → IMU 时间同步取姿态
//           → EKF 预测(CTRV) → 匹配模型板↔检测板 → EKF 更新(含新息门控)
//           → 绘制四块板重投影 + 右上角实时状态 HUD → 导出视频 + 状态 CSV
//
//  运行：  cmake .. && make rm_ekf_third
//          ./rm_ekf_third [输入视频] [输出视频] [IMU.txt] [状态CSV]
// ============================================================================
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include "armor_detector.h"
#include "armor_types.h"
#include "camera_config.h"
#include "number_classifier.h"
#include "pnp_solver.h"
#include "neural_detector.h"

#include "ekf/ekf.h"
#include "ekf/imu_sync.h"
#include "ekf/observer_model.h"
#include "ekf/state.h"

#ifdef _WIN32
#include <windows.h>
#endif

// ===== 检测器开关 =====
#define USE_NN

// 默认路径（素材文件夹）
const char* VIDEO_PATH = "C:/work/electric_competition/robot/homework/third_train/素材/2026-05-20_23-57-31 (1).avi";
const char* IMU_PATH   = "C:/work/electric_competition/robot/homework/third_train/素材/2026-05-20_23-57-31.txt";

// 视频帧时间 ↔ IMU 时间的对齐偏移（秒）。若发现姿态对不上，调这个数。
// 默认 0：视频第0帧对齐 IMU 时间轴原点（本数据两者同录，基本同步）。
static const double VIDEO_IMU_OFFSET = 0.0;

// ---------- 工具：多路径找文件 ----------
static std::string findFile(const std::vector<std::string>& candidates) {
    for (const auto& p : candidates) { std::ifstream f(p); if (f.good()) return p; }
    return "";
}
#ifdef _WIN32
static std::string toAnsiShortPath(const std::string& utf8Path) {
    int n = MultiByteToWideChar(CP_UTF8, 0, utf8Path.c_str(), -1, nullptr, 0);
    if (n <= 0) return utf8Path;
    std::wstring w(n - 1, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8Path.c_str(), -1, &w[0], n);
    wchar_t shortBuf[MAX_PATH] = {0};
    if (!GetShortPathNameW(w.c_str(), shortBuf, MAX_PATH)) return utf8Path;
    int a = WideCharToMultiByte(CP_ACP, 0, shortBuf, -1, nullptr, 0, nullptr, nullptr);
    if (a <= 0) return utf8Path;
    std::string s(a - 1, '\0');
    WideCharToMultiByte(CP_ACP, 0, shortBuf, -1, &s[0], a, nullptr, nullptr);
    return s;
}
#endif

static cv::Scalar colorOf(const std::string& c) {
    return c == "red" ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 100, 0);
}

// 把 3D 相机系点投影到像素（理想针孔，用真实内参；重投影可视化用）
static cv::Point2f projectToPixel(const Mat& pc, double fx, double fy, double cx, double cy) {
    double z = pc(2, 0);
    if (z <= 1e-4) return cv::Point2f(-1, -1);
    return cv::Point2f((float)(fx * pc(0, 0) / z + cx),
                       (float)(fy * pc(1, 0) / z + cy));
}

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(65001); SetConsoleCP(65001);
#endif

    std::cout << "[相机内参 K]\n" << getCameraMatrix() << std::endl;
    cv::Mat K = getCameraMatrix();
    double fx = K.at<double>(0,0), fy = K.at<double>(1,1),
           cx = K.at<double>(0,2), cy = K.at<double>(1,2);

    // ---- 检测器 + PnP（与第二周完全一致）----
#ifdef USE_NN
    std::string nnModel = findFile({"../models/Infantry-v8n-fp32.onnx",
                                    "models/Infantry-v8n-fp32.onnx",
                                    "../../../second_train/code/models/Infantry-v8n-fp32.onnx"});
    ArmorDetectorNN detector(nnModel);
    if (!detector.ok()) std::cerr << "[警告] NN 模型不可用" << std::endl;
#else
    ArmorDetector detector;
#endif
    PnPSolver solver;
    std::string modelPath = findFile({"../models/mlp.onnx","models/mlp.onnx",
                                      "../../../second_train/code/models/mlp.onnx"});
    std::string labelPath = findFile({"../models/label.txt","models/label.txt",
                                      "../../../second_train/code/models/label.txt"});
    NumberClassifier classifier(modelPath, labelPath);

    // ---- IMU ----
    std::string imuPath = (argc >= 4) ? argv[3] : std::string(IMU_PATH);
    ImuSync imu;
    if (!imu.load(imuPath)) { std::cerr << "[错误] IMU 载入失败" << std::endl; return -1; }

    // ---- EKF ----
    EKFEstimator ekf;
    ekf.setProcessNoise(1e-3, 1e-2, 1e-3);
    ekf.setObservationNoise(0.01, 0.01, 0.04);   // z(深度)噪声更大
    ekf.setGate(7.815);                            // df=3, 95%

    // ---- 视频 ----
    std::string inVideo  = (argc >= 2) ? argv[1] : std::string(VIDEO_PATH);
    std::string outVideo = (argc >= 3) ? argv[2] : "ekf_output.avi";
    std::string csvPath  = (argc >= 5) ? argv[4] : "ekf_states.csv";
#ifdef _WIN32
    std::string openPath = toAnsiShortPath(inVideo);
#else
    std::string openPath = inVideo;
#endif
    cv::VideoCapture cap(openPath);
    if (!cap.isOpened()) { std::cerr << "无法打开视频: " << inVideo << std::endl; return -1; }

    double fps = cap.get(cv::CAP_PROP_FPS); if (fps <= 0) fps = 30.0;
    int vw = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int vh = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cv::VideoWriter writer(outVideo, cv::VideoWriter::fourcc('M','J','P','G'), fps, cv::Size(vw, vh));
    if (writer.isOpened())
        std::cout << "[导出] 视频: " << outVideo << "  (" << vw << "x" << vh << " @" << fps << ")\n";

    std::ofstream csv(csvPath);
    csv << "frame,t,cx,cy,vx,vy,za,vz,th,om,r,accepted\n";

    cv::Mat frame;
    int frameIdx = 0;
    double prevFrameTime = -1.0;
    int totalAccepted = 0, totalDetect = 0;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        ++frameIdx;

        // 本帧时间（秒）
        double frameTime = frameIdx / fps + VIDEO_IMU_OFFSET;

        // 1) 检测 + 数字识别 + PnP
        std::vector<Armor> armors = detector.detect(frame, "red");
        if (!armors.empty()) classifier.classify(frame, armors);
        for (auto& a : armors) solver.solve(a);

        // 2) 收集“相机系 3D 观测”（solvePnP 已带畸变求解，故已是去畸变理想点）
        std::vector<Mat> detCam;
        for (const auto& a : armors) {
            if (!a.positionValid) continue;
            Mat p(3, 1);
            p(0,0)=a.position.x; p(1,0)=a.position.y; p(2,0)=a.position.z;
            detCam.push_back(p);
        }
        totalDetect += (int)detCam.size();

        // 3) IMU 时间同步 → 当前帧旋转 R_observer，Rt = R_observerᵀ
        ImuSample s = imu.nearest(frameTime);
        Mat Robserver = ImuSync::quatToRot(s.qw, s.qx, s.qy, s.qz);
        Mat Rt = Robserver.T();
        ekf.setCurrentRt(Rt);

        // 4) EKF：先做预测，再匹配+更新
        double dt = (prevFrameTime < 0) ? (1.0/fps) : (frameTime - prevFrameTime);
        if (dt < 0 || dt > 0.2) dt = 1.0/fps;   // 防异常跳变
        prevFrameTime = frameTime;

        if (!ekf.initialized()) {
            if (!detCam.empty()) ekf.init(detCam[0], 0, Rt);   // 冷启动
        } else {
            ekf.predict(dt);
        }

        int accepted = 0;
        if (ekf.initialized() && !detCam.empty()) {
            // 4a) 预测四块板在相机系的位置
            std::vector<Mat> predCam;
            const auto& x = ekf.state();
            for (int j = 0; j < N_PLATES; ++j)
                predCam.push_back(ObserverModel::predictPlate(j, x, Rt));
            // 4b) 匹配：检测板 ↔ 模型板
            auto pairs = EKFEstimator::match(predCam, detCam);
            // 4c) 用匹配结果做更新（内部逐观测门控）
            std::vector<int> pidx; std::vector<Mat> z;
            for (auto& pr : pairs) { pidx.push_back(pr.second); z.push_back(detCam[pr.first]); }
            accepted = ekf.update(pidx, z);
            totalAccepted += accepted;
        }

        // 5) 绘制
        for (const auto& a : armors) {
            cv::Scalar col = colorOf(a.color);
            for (int i = 0; i < 4; ++i) {
                cv::line(frame, a.keypoints[i], a.keypoints[(i+1)%4], col, 2);
                cv::circle(frame, a.keypoints[i], 3, col, -1);
            }
            if (a.positionValid) {
                std::stringstream ss; ss << "(" << std::fixed << std::setprecision(2)
                    << a.position.x << "," << a.position.y << "," << a.position.z << ")m";
                cv::putText(frame, ss.str(), a.keypoints[1] + cv::Point2f(-50,18),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0,255,255), 1.2);
            }
        }

        // 5a) 绘制四块板“EKF 预测重投影”（青色十字），与真实检测对比
        if (ekf.initialized()) {
            const auto& x = ekf.state();
            for (int j = 0; j < N_PLATES; ++j) {
                Mat pc = ObserverModel::predictPlate(j, x, Rt);
                cv::Point2f uv = projectToPixel(pc, fx, fy, cx, cy);
                if (uv.x >= 0) {
                    cv::drawMarker(frame, uv, cv::Scalar(255,255,0), cv::MARKER_CROSS, 16, 2);
                    std::string lbl = "P" + std::to_string(j);
                    cv::putText(frame, lbl, uv + cv::Point2f(6,-6),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1.5);
                }
            }
        }

        // 5b) 右上角实时状态 HUD
        if (ekf.initialized()) {
            const auto& x = ekf.state();
            std::vector<std::string> lines = {
                "EKF STATE",
                "cx " + std::to_string((int)(x[IX_CX]*100)) + "cm",
                "cy " + std::to_string((int)(x[IX_CY]*100)) + "cm",
                "v  " + std::to_string((int)(std::hypot(x[IX_VX],x[IX_VY])*100)) + "cm/s",
                "za " + std::to_string((int)(x[IX_ZA]*100)) + "cm",
                "vz " + std::to_string((int)(x[IX_VZ]*100)) + "cm/s",
                "th " + std::to_string((int)(x[IX_TH]*180/M_PI)) + "deg",
                "om " + std::to_string((int)(x[IX_OM]*180/M_PI)) + "d/s",
                "r  " + std::to_string((int)(x[IX_R]*100)) + "cm",
                "acc " + std::to_string(accepted)
            };
            int bx = vw - 210, by = 20;
            cv::rectangle(frame, cv::Point(bx-10, by-10), cv::Point(vw-10, by + 22*((int)lines.size())),
                          cv::Scalar(0,0,0), -1);
            for (size_t i = 0; i < lines.size(); ++i)
                cv::putText(frame, lines[i], cv::Point(bx, by + 20*(int)i + 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1.5);
        }

        csv << frameIdx << "," << std::fixed << std::setprecision(4) << frameTime;
        if (ekf.initialized()) {
            const auto& x = ekf.state();
            csv << "," << x[IX_CX] << "," << x[IX_CY] << "," << x[IX_VX] << "," << x[IX_VY]
                << "," << x[IX_ZA] << "," << x[IX_VZ] << "," << x[IX_TH] << "," << x[IX_OM]
                << "," << x[IX_R];
        } else {
            csv << ",,,,,,,,,";
        }
        csv << "," << accepted << "\n";

        cv::putText(frame, "frame " + std::to_string(frameIdx), {10,30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
        cv::imshow("ekf preview", frame);
        if (writer.isOpened()) writer.write(frame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    cap.release();
    if (writer.isOpened()) writer.release();
    csv.close();
    cv::destroyAllWindows();
    std::cout << "==== 统计 ====\n总检测板次: " << totalDetect
              << "\nEKF 接受板次: " << totalAccepted
              << "\n状态 CSV: " << csvPath << std::endl;
    return 0;
}
