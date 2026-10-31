#pragma once     //防止头文件被多个cpp包含
// ============================================================================
//  linalg.h  —— 极简稠密矩阵/向量类（仅 EKF 内部数学用，不依赖 OpenCV）
//  说明：第三周作业里 EKF 的状态最多 9 维、观测每块板 3 维，矩阵规模很小，
//        用一个“动态大小但带缓存”的简单实现即可，且能让你逐行看清
//        P = F·P·Fᵀ + Q、K = P·Hᵀ·S⁻¹ 这些公式的每一步。
//  如果你以后想换成 Eigen，只需把这里替换掉即可，EKF 主体不用改。
// ============================================================================
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>

class Mat {
public:
    int rows = 0, cols = 0;   //做乘法加减时需要检查维度
    std::vector<double> d;   // 行主序(row-major): d[i*cols + j]

    Mat() = default;
    Mat(int r, int c) : rows(r), cols(c), d((size_t)r * c, 0.0) {}
    Mat(int r, int c, double v) : rows(r), cols(c), d((size_t)r * c, v) {}

    // 生成单位阵
    static Mat eye(int n) {
        Mat m(n, n);
        for (int i = 0; i < n; ++i) m(i, i) = 1.0;
        return m;
    }

    double& operator()(int i, int j) { return d[(size_t)i * cols + j]; }
    double  operator()(int i, int j) const { return d[(size_t)i * cols + j]; }

    // 矩阵乘法  C = A·B
    Mat operator*(const Mat& b) const {
        if (cols != b.rows)
            throw std::runtime_error("Mat::* 维度不匹配");  //检查A的列数 == B的行数
        Mat c(rows, b.cols);
        for (int i = 0; i < rows; ++i)
            for (int k = 0; k < cols; ++k) {
                double aik = (*this)(i, k);
                if (aik == 0.0) continue;
                for (int j = 0; j < b.cols; ++j)
                    c(i, j) += aik * b(k, j);
            }
        return c;
    }

    // 标量乘法  C = A · s
    Mat operator*(double s) const {
        Mat c(rows, cols);
        for (size_t i = 0; i < d.size(); ++i) c.d[i] = d[i] * s;
        return c;
    }

    Mat operator+(const Mat& b) const {
        if (rows != b.rows || cols != b.cols) throw std::runtime_error("Mat::+ 维度不匹配");
        Mat c(rows, cols);
        for (size_t i = 0; i < d.size(); ++i) c.d[i] = d[i] + b.d[i];
        return c;
    }
    Mat operator-(const Mat& b) const {
        if (rows != b.rows || cols != b.cols) throw std::runtime_error("Mat::- 维度不匹配");
        Mat c(rows, cols);
        for (size_t i = 0; i < d.size(); ++i) c.d[i] = d[i] - b.d[i];
        return c;
    }
    Mat& operator+=(const Mat& b) { *this = *this + b; return *this; }

    // 转置
    Mat T() const {
        Mat t(cols, rows);
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                t(j, i) = (*this)(i, j);
        return t;
    }

    // 高斯-约当求逆（作业规模够用；若失败抛异常，由调用方做兜底）
    Mat inv() const {
        if (rows != cols) throw std::runtime_error("Mat::inv 非方阵");
        int n = rows;
        Mat a = *this;                 // 副本
        Mat inv = eye(n);
        for (int col = 0; col < n; ++col) {
            // 选主元（避免接近 0 的对角元）
            int piv = col;
            double maxv = std::fabs(a(col, col));
            for (int r = col + 1; r < n; ++r) {
                if (std::fabs(a(r, col)) > maxv) { maxv = std::fabs(a(r, col)); piv = r; }
            }
            if (maxv < 1e-12) throw std::runtime_error("Mat::inv 奇异矩阵");
            if (piv != col) {
                for (int k = 0; k < n; ++k) {
                    std::swap(a(col, k), a(piv, k));
                    std::swap(inv(col, k), inv(piv, k));
                }
            }
            double diag = a(col, col);
            for (int k = 0; k < n; ++k) { a(col, k) /= diag; inv(col, k) /= diag; }
            for (int r = 0; r < n; ++r) {
                if (r == col) continue;
                double f = a(r, col);
                if (f == 0.0) continue;
                for (int k = 0; k < n; ++k) {
                    a(r, k) -= f * a(col, k);
                    inv(r, k) -= f * inv(col, k);
                }
            }
        }
        return inv;
    }

    void print(const std::string& name = "") const {
        if (!name.empty()) std::cout << name << " =\n";
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j)
                std::cout << (*this)(i, j) << "\t";
            std::cout << "\n";
        }
    }
};

// 行向量形式构造 Mat（方便写 H 的某一行、或残差向量）
static Mat rowVec(const std::vector<double>& v) {
    Mat m(1, (int)v.size());
    for (size_t i = 0; i < v.size(); ++i) m(0, (int)i) = v[i];
    return m;
}
static Mat colVec(const std::vector<double>& v) {
    Mat m((int)v.size(), 1);
    for (size_t i = 0; i < v.size(); ++i) m((int)i, 0) = v[i];
    return m;
}
