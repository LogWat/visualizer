#pragma once
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "matplotlibcpp.h" // https://github.com/lava/matplotlib-cpp

namespace plot = matplotlibcpp;

namespace utils4visualize {

class Visualizer {
public:
    Visualizer() {
        std::vector<float> pwcs_x, pwcs_y;
    }
    ~Visualizer() {}

// x, yの共分散行列から誤差楕円を描画
void plot_covariance_ellipse(
    const Eigen::Matrix2f &covariance,
    const Eigen::Vector2f &mean,
    const float stddev
) {
    // 固有値分解
    Eigen::EigenSolver<Eigen::Matrix2f> es(covariance);
    Eigen::Vector2f eigenvalues = es.eigenvalues().real();
    Eigen::Matrix2f eigenvectors = es.eigenvectors().real();

    // 固有ベクトルの角度
    float angle = atan2(eigenvectors(1, 0), eigenvectors(0, 0));

    const float theta = 2 * M_PI / 100;
    std::vector<float> x, y;
    for (int i = 0; i < 100; i++) {
        float _x = stddev * sqrt(eigenvalues(0)) * cos(theta * i);
        float _y = stddev * sqrt(eigenvalues(1)) * sin(theta * i);
        x.push_back(_x * cos(angle) - _y * sin(angle) + mean(0));
        y.push_back(_x * sin(angle) + _y * cos(angle) + mean(1));
    }
    plot::clf();
    plot::plot(x, y, "b-");
    plot::plot({mean(0)}, {mean(1)}, "ro");
    plot::xlim(mean(0) - 3, mean(0) + 3);
    plot::ylim(mean(1) - 3, mean(1) + 3);
    plot::title("Covariance Ellipse (95% Confidence)");
    plot::xlabel("X");
    plot::ylabel("Y");
    plot::pause(0.01);
}

void plot_covariance_ellipse_detail(
    const Eigen::Matrix2f &covariance,
    const Eigen::Vector2f &mean
    // const float confidence = 0.95  // デフォルトで95%信頼区間
) {
    // カイ二乗分布の逆関数（自由度2、95%信頼区間）
    float chi2_val = 5.991;  // 0.95信頼区間の臨界値
    float chi2_val_99 = 9.210;  // 0.99信頼区間の臨界値
    float chi2_val_65 = 2.706;  // 0.65信頼区間の臨界値
    
    // 固有値分解
    Eigen::EigenSolver<Eigen::Matrix2f> es(covariance);
    Eigen::Vector2f eigenvalues = es.eigenvalues().real();
    Eigen::Matrix2f eigenvectors = es.eigenvectors().real();
    
    float angle = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));
    
    float axis1 = std::sqrt(chi2_val * eigenvalues(0));
    float axis2 = std::sqrt(chi2_val * eigenvalues(1));
    float axis1_99 = std::sqrt(chi2_val_99 * eigenvalues(0));
    float axis2_99 = std::sqrt(chi2_val_99 * eigenvalues(1));
    float axis1_65 = std::sqrt(chi2_val_65 * eigenvalues(0));
    float axis2_65 = std::sqrt(chi2_val_65 * eigenvalues(1));
    
    std::vector<float> x, y, x_99, y_99, x_65, y_65;
    const int num_points = 100;
    const float theta = 2 * M_PI / num_points;
    
    for (int i = 0; i < num_points; i++) {
        float _x = std::cos(theta * i);
        float _y = std::sin(theta * i);
        float rotated_x = (axis1 * _x * std::cos(angle) - axis2 * _y * std::sin(angle)) + mean(0);
        float rotated_y = (axis1 * _x * std::sin(angle) + axis2 * _y * std::cos(angle)) + mean(1);
        float rotated_x_99 = (axis1_99 * _x * std::cos(angle) - axis2_99 * _y * std::sin(angle)) + mean(0);
        float rotated_y_99 = (axis1_99 * _x * std::sin(angle) + axis2_99 * _y * std::cos(angle)) + mean(1);
        float rotated_x_65 = (axis1_65 * _x * std::cos(angle) - axis2_65 * _y * std::sin(angle)) + mean(0);
        float rotated_y_65 = (axis1_65 * _x * std::sin(angle) + axis2_65 * _y * std::cos(angle)) + mean(1);
        x.push_back(rotated_x);
        y.push_back(rotated_y);
        x_99.push_back(rotated_x_99);
        y_99.push_back(rotated_y_99);
        x_65.push_back(rotated_x_65);
        y_65.push_back(rotated_y_65);
    }
    
    plot::clf();
    plot::plot(x_65, y_65, "y-");
    plot::plot(x, y, "g-");
    plot::plot(x_99, y_99, "b-");
    plot::plot({mean(0)}, {mean(1)}, "y.");
    plot::xlim(mean(0) - 1.5, mean(0) + 1.5);
    plot::ylim(mean(1) - 1.5, mean(1) + 1.5);
    plot::title("Covariance Ellipse (65%, 95%, 99% Confidence)");
    plot::xlabel("X");
    plot::ylabel("Y");
    plot::pause(0.01);
}


};

} // namespace utils4visualize
