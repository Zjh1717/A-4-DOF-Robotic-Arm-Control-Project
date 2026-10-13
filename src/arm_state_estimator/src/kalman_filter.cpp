#include "arm_state_estimator/kalman_filter.h"

namespace arm_state_estimator {

JointKalmanFilter::JointKalmanFilter(
    double dt, double q_pos, double q_vel, double r)
    : dt_(dt) {
    x_ = Eigen::Vector2d::Zero();
    P_ = Eigen::Matrix2d::Identity() * 0.1;

    // 状态转移矩阵 F = [1 dt; 0 1]
    F_ << 1.0, dt_,
          0.0, 1.0;

    // 输入矩阵 B = [0.5*dt^2; dt]
    B_ << 0.5 * dt_ * dt_,
          dt_;

    // 过程噪声
    Q_ << q_pos, 0.0,
          0.0, q_vel;

    // 观测矩阵 H = [1, 0]
    H_ << 1.0, 0.0;

    R_ = r;
}

void JointKalmanFilter::predict(double acceleration_cmd) {
    // 状态预测: x = F*x + B*u
    x_ = F_ * x_ + B_ * acceleration_cmd;

    // 协方差预测: P = F*P*F^T + Q
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void JointKalmanFilter::update(double position_measurement) {
    // 标量 S = H*P*H^T + R
    double S = (H_ * P_ * H_.transpose())(0, 0) + R_;

    // 卡尔曼增益 K = P*H^T / S    (2x1)
    Eigen::Matrix<double, 2, 1> K = P_ * H_.transpose() / S;

    // 新息（标量）
    double innovation = position_measurement - (H_ * x_)(0);

    // 状态更新
    x_ = x_ + K * innovation;

    // 协方差更新
    P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
}

void JointKalmanFilter::reset(double pos, double vel) {
    x_ << pos, vel;
    P_ = Eigen::Matrix2d::Identity() * 0.1;
}

}  // namespace arm_state_estimator
