#ifndef ARM_STATE_ESTIMATOR_KALMAN_FILTER_H
#define ARM_STATE_ESTIMATOR_KALMAN_FILTER_H

#include <Eigen/Dense>

namespace arm_state_estimator {

class JointKalmanFilter {
public:
    JointKalmanFilter(double dt,
                      double process_noise_pos = 1e-4,
                      double process_noise_vel = 1e-3,
                      double measurement_noise = 1e-3);

    // 预测步骤
    void predict(double acceleration_cmd);

    // 更新步骤（观测到位置）
    void update(double position_measurement);

    // 获取估计值
    double getPosition() const { return x_(0); }
    double getVelocity() const { return x_(1); }

    // 重置
    void reset(double pos = 0.0, double vel = 0.0);

private:
    Eigen::Vector2d x_;       // 状态 [q, dq]
    Eigen::Matrix2d P_;       // 协方差
    Eigen::Matrix2d F_;       // 状态转移矩阵
    Eigen::Vector2d B_;       // 输入矩阵
    Eigen::Matrix2d Q_;       // 过程噪声
    Eigen::Matrix<double, 1, 2> H_;  // 观测矩阵 [1, 0]
    double R_;                // 测量噪声
    double dt_;
};

}  // namespace arm_state_estimator
#endif
