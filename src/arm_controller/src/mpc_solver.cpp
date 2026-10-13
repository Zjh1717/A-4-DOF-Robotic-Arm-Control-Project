#include "arm_controller/mpc_solver.h"

namespace arm_controller {

MPCSolver::MPCSolver(int n_joints, int horizon, double dt)
    : n_joints_(n_joints), horizon_(horizon), dt_(dt) {
    q_min_ = Eigen::VectorXd::Constant(n_joints_, -3.14);
    q_max_ = Eigen::VectorXd::Constant(n_joints_, 3.14);
    dq_max_ = Eigen::VectorXd::Constant(n_joints_, 1.0);
    Q_pos_ = Eigen::VectorXd::Constant(n_joints_, 100.0);
    Q_vel_ = Eigen::VectorXd::Constant(n_joints_, 1.0);
    R_ = Eigen::VectorXd::Constant(n_joints_, 0.01);
    buildSystemMatrices();
}

void MPCSolver::buildSystemMatrices() {
    // 简化的双积分模型: q_{k+1} = q_k + dt*dq_k + 0.5*dt^2*u_k
    //                   dq_{k+1} = dq_k + dt*u_k
    // 状态向量 x = [q(0..3), dq(0..3)]^T  (8维)
    int nx = 2 * n_joints_;
    A_ = Eigen::MatrixXd::Identity(nx, nx);
    B_ = Eigen::MatrixXd::Zero(nx, n_joints_);

    for (int i = 0; i < n_joints_; ++i) {
        // q_{i} 行
        A_(i, n_joints_ + i) = dt_;
        B_(i, i) = 0.5 * dt_ * dt_;
        // dq_{i} 行
        A_(n_joints_ + i, n_joints_ + i) = 1.0;
        B_(n_joints_ + i, i) = dt_;
    }
}

bool MPCSolver::solve(
    const Eigen::VectorXd& q_current,
    const Eigen::VectorXd& dq_current,
    const std::vector<Eigen::VectorXd>& ref_trajectory,
    std::vector<Eigen::VectorXd>& u_optimal) {

    // TODO: 实现MPC优化求解
    // 1. 构建QP问题:
    //    min Σ_{k=0}^{N-1} [ (x_k - x_ref_k)^T Q (x_k - x_ref_k)
    //                        + u_k^T R u_k ]
    //    s.t. x_{k+1} = A x_k + B u_k
    //         q_min <= q_k <= q_max
    //         |dq_k| <= dq_max
    // 2. 使用OSQP求解
    // 3. 取最优序列的第一个控制量作为输出

    u_optimal.resize(horizon_);
    for (auto& u : u_optimal) {
        u = Eigen::VectorXd::Zero(n_joints_);
    }
    return true;
}

void MPCSolver::setJointLimits(const Eigen::VectorXd& lower,
                                const Eigen::VectorXd& upper) {
    q_min_ = lower;
    q_max_ = upper;
}

void MPCSolver::setVelocityLimits(const Eigen::VectorXd& limits) {
    dq_max_ = limits;
}

void MPCSolver::setWeights(const Eigen::VectorXd& Q_pos,
                            const Eigen::VectorXd& Q_vel,
                            const Eigen::VectorXd& R) {
    Q_pos_ = Q_pos;
    Q_vel_ = Q_vel;
    R_ = R;
}

}  // namespace arm_controller
