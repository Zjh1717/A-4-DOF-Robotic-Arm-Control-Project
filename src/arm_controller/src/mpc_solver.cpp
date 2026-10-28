#include "arm_controller/mpc_solver.h"

#include <algorithm>
#include <cmath>

namespace arm_controller {

MPCSolver::MPCSolver(int n_joints, int horizon, double dt)
    : n_joints_(n_joints), horizon_(horizon), dt_(dt) {
    q_min_ = Eigen::VectorXd::Constant(n_joints_, -3.14);
    q_max_ = Eigen::VectorXd::Constant(n_joints_, 3.14);
    dq_max_ = Eigen::VectorXd::Constant(n_joints_, 1.0);
    Q_pos_ = Eigen::VectorXd::Constant(n_joints_, 100.0);
    Q_vel_ = Eigen::VectorXd::Constant(n_joints_, 1.0);
    R_ = Eigen::VectorXd::Constant(n_joints_, 0.01);
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

void MPCSolver::rollout(const Eigen::VectorXd& q0, const Eigen::VectorXd& dq0,
                        const std::vector<Eigen::VectorXd>& u,
                        std::vector<Eigen::VectorXd>& q,
                        std::vector<Eigen::VectorXd>& dq) const {
    q.resize(horizon_ + 1);
    dq.resize(horizon_ + 1);
    q[0] = q0;
    dq[0] = dq0;
    for (int k = 0; k < horizon_; ++k) {
        Eigen::VectorXd q_next = q[k] + dt_ * dq[k];
        // 关节限位硬约束（对预测状态饱和处理）。
        q_next = q_next.cwiseMax(q_min_).cwiseMin(q_max_);
        q[k + 1] = q_next;
        dq[k + 1] = u[k];
    }
}

bool MPCSolver::solve(const Eigen::VectorXd& q_current,
                      const Eigen::VectorXd& dq_current,
                      const std::vector<Eigen::VectorXd>& ref_trajectory,
                      std::vector<Eigen::VectorXd>& u_optimal) {
    if (q_current.size() != n_joints_ || dq_current.size() != n_joints_) {
        return false;
    }

    // 参考轨迹补齐/截断到 horizon 步。
    std::vector<Eigen::VectorXd> ref(horizon_ + 1);
    for (int k = 0; k <= horizon_; ++k) {
        if (k < static_cast<int>(ref_trajectory.size())) {
            ref[k] = ref_trajectory[k];
        } else if (!ref_trajectory.empty()) {
            ref[k] = ref_trajectory.back();
        } else {
            ref[k] = q_current;  // 无参考则保持当前位置
        }
    }

    // 初始化控制序列为 0。
    std::vector<Eigen::VectorXd> u(horizon_,
                                   Eigen::VectorXd::Zero(n_joints_));
    std::vector<Eigen::VectorXd> q, dq;
    std::vector<Eigen::VectorXd> grad(horizon_,
                                      Eigen::VectorXd::Zero(n_joints_));

    for (int iter = 0; iter < max_iter_; ++iter) {
        rollout(q_current, dq_current, u, q, dq);

        // 伴随法求梯度（忽略饱和段的子梯度，按线性传播处理）。
        Eigen::VectorXd lam_q = Eigen::VectorXd::Zero(n_joints_);
        Eigen::VectorXd lam_dq = Eigen::VectorXd::Zero(n_joints_);
        for (int k = horizon_ - 1; k >= 0; --k) {
            Eigen::VectorXd cq =
                2.0 * Q_pos_.cwiseProduct(q[k + 1] - ref[k + 1]);
            Eigen::VectorXd cv = 2.0 * Q_vel_.cwiseProduct(dq[k + 1]);
            lam_q += cq;
            lam_dq += cv;
            grad[k] = 2.0 * R_.cwiseProduct(u[k]) + lam_dq;
            // 传播回状态 k：q_{k+1}=sat(q_k+dt*dq_k)，dq_{k+1}=u_k。
            lam_dq = lam_q * dt_;
            // lam_q 不变（∂q_{k+1}/∂q_k = I）。
        }

        // 投影梯度下降 + 速度盒约束投影。
        for (int k = 0; k < horizon_; ++k) {
            u[k] -= lr_ * grad[k];
            u[k] = u[k].cwiseMax(-dq_max_).cwiseMin(dq_max_);
        }
    }

    rollout(q_current, dq_current, u, q, dq);
    u_optimal = u;
    predicted_states_ = q;
    return true;
}

}  // namespace arm_controller
