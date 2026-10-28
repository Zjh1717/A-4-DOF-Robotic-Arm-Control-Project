#ifndef ARM_CONTROLLER_MPC_SOLVER_H
#define ARM_CONTROLLER_MPC_SOLVER_H

#include <Eigen/Dense>

#include <vector>

namespace arm_controller {

// 关节空间 MPC（自包含投影梯度，不依赖 OSQP/CasADi，纯 Eigen）。
//
// 模型（速度指令）：
//   q_{k+1} = sat(q_k + dt*dq_k, q_min, q_max)   （关节限位硬约束）
//   dq_{k+1} = u_k                              （u = 速度指令 q_dot）
// 状态 x=[q; dq]，控制 u=q_dot（rad/s）。
//
// 代价：Σ ||q_k - q_ref_k||²_Qp + Σ ||dq_k||²_Qv + Σ ||u_k||²_R。
// 速度硬约束：|u_k| <= v_max（投影到盒约束）。
class MPCSolver {
public:
    MPCSolver(int n_joints, int horizon, double dt);
    ~MPCSolver() = default;

    void setJointLimits(const Eigen::VectorXd& lower,
                        const Eigen::VectorXd& upper);
    void setVelocityLimits(const Eigen::VectorXd& limits);
    void setWeights(const Eigen::VectorXd& Q_pos,
                    const Eigen::VectorXd& Q_vel,
                    const Eigen::VectorXd& R);
    void setMaxIterations(int max_iter) { max_iter_ = max_iter; }
    void setStepSize(double lr) { lr_ = lr; }

    // 求解：输入当前状态与参考轨迹（N 步 q_ref），输出速度指令序列（rad/s）。
    bool solve(const Eigen::VectorXd& q_current,
               const Eigen::VectorXd& dq_current,
               const std::vector<Eigen::VectorXd>& ref_trajectory,
               std::vector<Eigen::VectorXd>& u_optimal);

    // 预测的状态序列（q，长度 horizon+1），用于调试/可视化。
    const std::vector<Eigen::VectorXd>& getPredictedStates() const {
        return predicted_states_;
    }

private:
    int n_joints_;
    int horizon_;
    double dt_;
    int max_iter_ = 100;
    double lr_ = 0.05;

    Eigen::VectorXd q_min_, q_max_;
    Eigen::VectorXd dq_max_;
    Eigen::VectorXd Q_pos_, Q_vel_, R_;

    std::vector<Eigen::VectorXd> predicted_states_;

    void rollout(const Eigen::VectorXd& q0, const Eigen::VectorXd& dq0,
                 const std::vector<Eigen::VectorXd>& u,
                 std::vector<Eigen::VectorXd>& q,
                 std::vector<Eigen::VectorXd>& dq) const;
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER_MPC_SOLVER_H
