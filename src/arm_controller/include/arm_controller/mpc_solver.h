#ifndef ARM_CONTROLLER_MPC_SOLVER_H
#define ARM_CONTROLLER_MPC_SOLVER_H

#include <array>
#include <vector>
#include <Eigen/Dense>

namespace arm_controller {

class MPCSolver {
public:
    MPCSolver(int n_joints, int horizon, double dt);
    ~MPCSolver() = default;

    // 设置约束
    void setJointLimits(const Eigen::VectorXd& lower,
                        const Eigen::VectorXd& upper);
    void setVelocityLimits(const Eigen::VectorXd& limits);

    // 设置权重
    void setWeights(const Eigen::VectorXd& Q_pos,
                    const Eigen::VectorXd& Q_vel,
                    const Eigen::VectorXd& R);

    // ★ 核心求解函数
    // 输入: 当前关节状态 q, dq; 参考轨迹 ref_q (N步)
    // 输出: 最优控制序列 u_opt (N步关节位置增量)
    bool solve(const Eigen::VectorXd& q_current,
               const Eigen::VectorXd& dq_current,
               const std::vector<Eigen::VectorXd>& ref_trajectory,
               std::vector<Eigen::VectorXd>& u_optimal);

    // 获取预测轨迹（用于可视化）
    const std::vector<Eigen::VectorXd>& getPredictedStates() const;

private:
    int n_joints_;
    int horizon_;
    double dt_;

    Eigen::VectorXd q_min_, q_max_;
    Eigen::VectorXd dq_max_;
    Eigen::VectorXd Q_pos_, Q_vel_, R_;

    std::vector<Eigen::VectorXd> predicted_states_;

    // TODO: 使用OSQP/CasADi/ACADO等求解QP问题
    // 状态空间模型: x_{k+1} = A*x_k + B*u_k
    // x = [q; dq], u = q_delta (位置增量)
    Eigen::MatrixXd A_;  // 系统矩阵
    Eigen::MatrixXd B_;  // 输入矩阵
    void buildSystemMatrices();
};

}  // namespace arm_controller
#endif
