#ifndef ARM_CONTROLLER_ARM_IK_H
#define ARM_CONTROLLER_ARM_IK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace arm_controller {

// 4-DOF 机械臂解析逆解（底座 yaw + 平面 3R）。
// 工作在 URDF 空间：关节角为弧度、坐标以 base_link 为参考系、零位=连杆水平向前。
//
// 姿态可达性：本臂只有 4 个关节，只能实现“绕 Z 的 yaw + 绕 Y 的 pitch”两个姿态
// 自由度；绕工具轴的自转（roll）不可达，会自动投影忽略。
//
// 几何参数来自 src/arm_description/urdf/arm_4dof.urdf：
//   L1=L2=L3=0.06m；肩关节(joint2)世界高度=0.025+0.04=0.065m。
class ArmIK {
public:
    ArmIK();

    // 设置关节限位（弧度，4 维），来自 URDF 的 lower/upper。
    void setJointLimits(const Eigen::VectorXd& lower,
                        const Eigen::VectorXd& upper);

    // 求解末端 tool 位姿 -> 4 关节角。
    //   pos:       base_link 系末端位置 [m]
    //   quat:      期望朝向四元数（Eigen 约定 w,x,y,z；ROS 的四元数是 x,y,z,w，
    //              调用方需自行换算）。若 quat 接近零范数，视为“未指定姿态”，
    //              冗余自由度用“保持当前工具朝向”消掉。
    //   current_q: 当前 4 关节角 [rad]，用于奇异/多解时选近解。
    //   q_out:     输出 4 关节角 [rad]。
    // 返回 false 表示目标不可达或超限，调用方应保持当前位姿并提示。
    bool solve(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
               const Eigen::VectorXd& current_q, Eigen::VectorXd& q_out);

    // 几何常量（公开便于测试与文档核对）。
    static constexpr double kLinkLen = 0.06;
    static constexpr double kShoulderHeight = 0.065;

private:
    Eigen::VectorXd lower_;
    Eigen::VectorXd upper_;
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER_ARM_IK_H
