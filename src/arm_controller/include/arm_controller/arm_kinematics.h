#ifndef ARM_CONTROLLER_ARM_KINEMATICS_H
#define ARM_CONTROLLER_ARM_KINEMATICS_H

#include <Eigen/Dense>

#include <map>
#include <string>
#include <vector>

namespace arm_controller {

// 从 URDF 解析 4-DOF 机械臂运动学（不硬编码 DH）。
// 直接按 URDF 的 origin/axis 链式推正解与几何雅可比，连杆长度取自 URDF。
class ArmKinematics {
public:
    ArmKinematics() = default;

    // 从 URDF 文本加载；成功返回 true。
    bool loadFromUrdf(const std::string& urdf_text);

    // 运动学关节数（revolute），不含 fixed 关节。
    int numJoints() const { return static_cast<int>(joint_names_.size()); }

    // 正解：q(numJoints) -> 末端 tool 位置。
    bool forwardKinematics(const Eigen::VectorXd& q, Eigen::Vector3d& ee_pos) const;

    // 几何雅可比（线速度 3xN + 角速度 3xN，输出 6xN）。
    bool jacobian(const Eigen::VectorXd& q, Eigen::MatrixXd& J) const;

    const Eigen::VectorXd& lowerLimits() const { return lower_; }
    const Eigen::VectorXd& upperLimits() const { return upper_; }
    const Eigen::VectorXd& velocityLimits() const { return velocity_; }
    const std::vector<std::string>& jointNames() const { return joint_names_; }

private:
    struct JointInfo {
        std::string name;
        std::string type;
        std::string parent;
        std::string child;
        Eigen::Vector3d origin_xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
        Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        double lower = -3.141592653589793;
        double upper = 3.141592653589793;
        double velocity = 0.0;
    };

    static Eigen::Matrix3d rpyToRotation(const Eigen::Vector3d& rpy);

    std::map<std::string, JointInfo> joints_;          // name -> joint
    std::vector<std::string> revolute_order_;          // 拓扑序（含 fixed）
    std::vector<std::string> joint_names_;             // 运动学关节名
    std::string base_link_ = "base_link";
    Eigen::VectorXd lower_, upper_, velocity_;
};

}  // namespace arm_controller

#endif  // ARM_CONTROLLER_ARM_KINEMATICS_H
