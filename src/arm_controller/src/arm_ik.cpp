#include "arm_controller/arm_ik.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace arm_controller {

namespace {

double clampd(double v, double lo, double hi) {
    return std::min(std::max(v, lo), hi);
}

// 候选解的关节限位是否全部满足。
bool inLimits(const Eigen::VectorXd& q, const Eigen::VectorXd& lower,
              const Eigen::VectorXd& upper) {
    for (int i = 0; i < q.size(); ++i) {
        if (q[i] < lower[i] - 1e-9 || q[i] > upper[i] + 1e-9) {
            return false;
        }
    }
    return true;
}

}  // namespace

ArmIK::ArmIK() {
    lower_ = Eigen::VectorXd::Constant(4, -3.141592653589793);
    upper_ = Eigen::VectorXd::Constant(4, 3.141592653589793);
}

void ArmIK::setJointLimits(const Eigen::VectorXd& lower,
                           const Eigen::VectorXd& upper) {
    lower_ = lower;
    upper_ = upper;
}

bool ArmIK::solve(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
                  const Eigen::VectorXd& current_q, Eigen::VectorXd& q_out) {
    if (current_q.size() != 4 || lower_.size() != 4 || upper_.size() != 4) {
        return false;
    }
    q_out = current_q;  // 失败时保持当前位姿

    const double x = pos.x();
    const double y = pos.y();
    const double z = pos.z();
    const double R = std::sqrt(x * x + y * y);

    // 1) 底座 yaw（绕 Z）。目标正上方（x=y=0）为奇异，保持当前 yaw。
    const double q1 = (R < 1e-6) ? current_q[0] : std::atan2(y, x);

    // 2) 工具 pitch φ（绕 Y，径向平面内）。
    //    URDF 工具 X 轴方向在径向平面内为 (cosφ, 0, -sinφ)。
    double phi;
    if (quat.squaredNorm() < 1e-8) {
        // 未指定姿态：保持当前工具朝向。
        phi = current_q[1] + current_q[2] + current_q[3];
    } else {
        const Eigen::Quaterniond qn = quat.normalized();
        const Eigen::Vector3d d = qn * Eigen::Vector3d::UnitX();
        const double erx = std::cos(q1);
        const double ery = std::sin(q1);
        const double d_r = d.x() * erx + d.y() * ery;  // 投影到径向方向
        const double d_z = d.z();
        phi = std::atan2(-d_z, d_r);
    }

    // 3) 腕点相对肩关节的位置。
    //    tool = wrist + L3*(cosφ, 0, -sinφ)
    const double wrist_R = R - kLinkLen * std::cos(phi);
    const double wrist_Z = (z - kShoulderHeight) + kLinkLen * std::sin(phi);

    // 4) 平面 2 连杆求解（把 Z 取反为 Z'，套用标准 2R 公式）。
    const double wy = -wrist_Z;  // Z' = -Z，对应标准公式里的 py
    const double rw2 = wrist_R * wrist_R + wy * wy;
    const double c3 =
        (rw2 - 2.0 * kLinkLen * kLinkLen) / (2.0 * kLinkLen * kLinkLen);
    if (c3 < -1.0 || c3 > 1.0) {
        return false;  // 超出工作空间
    }

    const double alpha = std::atan2(wy, wrist_R);
    const double q3_abs = std::acos(clampd(c3, -1.0, 1.0));

    // 两种肘构型（肘上/肘下），选离当前构型最近且满足限位的解。
    struct Candidate {
        Eigen::VectorXd q;
    };
    std::vector<Candidate> cands;
    for (int sign = -1; sign <= 1; sign += 2) {
        const double q3 = sign * q3_abs;
        const double q2 =
            alpha - std::atan2(kLinkLen * std::sin(q3),
                               kLinkLen + kLinkLen * std::cos(q3));
        const double q4 = phi - q2 - q3;
        Eigen::VectorXd q(4);
        q << q1, q2, q3, q4;
        cands.push_back({q});
    }

    const Candidate* best = nullptr;
    double best_dist = 1e18;
    for (const auto& c : cands) {
        if (!inLimits(c.q, lower_, upper_)) {
            continue;
        }
        const double dist = (c.q - current_q).squaredNorm();
        if (dist < best_dist) {
            best_dist = dist;
            best = &c;
        }
    }
    if (best == nullptr) {
        return false;  // 无满足限位的解
    }
    q_out = best->q;
    return true;
}

}  // namespace arm_controller
