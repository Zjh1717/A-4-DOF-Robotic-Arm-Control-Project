#include "arm_controller/arm_kinematics.h"

#include <cmath>
#include <sstream>

namespace arm_controller {

namespace {

// 解析 "a b c" -> Vector3d。
Eigen::Vector3d parseVec3(const std::string& s, const Eigen::Vector3d& fallback) {
    std::istringstream iss(s);
    double a = fallback[0], b = fallback[1], c = fallback[2];
    if (!(iss >> a >> b >> c)) {
        return fallback;
    }
    return Eigen::Vector3d(a, b, c);
}

double parseDouble(const std::string& s, double fallback) {
    std::istringstream iss(s);
    double v = fallback;
    if (!(iss >> v)) {
        return fallback;
    }
    return v;
}

// 从形如 attr="value" 的片段中提取属性值。
std::string extractAttr(const std::string& s, const std::string& attr) {
    size_t p = s.find(attr + "=\"");
    if (p == std::string::npos) {
        return "";
    }
    p += attr.size() + 2;
    size_t q = s.find('"', p);
    if (q == std::string::npos) {
        return "";
    }
    return s.substr(p, q - p);
}

}  // namespace

Eigen::Matrix3d ArmKinematics::rpyToRotation(const Eigen::Vector3d& rpy) {
    // URDF rpy 为固定轴 X-Y-Z（roll-pitch-yaw）：R = Rz(yaw) * Ry(pitch) * Rx(roll)。
    double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
    double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
    double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
    Eigen::Matrix3d R;
    R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
         sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
         -sp, cp * sr, cp * cr;
    return R;
}

bool ArmKinematics::loadFromUrdf(const std::string& urdf_text) {
    joints_.clear();
    revolute_order_.clear();
    joint_names_.clear();

    // 手工解析 <joint ...> ... </joint>（避免 libstdc++ 对某些正则的兼容问题）。
    size_t pos = 0;
    while ((pos = urdf_text.find("<joint ", pos)) != std::string::npos) {
        size_t tag_end = urdf_text.find('>', pos);
        if (tag_end == std::string::npos) {
            break;
        }
        std::string tag = urdf_text.substr(pos, tag_end - pos + 1);
        std::string name = extractAttr(tag, "name");
        std::string type = extractAttr(tag, "type");
        size_t body_end = urdf_text.find("</joint>", tag_end);
        if (body_end == std::string::npos) {
            break;
        }
        std::string body = urdf_text.substr(tag_end + 1, body_end - tag_end - 1);

        JointInfo info;
        info.name = name;
        info.type = type;

        size_t p = body.find("<parent");
        if (p != std::string::npos) {
            size_t e = body.find('>', p);
            info.parent = extractAttr(body.substr(p, e - p + 1), "link");
        }
        p = body.find("<child");
        if (p != std::string::npos) {
            size_t e = body.find('>', p);
            info.child = extractAttr(body.substr(p, e - p + 1), "link");
        }
        p = body.find("<origin");
        if (p != std::string::npos) {
            size_t e = body.find('>', p);
            std::string otag = body.substr(p, e - p + 1);
            std::string xyz = extractAttr(otag, "xyz");
            std::string rpy = extractAttr(otag, "rpy");
            info.origin_xyz = parseVec3(xyz, Eigen::Vector3d::Zero());
            if (!rpy.empty()) {
                info.rpy = parseVec3(rpy, Eigen::Vector3d::Zero());
            }
        }
        p = body.find("<axis");
        if (p != std::string::npos) {
            size_t e = body.find('>', p);
            std::string atag = body.substr(p, e - p + 1);
            info.axis = parseVec3(extractAttr(atag, "xyz"),
                                  Eigen::Vector3d::UnitZ()).normalized();
        }
        p = body.find("<limit");
        if (p != std::string::npos) {
            size_t e = body.find('>', p);
            std::string ltag = body.substr(p, e - p + 1);
            std::string lo = extractAttr(ltag, "lower");
            std::string up = extractAttr(ltag, "upper");
            std::string vel = extractAttr(ltag, "velocity");
            if (!lo.empty()) info.lower = parseDouble(lo, info.lower);
            if (!up.empty()) info.upper = parseDouble(up, info.upper);
            if (!vel.empty()) info.velocity = parseDouble(vel, 0.0);
        }

        joints_[info.name] = info;
        pos = body_end + std::string("</joint>").size();
    }

    // 从 base_link 沿 parent->child 建立拓扑序（含 fixed）。
    std::string current = base_link_;
    std::map<std::string, JointInfo> remaining = joints_;
    while (!remaining.empty()) {
        bool found = false;
        for (auto it = remaining.begin(); it != remaining.end(); ++it) {
            if (it->second.parent == current) {
                revolute_order_.push_back(it->second.name);
                if (it->second.type == "revolute") {
                    joint_names_.push_back(it->second.name);
                }
                current = it->second.child;
                remaining.erase(it);
                found = true;
                break;
            }
        }
        if (!found) {
            break;  // 无法继续（避免死循环）
        }
    }

    const int n = numJoints();
    if (n <= 0) {
        return false;
    }
    lower_ = Eigen::VectorXd(n);
    upper_ = Eigen::VectorXd(n);
    velocity_ = Eigen::VectorXd(n);
    int idx = 0;
    for (const auto& name : joint_names_) {
        const JointInfo& j = joints_[name];
        lower_[idx] = j.lower;
        upper_[idx] = j.upper;
        velocity_[idx] = j.velocity;
        ++idx;
    }
    return true;
}

bool ArmKinematics::forwardKinematics(const Eigen::VectorXd& q,
                                      Eigen::Vector3d& ee_pos) const {
    if (q.size() != numJoints()) {
        return false;
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    int qi = 0;
    for (const auto& name : revolute_order_) {
        const JointInfo& j = joints_.at(name);
        Eigen::Matrix4d Tj = Eigen::Matrix4d::Identity();
        Tj.block<3, 3>(0, 0) = rpyToRotation(j.rpy);
        Tj.block<3, 1>(0, 3) = j.origin_xyz;
        T = T * Tj;
        if (j.type == "revolute") {
            double angle = q[qi++];
            Eigen::Matrix4d Rj = Eigen::Matrix4d::Identity();
            const Eigen::Vector3d& a = j.axis;
            Eigen::Matrix3d K;
            K << 0, -a[2], a[1],
                 a[2], 0, -a[0],
                 -a[1], a[0], 0;
            Rj.block<3, 3>(0, 0) =
                Eigen::Matrix3d::Identity() + std::sin(angle) * K +
                (1.0 - std::cos(angle)) * (K * K);
            T = T * Rj;
        }
    }
    ee_pos = T.block<3, 1>(0, 3);
    return true;
}

bool ArmKinematics::jacobian(const Eigen::VectorXd& q, Eigen::MatrixXd& J) const {
    if (q.size() != numJoints()) {
        return false;
    }
    Eigen::Vector3d ee_pos;
    if (!forwardKinematics(q, ee_pos)) {
        return false;
    }

    // 遍历链，记录每个 revolute 关节的世界位姿。
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    std::vector<Eigen::Vector3d> origins;
    std::vector<Eigen::Vector3d> axes;
    for (const auto& name : revolute_order_) {
        const JointInfo& j = joints_.at(name);
        Eigen::Matrix4d Tj = Eigen::Matrix4d::Identity();
        Tj.block<3, 3>(0, 0) = rpyToRotation(j.rpy);
        Tj.block<3, 1>(0, 3) = j.origin_xyz;
        T = T * Tj;
        if (j.type == "revolute") {
            origins.push_back(T.block<3, 1>(0, 3));
            axes.push_back((T.block<3, 3>(0, 0) * j.axis).normalized());
        }
    }

    const int n = numJoints();
    J = Eigen::MatrixXd::Zero(6, n);
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d linear = axes[i].cross(ee_pos - origins[i]);
        J.block<3, 1>(0, i) = linear;
        J.block<3, 1>(3, i) = axes[i];
    }
    return true;
}

}  // namespace arm_controller
