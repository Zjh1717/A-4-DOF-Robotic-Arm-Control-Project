#include "arm_controller/trajectory_handler.h"

#include <algorithm>

namespace arm_controller {

void TrajectoryHandler::setTrajectory(
    const trajectory_msgs::JointTrajectory& traj, double timeout_s) {
    points_.clear();
    num_joints_ = 0;
    end_time_ = 0.0;
    for (const auto& p : traj.points) {
        Sample s;
        s.t = p.time_from_start.toSec();
        s.q = p.positions;
        num_joints_ = std::max(num_joints_, static_cast<int>(s.q.size()));
        points_.push_back(s);
        end_time_ = std::max(end_time_, s.t + timeout_s);
    }
}

bool TrajectoryHandler::sample(double time, Eigen::VectorXd& q_ref) const {
    if (points_.empty() || num_joints_ <= 0) {
        return false;
    }
    // 找到时间区间 [points_[i], points_[i+1]]。
    if (time <= points_.front().t) {
        q_ref = Eigen::Map<const Eigen::VectorXd>(points_.front().q.data(),
                                                  num_joints_);
        return true;
    }
    for (size_t i = 0; i + 1 < points_.size(); ++i) {
        double t0 = points_[i].t;
        double t1 = points_[i + 1].t;
        if (time <= t1) {
            double frac = (t1 > t0) ? (time - t0) / (t1 - t0) : 0.0;
            q_ref = Eigen::VectorXd(num_joints_);
            for (int j = 0; j < num_joints_; ++j) {
                q_ref[j] = points_[i].q[j] +
                           frac * (points_[i + 1].q[j] - points_[i].q[j]);
            }
            return true;
        }
    }
    q_ref = Eigen::Map<const Eigen::VectorXd>(points_.back().q.data(),
                                              num_joints_);
    return true;
}

}  // namespace arm_controller
