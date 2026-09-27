#include "mav_local_planner/mav_idle_checker.h"

namespace mav_planning {

    IdleChecker::IdleChecker() {
        queueSize_ = 10;
        minSpeedTreshold_ = 0.1;
    }

    IdleChecker::IdleChecker(const int queueSize, const double minSpeedTreshold) {
        queueSize_ = queueSize;
        minSpeedTreshold_ = minSpeedTreshold;
    }

    void IdleChecker::AddOdometry(const nav_msgs::Odometry odometry) {
        if (odometries_.size() >= queueSize_)
            odometries_.pop_front();
        odometries_.push_back(odometry);
    }

    double IdleChecker::GetMeanLinearSpeed() {
        if (! odometries_.size())
            return 0.0;

        double sum = 0.0;
        for (const auto& odom: odometries_) {
            double linearVelocity = sqrt(
                odom.twist.twist.linear.x * odom.twist.twist.linear.x +
                odom.twist.twist.linear.y * odom.twist.twist.linear.y +
                odom.twist.twist.linear.z * odom.twist.twist.linear.z
            );
            sum += linearVelocity;
        }
        double meanVelocity = sum / odometries_.size();
        return meanVelocity;
    }

    bool IdleChecker::IsIdle() {
        double meanVelocity = GetMeanLinearSpeed();
        if (abs(meanVelocity) < minSpeedTreshold_) {
            return true;
        } else {
            return false;
        }
    }

}  // namespace mav_planning
