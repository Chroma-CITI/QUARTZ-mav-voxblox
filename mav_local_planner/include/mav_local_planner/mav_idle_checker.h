#pragma once

#include <deque>

#include <nav_msgs/Odometry.h>

namespace mav_planning {

class IdleChecker {
    public:
    IdleChecker();
    IdleChecker(const int queueSize, const double minSpeedTreshold);
    void AddOdometry(const nav_msgs::Odometry odometry);
    double GetMeanLinearSpeed();
    bool IsIdle();
    
    private:
    int queueSize_;
    double minSpeedTreshold_; // in m/s
    std::deque<nav_msgs::Odometry> odometries_;
};

}  // namespace mav_planning