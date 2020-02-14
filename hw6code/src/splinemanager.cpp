#include "splinemanager.hpp"

SplineManager::SplineManager() {
    a_ = b_ = c_ = d_ = 0;
}

void SplineManager::setSpline(double goal_pos, double start_pos, double start_vel, ros::Time t_start) {
    double avg_speed = 1.5;
    double min_time = 1.0;
    double t_move = max(min_time, abs(goal_pos - start_pos) / avg_speed);

    t0_ = t_start;
    tf_ = t_start + ros::Duration(t_move);

    a_ = start_pos;
    b_ = start_vel;
    c_ = ( 3.0 * (goal_pos - start_pos) / t_move + 2.0 * start_vel) / t_move;
    d_ = (-2.0 * (goal_pos - start_pos) / t_move - 3.0 * start_vel) / (t_move*t_move);
}

SplinePoint SplineManager::getPoint(ros::Time time) {
    double t;
    if (time <= t0_) {
        t = 0;
    }
    else if (time >= tf_) {
        t = (tf_ - t0_).toSec();
    }
    else {
        t = (time - t0_).toSec();
    }

    SplinePoint point;
    point.pos = a_ + b_*t + c_*t*t + d_*t*t*t;
    point.vel = b_ + 2.0*c_*t + 3.0*d_*t*t;
    return point;
}