#include "splinemanager.hpp"

SplineManager::SplineManager() {
    a_ = b_ = c_ = d_ = 0;
}

void SplineManager::setSpline(double start_pos, double start_vel, double goal_pos, double goal_vel, ros::Time t_start, ros::Duration t_move) {
    t0_ = t_start;
    tf_ = t_start + t_move;

    // Rename vars so I can just copy-paste from mathematica
    double sp = start_pos;
    double sv = start_vel;
    double fp = goal_pos;
    double fv = goal_vel;
    double tm = t_move.toSec();

    a_ = sp;
    b_ = sv;
    c_ = -(-3*fp + 3*sp + fv*tm + 2*sv*tm) / (tm*tm);
    d_ = -(2*fp - 2*sp - fv*tm - sv*tm) / (tm*tm*tm);
}

SplinePoint SplineManager::getPoint(ros::Time time) {
    double t;
    SplinePoint point;
    if (time <= t0_) {
        t = 0;
        point.at_end = false;
    }
    else if (time >= tf_) {
        t = (tf_ - t0_).toSec();
        point.at_end = true;
    }
    else {
        t = (time - t0_).toSec();
        point.at_end = false;
    }

    
    point.pos = a_ + b_*t + c_*t*t + d_*t*t*t;
    point.vel = b_ + 2.0*c_*t + 3.0*d_*t*t;
    return point;
}