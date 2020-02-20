#ifndef __SPLINE_MANAGER__
#define __SPLINE_MANAGER__

#include "common.hpp"

struct SplinePoint {
    double pos;
    double vel;
};

class SplineManager {
    private:
    double a_, b_, c_, d_; // Spline parameters
    ros::Time t0_, tf_; // Start and end time

    public:
    SplineManager();
    void setSpline(double goal_pos, double start_pos, double start_vel, ros::Time t_start);
    SplinePoint getPoint(ros::Time time);
};

#endif