#ifndef __SPLINE_MANAGER__
#define __SPLINE_MANAGER__

#include "common.hpp"

struct SplinePoint {
    double pos;
    double vel;
    bool at_end;
};

class SplineManager {
    private:
    double a_, b_, c_, d_; // Spline parameters
    ros::Time t0_, tf_; // Start and end time
    

    public:
    SplineManager();
    void setSpline(double start_pos, double start_vel, double goal_pos, double goal_vel, ros::Time t_start, ros::Duration t_move);
    SplinePoint getPoint(ros::Time time);
    static double getJointMoveTime(Vector6d start, Vector6d end);
    static double getPoseMoveTime(Vector6d start, Vector6d end);
};

#endif