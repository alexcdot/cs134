#ifndef __KINEMATICS__
#define __KINEMATICS__

#include "common.hpp"

#include "sensor_msgs/JointState.h"
#include "boogaloo/MassChange.h"

#define vec(x,y,z)  (Eigen::Vector3d((x), (y), (z)))
#define Rx(q)       (Eigen::AngleAxisd((q), vec(1.0, 0.0, 0.0)).toRotationMatrix())
#define Ry(q)       (Eigen::AngleAxisd((q), vec(0.0, 1.0, 0.0)).toRotationMatrix())
#define Rz(q)       (Eigen::AngleAxisd((q), vec(0.0, 0.0, 1.0)).toRotationMatrix())

struct FKinResult {
    Vector6d pose;
    Matrix6d jacobian;
    Quaterniond orientation;
};

struct ArmProperties {
    vector<Vector3d> translations;
    vector<Matrix3d> rotations;
    vector<double> zeroes;
    vector<double> gearings;
    vector<vector<double>> grav_sin;
    vector<vector<double>> grav_cos;
};

struct Joints {
    Vector6d pos;
    Vector6d vel;
    Vector6d torque;
};

enum JointID {
    YAW=0,
    SHOULDER=1,
    ELBOW=2,
    WRIST=3,
    TWIST=4,
    GRIPPER=5
};

class Kinematics {
    private:
    ArmProperties ARM_PROP = {
        {
            vec(0, -0.124, 0.081),
            vec(0, -0.051, 0.051),
            vec(-0.606, 0, -0.064),
            vec(0, 0.298, 0),
            vec(0.038, 0, 0.110),
            vec(0, 0, 0.056)
        },
        {Rz(0), Rx(PI/2) * Rz(-PI/2), Rz(0), Rx(PI), Ry(PI/2), Rz(PI)},
        {0, 0, 0, 0, 0, 0},
        {1, 1, 2, 3, 1, -2.32},
        {{0, -14.4, -5.16 / 2, 0.12 / 3, 0, 0}, {0, -18.7, -8.63 / 2, 3.58 / 3, 0, 0}},
        {{0, -0.5, 0.9 / 2, 0 / 3, 0, 0}, {0, -0.5, 0 / 2, 0.0 / 3, 0, 0}}
        //-3.58
    };
    
    static const std::string JOINT_NAMES[];
    static const double ERR;
    FKinResult zero_pos_;
    int mass_status_;

    public:
    Kinematics();
    FKinResult runForwardKinematics(Vector6d joint_vals);
    Vector6d runInverseKinematics(Vector6d target_pose);
    Vector6d getExpectedJointTorques(Vector6d joint_vals);
    sensor_msgs::JointState jointsToJS(Vector6d joint_pos, Vector6d joint_vel, Vector6d joint_torque);
    Joints jsToJoints(sensor_msgs::JointState joints);
    sensor_msgs::JointState toHebi(sensor_msgs::JointState normal_joints);
    sensor_msgs::JointState fromHebi(sensor_msgs::JointState hebi_joints);
    void updateMassStatus(const boogaloo::MassChange::ConstPtr& msg);
};

#endif