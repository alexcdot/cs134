#ifndef __KINEMATICS__
#define __KINEMATICS__

#include "common.hpp"

#include "sensor_msgs/JointState.h"

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
            vec(0, 0.1, 0),
            vec(0, 0, 0.1),
            vec(-0.6, 0, 0),
            vec(0, 0.3, 0),
            vec(0.15, 0, 0.1),
            vec(0.0, 0.0, 0.1)
        },
        {Rz(0), Ry(PI/2), Rz(0), Rz(0), Ry(PI/2), Rz(0)},
        {0, 0, 0, 0, 0, 0},
        {1, 1, 2, 3, 1, 0.5}
    };
    
    static const std::string JOINT_NAMES[];

    public:
    Kinematics();
    FKinResult runForwardKinematics(Vector6d joint_vals);
    Vector6d runInverseKinematics(Vector6d target_pose, Vector6d guess);
    sensor_msgs::JointState jointsToJS(Vector6d joint_pos, Vector6d joint_vel);
    Joints jsToJoints(sensor_msgs::JointState joints);
    sensor_msgs::JointState toHebi(sensor_msgs::JointState normal_joints);
    sensor_msgs::JointState fromHebi(sensor_msgs::JointState hebi_joints);

};

#endif