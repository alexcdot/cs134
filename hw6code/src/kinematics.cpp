#include "kinematics.hpp"

Kinematics::Kinematics() { }

FKinResult Kinematics::runForwardKinematics(Vector6d joint_vals) {
    assert(ARM_PROP.translations.size() == 6);
    assert(ARM_PROP.rotations.size() == 6);

    Matrix3d total_rot = Matrix3d::Identity();
    Vector3d total_pos = Vector3d::Zero();

    vector<Vector3d> joint_poses = vector<Vector3d>();
    vector<Matrix3d> joint_oris = vector<Matrix3d>();

    // Run forward kinematics, summing total position through 5 joints
    for(int i = 0; i < 5; i++) {
        total_pos += total_rot * ARM_PROP.translations[i];
        total_rot *= ARM_PROP.rotations[i];
        total_rot *= Rz(joint_vals[i]);

        // Log position of each joint
        joint_poses.push_back(total_pos);
        joint_oris.push_back(total_rot);
    }
    // From final motor to tip
    total_pos += total_rot * ARM_PROP.translations[5];
    // The last rotation comes from the gripeer, so multiplying the joint_val is unnecessary
    total_rot *= ARM_PROP.rotations[5];
    joint_poses.push_back(total_pos);
    joint_oris.push_back(total_rot);

    FKinResult res;

    // Tip pose is stored at total pos after fkin
    res.pose(0) = total_pos(0);
    res.pose(1) = total_pos(1);
    res.pose(2) = total_pos(2);

    // Calculate tip orientation vector
    Vector3d tip_ori = (joint_poses[GRIPPER] - joint_poses[TWIST]).normalized();
    // X value of orientation is -z axis of orientation
    double trig_x = -tip_ori(2);
    // Calculate tip position vector along the table plane
    Vector3d tip_vec = (joint_poses[WRIST] - joint_poses[ELBOW]);
    tip_vec(2) = 0;
    tip_vec = tip_vec.normalized();
    // X value is dot of position and orientation vector
    double trig_y = tip_vec.dot(tip_ori);

    // Tip orientation angle
    res.pose(3) = atan2(trig_y, trig_x);

    // Twist is negative the twist plus orientation of tip (scaled by how far down arm is pointing)
    res.pose(4) = -joint_vals[TWIST] + trig_x * atan2(tip_vec(1), tip_vec(0));

    // Grip is just grip :)
    res.pose(5) = joint_vals[GRIPPER];

    // Store tip orientation as quaternion for rviz messages
    res.orientation = Quaterniond(total_rot);

    res.jacobian = Matrix6d::Zero();
    // Transformation from position to derivative in X-Y coordinate frame
    Matrix3d deriv_transform;
    deriv_transform << 0, -1, 0,
                       1, 0,  0,
                       0, 0,  0;
    for (int i = 0; i < 4; i++) {
        // Calculate positional derivative
        Vector3d to_tip = joint_poses[GRIPPER] - joint_poses[i];
        Vector3d axis_frame = joint_oris[i].inverse() * to_tip;
        Vector3d deriv = joint_oris[i] * deriv_transform * axis_frame;
        res.jacobian.block<3, 1>(0, i) = deriv;
    }
    // Define claw orientation jacobian
    res.jacobian.block<1, 5>(3, 0) << 0, 1, 1, 1, 0;
    // Define claw twist jacobian
    double pitch_derivative = -sin(res.pose(3)) * atan2(tip_vec(1), tip_vec(0));
    res.jacobian.block<1, 5>(4, 0) << cos(res.pose(3)), 
                                    pitch_derivative,
                                    pitch_derivative,
                                    pitch_derivative,
                                    -1;
    // Define gripper jacobian
    res.jacobian(5, 5) = 1;
    
    return res;
}

Vector6d Kinematics::runInverseKinematics(Vector6d target_pose, Vector6d guess) {
    Vector6d current = Vector6d(guess);
    FKinResult fkin = runForwardKinematics(current);
    Vector6d delta = target_pose - fkin.pose;
    int i = 0;
    while (delta.norm() > 0.001 && i < 30) {
        Vector6d poseDelta = fkin.jacobian.inverse() * delta;

        fkin = runForwardKinematics(current + poseDelta);

        Vector6d newDelta = target_pose - fkin.pose;

        while (newDelta.norm() > delta.norm()) {
            poseDelta /= 2;
            fkin = runForwardKinematics(current + poseDelta);
            newDelta = target_pose - fkin.pose;
        }

        current += poseDelta;

        while (delta.norm() < newDelta.norm()) {
            cout << "uhh" << endl;
            cout << fkin.jacobian << endl;
            cout << fkin.jacobian.inverse() << endl;
            cout << delta << endl<<endl;
            cout << fkin.jacobian.inverse() * delta << endl;
            cout << "---" << endl;
            ros::Duration(0.1).sleep();
        }

        delta = newDelta;
        i++;
    }
    cout << delta.norm() << endl;
    cout << fkin.jacobian.inverse() << endl;
    cout << current << endl << endl;
    cout << fkin.pose << endl;
    return current;
}