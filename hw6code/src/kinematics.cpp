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
    total_rot *= ARM_PROP.rotations[5];
    joint_poses.push_back(total_pos);
    joint_oris.push_back(total_rot);

    FKinResult res;

    // Tip pose is stored at total pos after fkin
    res.pose(0) = total_pos(0);
    res.pose(1) = total_pos(1);
    res.pose(2) = total_pos(2);

    // Calculate tip orientation vector
    Vector3d tip_ori = (joint_oris[GRIPPER] * Vector3d::UnitZ()).normalized();
    
    // TODO this makes an assumption about the zero position
    double yaw = joint_vals(YAW) + PI/2;
    
    Vector2d yaw_vec;
    yaw_vec << cos(yaw), sin(yaw);

    // X value of gripper is -z axis of orientation
    double grip_x = -tip_ori(2);
    // Y value of gripper is sqrt(x^2 + y^2) * direction facing
    double grip_y = sqrt(pow(tip_ori(0), 2) + pow(tip_ori(1), 2)) * 
        ((tip_ori.head(2).dot(yaw_vec) >= 0) ? 1 : -1);
    
    // Tip orientation angle
    res.pose(3) = atan2(grip_y, grip_x);
    
    
    // Twist is negative the twist plus orientation of tip (scaled by how far down arm is pointing)
    res.pose(4) = -joint_vals[TWIST] + grip_x * yaw;

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
    double pitch_derivative = -grip_y * yaw;
    res.jacobian.block<1, 5>(4, 0) << grip_x, 
                                    pitch_derivative,
                                    pitch_derivative,
                                    pitch_derivative,
                                    -1;
    // Define gripper jacobian
    res.jacobian(5, 5) = 1;
    
    return res;
}

Vector6d Kinematics::runInverseKinematics(Vector6d target_pose, Vector6d joint_guess) {
    
    // Initialize initial joint config, fkin, and delta
    Vector6d joint_current = Vector6d(joint_guess);
    FKinResult fkin = runForwardKinematics(joint_current);
    Vector6d pose_delta = target_pose - fkin.pose;

    // Use transpose "spring" method to get close
    int i = 0;
    while (pose_delta.norm() > 0.001) {
        // Calculate optimal move dist (from some paper, I assume it's true)
        Vector6d jj = fkin.jacobian * fkin.jacobian.transpose() * pose_delta;
        double alpha = pose_delta.dot(jj) / jj.dot(jj);

        // Calculate joint delta from spring forces
        Vector6d joint_delta = alpha * fkin.jacobian.transpose() * pose_delta;

        // Run fkin on new point
        fkin = runForwardKinematics(joint_current + joint_delta);

        // Update current joints and pose delta
        pose_delta = target_pose - fkin.pose;
        joint_current += joint_delta;

        i++;
    }

    // Use inverse kinematics to exact in on pose
    int j = 0;
    while (pose_delta.norm() > 0.0000001 && j < 30) {
        // Calculate joint delta from inverse motions
        Vector6d joint_delta = fkin.jacobian.inverse() * pose_delta;

        // Run fkin on this new point
        fkin = runForwardKinematics(joint_current + joint_delta);

        // Reduce joint delta until it actually results in a better position
        // This prevents (or at least mitigates) singularity slingshotting
        Vector6d new_pose_delta = target_pose - fkin.pose;
        while(new_pose_delta.norm() > pose_delta.norm()) {
            joint_delta /= 2;
            fkin = runForwardKinematics(joint_current + joint_delta);
            new_pose_delta = target_pose - fkin.pose;
        }

        // Update current joints and pose delta
        pose_delta = new_pose_delta;
        joint_current += joint_delta;

        j++;
    }

    if (j == 30) {
        cout << "failed to converge" << endl;
        cout << "Guess:\n" << joint_guess << endl;
        cout << "Current:\n" << joint_current << endl;
        cout << "Jacobian:\n" << fkin.jacobian << endl;
        cout << "Inverse:\n" << fkin.jacobian.transpose() << endl;
        exit(1);
    }

    return joint_current;
}