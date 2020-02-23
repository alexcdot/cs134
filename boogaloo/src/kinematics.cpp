#include "kinematics.hpp"

const std::string Kinematics::JOINT_NAMES[] = {
    "Boogaloo/yaw",
    "Boogaloo/shoulder",
    "Boogaloo/elbow",
    "Boogaloo/wrist",
    "Boogaloo/twist",
    "Boogaloo/gripper"
};

const double Kinematics::ERR = 0.01;

Kinematics::Kinematics() {
    zero_pos_ = runForwardKinematics(Vector6d::Zero());
}

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
    double yaw = joint_vals(YAW);
    
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

Vector6d Kinematics::runInverseKinematics(Vector6d target_pose) {
    Matrix3d total_rot = Matrix3d::Identity();
    Vector3d total_pos = Vector3d::Zero();

    vector<Vector3d> zero_poses = vector<Vector3d>();

    // Run forward kinematics on zero position, summing total position through 
    // 5 joints
    for(int i = 0; i < 5; i++) {
        total_pos += total_rot * ARM_PROP.translations[i];
        total_rot *= ARM_PROP.rotations[i];

        // Log position of each joint
        zero_poses.push_back(total_pos);
    }
    // From final motor to tip
    total_pos += total_rot * ARM_PROP.translations[5];
    zero_poses.push_back(total_pos);

    // Distance from yaw motor to target tip
    double yaw_to_tip = (target_pose.head(3) - zero_poses[YAW]).head(2).norm();
    // Fixed side offset of gripper from yaw
    double side_offset = zero_poses[GRIPPER].y() - zero_poses[YAW].y();
    // Distance we need to extend
    double extend = sqrt(yaw_to_tip*yaw_to_tip - side_offset*side_offset);

    // Conveniently rename constants we want
    double wrist_len = abs(ARM_PROP.translations[TWIST].x()) + 
                       abs(ARM_PROP.translations[GRIPPER].z());
    double forearm_len = abs(ARM_PROP.translations[WRIST].y());
    double reararm_len = abs(ARM_PROP.translations[ELBOW].x());

    // Wrist angle
    double wrist_angle = target_pose(3);
    // Distance first two joints need to cover
    double extend_2 = extend - wrist_len * sin(wrist_angle);
    // If asked to reach backwards, don't
    extend_2 = max(ERR, extend_2);
    // z height first two joints need to reach 2
    double z_2 = target_pose.z() - zero_poses[SHOULDER].z() + 
                 wrist_len * cos(wrist_angle);
    // Diagonal distance of first two 
    double diag = sqrt(extend_2*extend_2 + z_2*z_2);
    // Make sure diag is in bounds
    diag = min(reararm_len + forearm_len - ERR, diag);
    diag = max(reararm_len - forearm_len + ERR, diag);
    // Get triangle angles
    double diag_ang = acos((reararm_len*reararm_len + forearm_len*forearm_len - diag*diag) / (2 * reararm_len * forearm_len));
    double forearm_ang = acos((diag*diag + reararm_len*reararm_len - forearm_len*forearm_len) / (2 * diag * reararm_len));
    double rise_ang = atan2(z_2, extend_2);

    // Build up joints for shoulder, elbow, wrist, gripper
    Vector6d calc_joints = Vector6d::Zero();
    calc_joints(SHOULDER) = rise_ang + forearm_ang - PI/2;
    calc_joints(ELBOW) = diag_ang - PI/2;
    calc_joints(WRIST) = rise_ang + forearm_ang + diag_ang - wrist_angle - PI;
    calc_joints(GRIPPER) = target_pose(GRIPPER);

    // Calculate yaw angle
    double zeroed_yaw = atan2(side_offset, extend);
    double target_yaw = atan2(target_pose.y() - zero_poses[YAW].y(),
                              target_pose.x() - zero_poses[YAW].x());
    calc_joints(YAW) = target_yaw - zeroed_yaw;

    // Calculate twist angle
    calc_joints(TWIST) = -target_pose(4) + (target_yaw - zeroed_yaw) * cos(target_pose(3));

    return calc_joints;
}

Vector6d Kinematics::getFloatingJointTorques(Vector6d joint_vals) {
    Vector6d torques = Vector6d::Zero();
    torques(WRIST) = ARM_PROP.grav_mag[WRIST] *
        sin(joint_vals(SHOULDER) + joint_vals(ELBOW) - joint_vals(WRIST) - PI + ARM_PROP.grav_off[WRIST]);
    torques(ELBOW) = -torques(WRIST) + ARM_PROP.grav_mag[ELBOW] *
        sin(joint_vals(SHOULDER) + joint_vals(ELBOW) - (PI/2) + ARM_PROP.grav_off[ELBOW]);
    torques(SHOULDER) = torques(ELBOW) + ARM_PROP.grav_mag[SHOULDER] *
        sin(joint_vals(SHOULDER) + ARM_PROP.grav_off[SHOULDER]);

    cout << "wrist " << joint_vals(SHOULDER) + joint_vals(ELBOW) - joint_vals(WRIST) - PI <<
     " elbow " << joint_vals(SHOULDER) + joint_vals(ELBOW) - (PI/2) <<
     " shoulder " << joint_vals(SHOULDER);
    return torques;
}

sensor_msgs::JointState Kinematics::jointsToJS(Vector6d joint_pos, Vector6d joint_vel, Vector6d joint_torque) {
    sensor_msgs::JointState msg;
    for (int i = 0; i < 6; i++) {
        msg.name.push_back(JOINT_NAMES[i]);
        msg.position.push_back(joint_pos(i));
        msg.velocity.push_back(joint_vel(i));
        msg.effort.push_back(joint_torque(i));
    }
    msg.header.frame_id = "world";
    return msg;
}

Joints Kinematics::jsToJoints(sensor_msgs::JointState joints) {
    Joints ret;
    for (int i = 0; i < 6; i++) {
        ret.pos(i) = joints.position[i];
        ret.vel(i) = joints.velocity[i];
        ret.torque(i) = joints.effort[i];
    }
    return ret;
}

sensor_msgs::JointState Kinematics::toHebi(sensor_msgs::JointState normal_joints) {
    sensor_msgs::JointState hebi_joints;
    for (int i = 0; i < 6; i++) {
        hebi_joints.name.push_back(normal_joints.name[i]);
        if (!normal_joints.position.empty())
            hebi_joints.position.push_back(ARM_PROP.zeroes[i] +
                normal_joints.position[i] / ARM_PROP.gearings[i]);
        if (!normal_joints.velocity.empty())
            hebi_joints.velocity.push_back(normal_joints.velocity[i] / ARM_PROP.gearings[i]);
        if (!normal_joints.effort.empty())
            hebi_joints.effort.push_back(normal_joints.effort[i] * ARM_PROP.gearings[i]);
    }
    hebi_joints.header = normal_joints.header;
    return hebi_joints;
}

sensor_msgs::JointState Kinematics::fromHebi(sensor_msgs::JointState hebi_joints) {
    sensor_msgs::JointState normal_joints;
    for (int i = 0; i < 6; i++) {
        normal_joints.name.push_back(hebi_joints.name[i]);
        if (!hebi_joints.position.empty())
            normal_joints.position.push_back(-ARM_PROP.zeroes[i] +
                hebi_joints.position[i] * ARM_PROP.gearings[i]);
        if (!hebi_joints.velocity.empty())
            normal_joints.velocity.push_back(hebi_joints.velocity[i] * ARM_PROP.gearings[i]);
        if (!hebi_joints.effort.empty())
            normal_joints.effort.push_back(hebi_joints.effort[i] / ARM_PROP.gearings[i]);
    }
    normal_joints.header = hebi_joints.header;
    return normal_joints;
}