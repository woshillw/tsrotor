#include "tsrotor_pid_controller_node.h"

Tsrotor::Tsrotor(ros::NodeHandle &n1, const ros::NodeHandle &_private_nh)
  : expect_Q(1.0f, 0.0f, 0.0f, 0.0f),
    expect_position(0.0f, 0.0f, 3.0f),
    expect_EulerAngle(0.0f,0.0f,0.0f),
    position_gain(0.0f, 0.0f, 0.0f),
    attitude_gain(0.0f, 0.0f, 0.0f),
    output_angular_accel(0.0f, 0.0f, 0.0f),
    dt(0.01),
    n1(n1),
    _private_nh(_private_nh)
{
    odemetry_sub = n1.subscribe("/tsrotor/ground_truth/odometry", 10, &Tsrotor::OdometryCallback, this);

    command_pose_sub = n1.subscribe("command/pose", 1, &Tsrotor::CommandPosCallback, this);

    command_position_gain_sub = n1.subscribe("command/position_gain", 1, &Tsrotor::Command_position_gain_Callback, this);

    command_velocity_pid_sub = n1.subscribe("command/velocity_pid", 1, &Tsrotor::Command_velocity_pid_Callback, this);

    command_attitude_gain_sub = n1.subscribe("command/attitude_gain", 1, &Tsrotor::Command_attitude_gain_Callback, this);

    command_angular_velocity_pid_sub = n1.subscribe("command/attitude_velocity_pid", 1, &Tsrotor::Command_angular_velocity_pid_Callback, this);

    motor_pub = n1.advertise<mav_msgs::Actuators>("/tsrotor/gazebo/command/motor_speed", 10);

    servo_pub = n1.advertise<mav_msgs::Actuators>("/tsrotor/gazebo/command/servo_position", 10);

    _state_r_pub = n1.advertise<std_msgs::Float32>("state/roll", 1);

    _state_p_pub = n1.advertise<std_msgs::Float32>("state/pitch", 1);

    _state_y_pub = n1.advertise<std_msgs::Float32>("state/yaw", 1);

    _state_r_der_pub = n1.advertise<std_msgs::Float32>("state/roll_der", 1);

    _state_p_der_pub = n1.advertise<std_msgs::Float32>("state/pitch_der", 1);

    _state_y_der_pub = n1.advertise<std_msgs::Float32>("state/yaw_der", 1);

    _state_u_pub = n1.advertise<std_msgs::Float32>("state/xpos", 1);

    _state_v_pub = n1.advertise<std_msgs::Float32>("state/ypos", 1);

    _state_w_pub = n1.advertise<std_msgs::Float32>("state/zpos", 1);

    _state_u_der_pub = n1.advertise<std_msgs::Float32>("state/xvel", 1);

    _state_v_der_pub = n1.advertise<std_msgs::Float32>("state/yvel", 1);

    _state_w_der_pub = n1.advertise<std_msgs::Float32>("state/zvel", 1);

    InitParam();
}

Eigen::MatrixXd Tsrotor::Pinv(Eigen::MatrixXd A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV); // M=USV*
    double Pinvtoler = 1.e-8;                                                            // tolerance
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row, col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues(); //奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i < k; ++i)
    {
        if (singularValues_inv(i) > Pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose()); // X=VS+U*

    return X;
}

void Tsrotor::CommandPosCallback(tsrotor::tsrotor_pose pose)
{
    expect_position(0) = pose.x;
    expect_position(1) = pose.y;
    expect_position(2) = pose.z;
    expect_EulerAngle(0) = pose.roll;
    expect_EulerAngle(1) = pose.pitch;
    expect_EulerAngle(2) = pose.yaw;
}

void Tsrotor::Command_attitude_gain_Callback(tsrotor::tsrotor_gain gain)
{
    attitude_gain(0) = gain.xgain;
    attitude_gain(1) = gain.ygain;
    attitude_gain(2) = gain.zgain;
}

void Tsrotor::Command_angular_velocity_pid_Callback(tsrotor::tsrotor_vel_pid vel_pid)
{
    _pid_u.kp = vel_pid.xvel_pid.kp;
    _pid_u.ki = vel_pid.xvel_pid.ki;
    _pid_u.kd = vel_pid.xvel_pid.kd;

    _pid_v.kp = vel_pid.yvel_pid.kp;
    _pid_v.ki = vel_pid.yvel_pid.ki;
    _pid_v.kd = vel_pid.yvel_pid.kd;

    _pid_w.kp = vel_pid.zvel_pid.kp;
    _pid_w.ki = vel_pid.zvel_pid.ki;
    _pid_w.kd = vel_pid.zvel_pid.kd;

    std::cout << " _pid_u:" << "kp:"<< _pid_u.kp << " ki:" << _pid_u.ki << " kd:" << _pid_u.kd << std::endl;
    std::cout << " _pid_v:" << "kp:"<< _pid_v.kp << " ki:" << _pid_v.ki << " kd:" << _pid_v.kd << std::endl;
    std::cout << " _pid_w:" << "kp:"<< _pid_w.kp << " ki:" << _pid_w.ki << " kd:" << _pid_w.kd << std::endl;
}

void Tsrotor::Command_position_gain_Callback(tsrotor::tsrotor_gain data)
{
    position_gain[0] = data.xgain;
    position_gain[1] = data.ygain;
    position_gain[2] = data.zgain;
}

void Tsrotor::Command_velocity_pid_Callback(tsrotor::tsrotor_vel_pid data)
{
    _pid_u.kp = data.xvel_pid.kp;
    _pid_u.ki = data.xvel_pid.ki;
    _pid_u.kd = data.xvel_pid.kd;

    _pid_v.kp = data.yvel_pid.kp;
    _pid_v.ki = data.yvel_pid.ki;
    _pid_v.kd = data.yvel_pid.kd;

    _pid_w.kp = data.zvel_pid.kp;
    _pid_w.ki = data.zvel_pid.ki;
    _pid_w.kd = data.zvel_pid.kd;

    std::cout << " xvel_kp:" << _pid_u.kp << " xvel_ki:" << _pid_u.ki << " xvel_kd:" << _pid_u.kd << std::endl;
    std::cout << " yvel_kp:" << _pid_v.kp << " yvel_ki:" << _pid_v.ki << " yvel_kd:" << _pid_v.kd << std::endl;
    std::cout << " zvel_kp:" << _pid_w.kp << " zvel_ki:" << _pid_w.ki << " zvel_kd:" << _pid_w.kd << std::endl;
}

void Tsrotor::OdometryCallback(nav_msgs::Odometry odometry)
{
    // ROS_INFO("Get!");

    state_Q.w() = odometry.pose.pose.orientation.w;
    state_Q.x() = odometry.pose.pose.orientation.x;
    state_Q.y() = odometry.pose.pose.orientation.y;
    state_Q.z() = odometry.pose.pose.orientation.z;
    state_Q.normalize();

    state_pos(0) = odometry.pose.pose.position.x;
    state_pos(1) = odometry.pose.pose.position.y;
    state_pos(2) = odometry.pose.pose.position.z;

    state_vel(0) = odometry.twist.twist.linear.x;
    state_vel(1) = odometry.twist.twist.linear.y;
    state_vel(2) = odometry.twist.twist.linear.z;
    state_vel = state_Q.toRotationMatrix() * state_vel; //从机体系旋转到导航系

    state_angle_rate(0) = odometry.twist.twist.angular.x;
    state_angle_rate(1) = odometry.twist.twist.angular.y;
    state_angle_rate(2) = odometry.twist.twist.angular.z;
}


void Tsrotor::GetRosParameter(const ros::NodeHandle &nh, const std::string &key,
                                    const float &default_value, float *value)
{
    ROS_ASSERT(value != nullptr);
    bool have_parameter = nh.getParam(key, *value);
    if (!have_parameter)
    {
        ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace() << "/" << key
                                                                << ", setting to default: " << default_value);
        *value = default_value;
    }
}


void Tsrotor::InitParam()
{
    float gain, kp, ki, kd, integral_limit, output_limit,dt;
    GetRosParameter(_private_nh, "pid_dt", 0.01, &dt);

    GetRosParameter(_private_nh, "position_gain/x", 0.0, &gain);
    position_gain(0) = gain;
    GetRosParameter(_private_nh, "position_gain/y", 0.0, &gain);
    position_gain(1) = gain;
    GetRosParameter(_private_nh, "position_gain/z", 0.0, &gain);
    position_gain(2) = gain;

    GetRosParameter(_private_nh, "attitude_gain/x", 0.0, &gain);
    attitude_gain(0) = gain;
    GetRosParameter(_private_nh, "attitude_gain/y", 0.0, &gain);
    attitude_gain(1) = gain;
    GetRosParameter(_private_nh, "attitude_gain/z", 0.0, &gain);
    attitude_gain(2) = gain;

    GetRosParameter(_private_nh, "pid_u/kp", 1, &kp);
    GetRosParameter(_private_nh, "pid_u/ki", 0.01, &ki);
    GetRosParameter(_private_nh, "pid_u/kd", 0.55, &kd);
    GetRosParameter(_private_nh, "pid_u/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_u/output_limit", 1, &output_limit);
    PID_Init(&_pid_u, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_v/kp", 1, &kp);
    GetRosParameter(_private_nh, "pid_v/ki", 0.01, &ki);
    GetRosParameter(_private_nh, "pid_v/kd", 0.55, &kd);
    GetRosParameter(_private_nh, "pid_v/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_v/output_limit", 1, &output_limit);
    PID_Init(&_pid_v, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_w/kp", 1, &kp);
    GetRosParameter(_private_nh, "pid_w/ki", 0.01, &ki);
    GetRosParameter(_private_nh, "pid_w/kd", 0.55, &kd);
    GetRosParameter(_private_nh, "pid_w/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_w/output_limit", 1, &output_limit);
    PID_Init(&_pid_w, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_p/kp", 1.8, &kp);
    GetRosParameter(_private_nh, "pid_p/ki", 0.025, &ki);
    GetRosParameter(_private_nh, "pid_p/kd", 0.88, &kd);
    GetRosParameter(_private_nh, "pid_p/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_p/output_limit", 1, &output_limit);
    PID_Init(&_pid_p, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_q/kp", 1.8, &kp);
    GetRosParameter(_private_nh, "pid_q/ki", 0.025, &ki);
    GetRosParameter(_private_nh, "pid_q/kd", 0.88, &kd);
    GetRosParameter(_private_nh, "pid_q/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_q/output_limit", 1, &output_limit);
    PID_Init(&_pid_q, kp, ki, kd, dt, output_limit, integral_limit);
    
    GetRosParameter(_private_nh, "pid_r/kp", 1.8, &kp);
    GetRosParameter(_private_nh, "pid_r/ki", 0.025, &ki);
    GetRosParameter(_private_nh, "pid_r/kd", 0.88, &kd);
    GetRosParameter(_private_nh, "pid_r/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_r/output_limit", 1, &output_limit);
    PID_Init(&_pid_r, kp, ki, kd, dt, output_limit, integral_limit);
    
    float kf, km, l;
    ////从 rosparam 读取飞行器参数 初始化控制分配矩阵
    GetRosParameter(_private_nh, "vehicle/kf", 1.7088e-05, &kf);
    GetRosParameter(_private_nh, "vehicle/km", 1.7088e-05 * 0.016, &km);
    GetRosParameter(_private_nh, "vehicle/l", 0.355, &l);
    GetRosParameter(_private_nh, "vehicle/mass", 2.274, &_mass);
    GetRosParameter(_private_nh, "vehicle/Ix", 0.021968, &_Ix);
    GetRosParameter(_private_nh, "vehicle/Iy", 0.021968, &_Iy);
    GetRosParameter(_private_nh, "vehicle/Iz", 0.042117, &_Iz);

    Eigen::Matrix<double, 6, 12> angular_to_m;

    // [                 0,            kf/2,     0,    kf,                0,            kf/2,                0,          -kf/2,    0,   -kf,                 0,           -kf/2]
    // [                 0, -(3^(1/2)*kf)/2,     0,     0,                0,  (3^(1/2)*kf)/2,                0, (3^(1/2)*kf)/2,    0,     0,                 0, -(3^(1/2)*kf)/2]
    // [                kf,               0,    kf,     0,               kf,               0,               kf,              0,   kf,     0,                kf,               0]
    // [         -(kf*l)/2,            km/2, -kf*l,   -km,        -(kf*l)/2,            km/2,         (kf*l)/2,           km/2, kf*l,   -km,          (kf*l)/2,            km/2]
    // [ -(3^(1/2)*kf*l)/2,  (3^(1/2)*km)/2,     0,     0, (3^(1/2)*kf*l)/2, -(3^(1/2)*km)/2, (3^(1/2)*kf*l)/2, (3^(1/2)*km)/2,    0,     0, -(3^(1/2)*kf*l)/2, -(3^(1/2)*km)/2]
    // [               -km,           -kf*l,    km, -kf*l,              -km,           -kf*l,               km,          -kf*l,  -km, -kf*l,                km,           -kf*l]

    angular_to_m <<
                 0,           -kf/2,     0,   -kf,                0,           -kf/2,                0,           kf/2,    0,    kf,                 0,            kf/2,
                 0, -(sqrt(3)*kf)/2,     0,     0,                0,  (sqrt(3)*kf)/2,                0, (sqrt(3)*kf)/2,    0,     0,                 0, -(sqrt(3)*kf)/2,
                kf,               0,    kf,     0,               kf,               0,               kf,              0,   kf,     0,                kf,               0,
         -(kf*l)/2,            km/2, -kf*l,   -km,        -(kf*l)/2,            km/2,         (kf*l)/2,           km/2, kf*l,   -km,          (kf*l)/2,            km/2,
 -(sqrt(3)*kf*l)/2,  (sqrt(3)*km)/2,     0,     0, (sqrt(3)*kf*l)/2, -(sqrt(3)*km)/2, (sqrt(3)*kf*l)/2, (sqrt(3)*km)/2,    0,     0, -(sqrt(3)*kf*l)/2, -(sqrt(3)*km)/2,
               -km,           -kf*l,    km, -kf*l,              -km,           -kf*l,               km,          -kf*l,  -km, -kf*l,                km,           -kf*l;

    _allocationMatrix = Pinv(angular_to_m);
}

void Tsrotor::EulerBasedControl(double dt)
{
    Eigen::Vector3d expect_velocity(0, 0, 0);
    Eigen::Vector3d expect_angular(0, 0, 0);
    Eigen::Vector3d expect_angular_acc(0, 0, 0);
    Eigen::Vector3d expect_rpy(0, 0, 0);
    Eigen::Vector3d expect_accel(0, 0, 0);

    ////////////位置控制 双环PID///////////////////
    expect_velocity <<  position_gain(0) * (expect_position(0) - state_pos(0)),
                        position_gain(1) * (expect_position(1) - state_pos(1)),
                        position_gain(2) * (expect_position(2) - state_pos(2));

    expect_accel << PID_calculate(&_pid_u, expect_velocity(0), state_vel(0)),
                    PID_calculate(&_pid_v, expect_velocity(1), state_vel(1)),
                    PID_calculate(&_pid_w, expect_velocity(2), state_vel(2));
  
    // std::printf("expect_accel(2)=%f\n", expect_accel(2));
    // std::printf("expect_velocity(2)=%f\n", expect_velocity(2));
    // std::printf("state_vel(2)=%f\n", state_vel(2));
    // std::printf("kp=%f,ki=%f,kd=%f,dt=%f,out=%f\n", _pid_w.kp, _pid_w.ki, _pid_w.kd, _pid_w.dt, _pid_w.output);
    // std::printf("cur_err=%f,last_err=%f,integral=%f\n\n", _pid_w.cur_err, _pid_w.last_err, _pid_w.integral);

    if(isnan(expect_accel(0)))
        expect_accel(0) = 0;
    if(isnan(expect_accel(1)))
        expect_accel(1) = 0;
    if(isnan(expect_accel(2)))
        expect_accel(2) = 0;

    std::printf("expect_accel out(1)=%f,out(2)=%f,out(3)=%f\n", expect_angular_acc(0), expect_angular_acc(1), expect_angular_acc(2));

    Force_IN_Navigation = expect_accel * 9.8f * _mass;
    Force_IN_Navigation(2) += 9.8f * _mass;
    // Force_IN_Navigation(0) = 0;
    // Force_IN_Navigation(1) = 0;
    ///////////////姿态控制 双环PID///////////
    expect_angular <<   attitude_gain(0) * (expect_EulerAngle(0) - state_rpy(0)*57.3f),
                        attitude_gain(1) * (expect_EulerAngle(1) - state_rpy(1)*57.3f),
                        attitude_gain(2) * (expect_EulerAngle(2) - state_rpy(2)*57.3f);

    expect_angular = expect_angular / 57.3f;

    expect_angular_acc <<   PID_calculate(&_pid_p, expect_angular(0), state_angle_rate(0)),
                            PID_calculate(&_pid_q, expect_angular(1), state_angle_rate(1)),
                            PID_calculate(&_pid_r, expect_angular(2), state_angle_rate(2));

    if(state_pos(2)<1)
    {
        expect_angular_acc(0) = 0;
        expect_angular_acc(1) = 0;
        expect_angular_acc(2) = 0;
    }

    // if(isnan(expect_angular_acc(0)))
    //     expect_angular_acc(0) = 0;
    // if(isnan(expect_angular_acc(1)))
    //     expect_angular_acc(1) = 0;
    // if(isnan(expect_angular_acc(2)))
    //     expect_angular_acc(2) = 0;

    std::printf("expect_angular_acc out(1)=%f,out(2)=%f,out(3)=%f\n\n", expect_angular_acc(0), expect_angular_acc(1), expect_angular_acc(2));

    Momrnt_IN_Body = expect_angular_acc;
}

void Tsrotor::ControlAllocation()
{
    Eigen::Matrix3d eRb = state_Q.toRotationMatrix();
    Eigen::Matrix3d bRe = eRb.transpose();
    Force_IN_Body = bRe * Force_IN_Navigation;

    Eigen::Matrix<double, 6, 1> ForceMomrnt_IN_Body;
    ForceMomrnt_IN_Body << Force_IN_Body, Momrnt_IN_Body;

    Eigen::Matrix<double, 12, 1> VirtualControl;
    VirtualControl = _allocationMatrix * ForceMomrnt_IN_Body;

    //////////////通过控制分配将机体系期望力和力矩转化为电机转速和倾转角
    double motor[6];//
    double alpha[6];//

    motor[0] = sqrt(sqrt(VirtualControl(0) * VirtualControl(0) + VirtualControl(1) * VirtualControl(1)));
    alpha[0] = atan2(VirtualControl(1), VirtualControl(0));

    motor[1] = sqrt(sqrt(VirtualControl(2) * VirtualControl(2) + VirtualControl(3) * VirtualControl(3)));
    alpha[1] = atan2(VirtualControl(3), VirtualControl(2));

    motor[2] = sqrt(sqrt(VirtualControl(4) * VirtualControl(4) + VirtualControl(5) * VirtualControl(5)));
    alpha[2] = atan2(VirtualControl(5), VirtualControl(4));

    motor[3] = sqrt(sqrt(VirtualControl(6) * VirtualControl(6) + VirtualControl(7) * VirtualControl(7)));
    alpha[3] = atan2(VirtualControl(7), VirtualControl(6));

    motor[4] = sqrt(sqrt(VirtualControl(8) * VirtualControl(8) + VirtualControl(9) * VirtualControl(9)));
    alpha[4] = atan2(VirtualControl(9), VirtualControl(8));

    motor[5] = sqrt(sqrt(VirtualControl(10) * VirtualControl(10) + VirtualControl(11) * VirtualControl(11)));
    alpha[5] = atan2(VirtualControl(11), VirtualControl(10));

    SetMotor(motor);
    SetServo(alpha);
}

Eigen::Vector3d &Tsrotor::getRPY()
{
    return state_rpy;
}

Eigen::Quaterniond &Tsrotor::getQuaterniond()
{
    return state_Q;
}

void Tsrotor::showParam()
{
    std::cout << "FM_b_to_actuator allocation:\n"
              << _allocationMatrix << std::endl;
}

void Tsrotor::QtoEuler(Eigen::Vector3d &rpy, const Eigen::Quaterniond &Q)
{
    rpy(0) = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), 1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y()));
    rpy(1) = asin(2 * (Q.w() * Q.y() - Q.x() * Q.z()));
    rpy(2) = atan2(2 * (Q.w() * Q.z() + Q.y() * Q.x()), 1 - 2 * (Q.z() * Q.z() + Q.y() * Q.y()));
}


void Tsrotor::SendRPY()
{
    std_msgs::Float32 msg;
    msg.data = state_rpy(0) / M_PI * 180;
    _state_r_pub.publish(msg);
    msg.data = state_rpy(1) / M_PI * 180;
    _state_p_pub.publish(msg);
    msg.data = state_rpy(2) / M_PI * 180;
    _state_y_pub.publish(msg);

    msg.data = state_angle_rate(0) / M_PI * 180;
    _state_r_der_pub.publish(msg);
    msg.data = state_angle_rate(1) / M_PI * 180;
    _state_p_der_pub.publish(msg);
    msg.data = state_angle_rate(2) / M_PI * 180;
    _state_y_der_pub.publish(msg);

    msg.data = state_pos(0);
    _state_u_pub.publish(msg);
    msg.data = state_pos(1);
    _state_v_pub.publish(msg);
    msg.data = state_pos(2);
    _state_w_pub.publish(msg);

    msg.data = state_vel(0);
    _state_u_der_pub.publish(msg);
    msg.data = state_vel(1);
    _state_v_der_pub.publish(msg);
    msg.data = state_vel(2);
    _state_w_der_pub.publish(msg);
}

void Tsrotor::SetMotor(double motor[])
{
    motor_msg.angular_velocities.clear();

    motor_msg.angular_velocities.push_back(motor[0]);

    motor_msg.angular_velocities.push_back(motor[1]);

    motor_msg.angular_velocities.push_back(motor[2]);

    motor_msg.angular_velocities.push_back(motor[3]);

    motor_msg.angular_velocities.push_back(motor[4]);

    motor_msg.angular_velocities.push_back(motor[5]);

    motor_pub.publish(motor_msg);
}


void Tsrotor::SetServo(double servo[])
{
    servo_msg.angular_velocities.clear();

    servo_msg.angular_velocities.push_back(servo[0]);

    servo_msg.angular_velocities.push_back(servo[1]);

    servo_msg.angular_velocities.push_back(servo[2]);

    servo_msg.angular_velocities.push_back(servo[3]);

    servo_msg.angular_velocities.push_back(servo[4]);

    servo_msg.angular_velocities.push_back(servo[5]);

    servo_pub.publish(servo_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tsrotor_pid_controller_node");

    ros::NodeHandle nh;
    
    ros::NodeHandle private_nh("~");

    Tsrotor tsrotor(nh, private_nh);

    ros::Rate loop_rate(100);

    tsrotor.showParam();

    // double motor[6]={100,100,100,100,100,100};//
    // double alpha[6]={0.6,0.6,0.6,0.6,0.6,0.6};//

    while (ros::ok())
    {
        tsrotor.EulerBasedControl(0.01);

        tsrotor.ControlAllocation();


        // tsrotor.SetMotor(motor);
        // tsrotor.SetServo(alpha);

        ros::spinOnce();

        tsrotor.QtoEuler(tsrotor.getRPY(), tsrotor.getQuaterniond());

        tsrotor.SendRPY();

        loop_rate.sleep();
    }

    return 0;
}
