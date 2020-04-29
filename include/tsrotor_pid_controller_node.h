#ifndef __TSROTOR_PID_CONTROLLER_NODE_H__
#define __TSROTOR_PID_CONTROLLER_NODE_H__

#include <sstream>
#include "iostream"
#include "string"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <math.h>
#include <std_msgs/Float32.h>

#include "tsrotor/tsrotor_gain.h"
#include "tsrotor/tsrotor_pose.h"
#include "tsrotor/tsrotor_vel_pid.h"

#include "pid_control.h"



class Tsrotor
{

public:
    double Sgn(double val);

    Eigen::Vector3d &getRPY();
    Eigen::Quaterniond &getQuaterniond();
    void QtoEuler(Eigen::Vector3d &rpy, const Eigen::Quaterniond &Q);


    void SendRPY();

    void SetMotor(double motor[]);
    
    void SetServo(double servo[]);

    void GetRosParameter(const ros::NodeHandle &nh, const std::string &key,
                         const float &default_value, float *value);

    void showParam();

    void InitParam();

    void QuaternionBasedControl(double dt);
    void EulerBasedControl(double dt);

    void ControlAllocation();

    void OdometryCallback(nav_msgs::Odometry odometry);

    void CommandPosCallback(tsrotor::tsrotor_pose pose);

    void Command_position_gain_Callback(tsrotor::tsrotor_gain pos_gain);

    void Command_velocity_pid_Callback(tsrotor::tsrotor_vel_pid vel_pid);

    void Command_attitude_gain_Callback(tsrotor::tsrotor_gain gain);

    void Command_angular_velocity_pid_Callback(tsrotor::tsrotor_vel_pid vel_pid);

    // void CommandPosCallback(xfly::xfly_pose pose);

    Eigen::MatrixXd Pinv(Eigen::MatrixXd A);
    Tsrotor(ros::NodeHandle &n1,const ros::NodeHandle &_private_nh);

private:
    Eigen::Vector3d state_rpy;        //vehicle current eular angle
    Eigen::Quaterniond state_Q;       //vehicle current Quaternion
    Eigen::Vector3d state_pos;        //vehicle current position
    Eigen::Vector3d state_vel;        //vehicle current body linear velocity
    Eigen::Vector3d state_angle_rate; //vehicle current body angle velocity

    Eigen::Quaterniond err_Q; //Error of Quaternion

    Eigen::Vector3d position_gain; //位置外环比例增益
    Eigen::Vector3d attitude_gain;

    Eigen::Vector3d expect_position; //位置期望
    Eigen::Vector3d expect_EulerAngle;

    Eigen::Vector3d Force_IN_Navigation, Force_IN_Body; //位置控制器输出：全局控制力期望
    Eigen::Vector3d Momrnt_IN_Body;

    Eigen::Vector3d output_angular_accel;

    Eigen::Quaterniond expect_Q; //vehicle expected Quaternion

    Eigen::Matrix<double, 12, 6> _allocationMatrix;

    mav_msgs::Actuators motor_msg, servo_msg;  //vehicle publish motor message

    ros::NodeHandle n1,_private_nh;            //ros node

    ros::Publisher motor_pub, servo_pub;       //ros motor publisher

    ros::Subscriber odemetry_sub;  //ros odemetry subscriber

    ros::Subscriber command_pose_sub;

    ros::Subscriber command_position_gain_sub, command_velocity_pid_sub;

    ros::Subscriber command_attitude_gain_sub, command_angular_velocity_pid_sub;

    ros::Publisher _state_r_pub, _state_p_pub, _state_y_pub;

    ros::Publisher _state_r_der_pub, _state_p_der_pub, _state_y_der_pub;

    ros::Publisher _state_u_pub,_state_v_pub,_state_w_pub;

    ros::Publisher _state_u_der_pub,_state_v_der_pub,_state_w_der_pub;

    PID _pid_p, _pid_q, _pid_r; //角速度PID控制器

    PID _pid_u, _pid_v, _pid_w;

    float _mass, _Ix, _Iy, _Iz; //飞行器质量、惯量

    double dt; //control duty
};

#endif