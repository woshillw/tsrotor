#include "ros/ros.h"
#include "gazebo_msgs/ApplyBodyWrench.h"
#include "cmath"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tsrotor_addwrench_node");

    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    gazebo_msgs::ApplyBodyWrench srv;

    srv.request.body_name = "tsrotor::tsrotor/base_link";

    ros::Rate loop_rate(500);

    int count = 0;

    while (ros::ok())
    {
        geometry_msgs::Vector3 force;

        force.x = 1+0.1*sin(2.0 * M_PI * count * 0.002);
        force.y = 1+0.1*cos(2.0 * M_PI * count * 0.002);
        force.z = 1+0.1*sin(2.0 * M_PI * count * 0.002);
        
        srv.request.wrench.force = force;

        geometry_msgs::Vector3 torque;
        torque.x = 0.1+0.01*sin(2.0 * M_PI * count * 0.002);
        torque.y = 0.1+0.01*sin(2.0 * M_PI * count * 0.002);
        torque.z = 0.1+0.01*sin(2.0 * M_PI * count * 0.002);
        srv.request.wrench.torque = torque;

        ros::Time time(0);
        ros::Duration duration(-1);

        srv.request.start_time = time;
        srv.request.duration = duration;

        if (client.call(srv))
        {
            ROS_INFO("%d, %s", srv.response.success, srv.response.status_message.c_str());
        }
        else 
        {
            ROS_ERROR("Failed to call service Service_demo");
            return 1;
        }

        loop_rate.sleep();

        ++count;
        if(count>500)
            count = 0;
    }
    return 0;
}
