#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;
geometry_msgs::PoseStamped pose;
geometry_msgs::TwistStamped vel;
/*double last_x = 0;
double last_y = 0;
double last_z = 0;
double vx = 0;
double vy = 0;
double vz = 0;*/
void POSEcallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    pose = *pose_msg;
    /*vx = (pose.pose.position.x - last_x) * 160;
    vy = (pose.pose.position.y - last_y) * 160;
    vz = (pose.pose.position.z - last_z) * 160;
    last_x = pose.pose.position.x;
    last_y = pose.pose.position.y;
    last_z = pose.pose.position.z;*/
}

void VELcallback(const geometry_msgs::TwistStamped::ConstPtr &twist_msg)
{
    vel = *twist_msg;
}

void timerCallback(const ros::TimerEvent &)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";

    odom.pose.pose = pose.pose;
    odom.twist.twist = vel.twist;
    /*odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = vz;
    odom.twist.twist.angular = vel.twist.angular;*/
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_pose_and_velocity");
    ros::NodeHandle nh;

    odom_pub = nh.advertise<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 1);

    ros::Subscriber pose_sub = nh.subscribe("/vrpn_client_node/YZU0/0/pose", 1, POSEcallback);
    ros::Subscriber twist_sub = nh.subscribe("/vrpn_client_node/YZU0/0/velocity", 1, VELcallback);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / 160.0), timerCallback);

    ros::spin();
    return 0;
}
