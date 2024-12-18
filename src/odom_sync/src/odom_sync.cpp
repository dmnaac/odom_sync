#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> SyncPolicy;

typedef message_filters::Synchronizer<SyncPolicy> Sync;

const std::string ODOMETRY_1_TOPIC = "";
const std::string ODOMETRY_2_TOPIC = "";

void HandleSyncedMessage(const nav_msgs::OdometryConstPtr &odom_msg1, const nav_msgs::OdometryConstPtr &odom_msg2)
{
    ROS_INFO("Synchronized messages received!");

    geometry_msgs::Twist twist1 = odom_msg1->twist.twist;
    geometry_msgs::Twist twist2 = odom_msg2->twist.twist;
    double linear_vel_1 = std::sqrt(twist1.linear.x * twist1.linear.x + twist1.linear.y * twist1.linear.y);
    double linear_vel_2 = std::sqrt(twist2.linear.x * twist2.linear.x + twist2.linear.y * twist2.linear.y);
    double slip_ratio_linear = std::fabs(linear_vel_1 - linear_vel_2) / linear_vel_1;
    double slip_ratio_angular = std::fabs(twist1.angular.z - twist2.angular.z) / twist1.angular.z;

    ROS_INFO("Linear slip ratio: %f, Angular slip ratio: %f", slip_ratio_linear, slip_ratio_angular);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_sync_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<nav_msgs::Odometry> sub1(nh, ODOMETRY_1_TOPIC, 10);
    message_filters::Subscriber<nav_msgs::Odometry> sub2(nh, ODOMETRY_2_TOPIC, 10);

    Sync sync(SyncPolicy(10), sub1, sub2);

    sync.registerCallback(boost::bind(&HandleSyncedMessage, _1, _2));

    ros::spin();

    return 0;
}