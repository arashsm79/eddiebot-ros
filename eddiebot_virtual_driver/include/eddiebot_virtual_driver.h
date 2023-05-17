#ifndef _EDDIEBOT_VIRTUAL_DRIVER_H
#define _EDDIEBOT_VIRTUAL_DRIVER_H

#include <semaphore.h>

#include <ros/ros.h>
#include <eddiebot_msgs/Velocity.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class EddieBotVirtualDriver
{
public:
    EddieBotVirtualDriver();
    void publish_odom();
    

private:
    void cmd_vel_cb_(const eddiebot_msgs::Velocity::ConstPtr &msg);
    

private:
    ros::NodeHandle nh_;
    
    ros::Subscriber cmd_vel_sub_;

    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;

    sem_t mutex_data_;

    // previous position
    double x_;
    double y_;
    double th_;

    double vx_;
    double vy_;
    double vth_;

    ros::Time last_time_;
};

#endif