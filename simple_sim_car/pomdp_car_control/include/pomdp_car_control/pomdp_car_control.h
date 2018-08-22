#pragma once

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace pomdp_car_control
{
class CarControl
{
public:
    CarControl(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    virtual ~CarControl();

    /**
     * Initial setup routine. Gets values from the parameter server.
     */
    void setup();

    /**
     * Computes the new position based on the incoming velocity command and the current position.
     */
    void compute_movement();

protected:
    void cmd_vel_callback(const geometry_msgs::Twist& msg);

    ros::NodeHandle& nh;
    ros::NodeHandle& p_nh;

private:
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    nav_msgs::Odometry odom_;
    geometry_msgs::Twist twist_;

    std::string world_frame_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::StampedTransform transform_;

    ros::Time last_time_;

    double th_ = 0.0;
};

} // namespace pomdp_car_control
