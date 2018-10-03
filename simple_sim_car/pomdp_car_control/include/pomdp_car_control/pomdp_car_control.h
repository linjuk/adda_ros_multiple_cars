#pragma once

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <pomdp_car_msgs/ActionObservation.h>

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

    bool pomdp_vel_callback(pomdp_car_msgs::ActionObservation::Request &req, pomdp_car_msgs::ActionObservation::Response &res);
//    bool pomdp_pos_callback(pomdp_car_msgs::ActionObservation::Request &req, pomdp_car_msgs::ActionObservation::Response &res);



    ros::NodeHandle& nh;
    ros::NodeHandle& p_nh;

private:
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber pomdp_cmd_vel_sub_;
    ros::Publisher odom_pub_;
    nav_msgs::Odometry odom_;
    geometry_msgs::Twist twist_;
    ros::ServiceServer velocity_service_;
//    ros::ServiceServer position_service_;


    std::string world_frame_;
    std::string base_frame_;
    std::string vel_topic_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::StampedTransform transform_;

    ros::Time last_time_;

    double th_ = 0.0;

    float current_velocity_;
    float change_velocity_;
    float max_velocity_;
    float min_velocity_;
};


} // namespace pomdp_car_control
