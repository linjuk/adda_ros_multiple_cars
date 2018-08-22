#include <nav_msgs/Odometry.h>
#include <pomdp_car_control/pomdp_car_control.h>

namespace pomdp_car_control
{
CarControl::CarControl(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh(nh), p_nh(private_nh)
{
    cmd_vel_sub_ = nh.subscribe("/cmd_vel", 10, &CarControl::cmd_vel_callback, this);

    odom_pub_    = nh.advertise<nav_msgs::Odometry>("/odom", 100);

    nh.getParam("car_start_x", odom_.pose.pose.position.x);
    nh.getParam("car_start_y", odom_.pose.pose.position.y);
    nh.getParam("car_start_orientation", th_);
    nh.getParam("world_frame", world_frame_);

    std::cout << "world frame: " << world_frame_ << std::endl;
}

void CarControl::setup()
{
    // Setup frames from the world frame (map) to the car (base_link)
    odom_.header.frame_id      = world_frame_;
    odom_.child_frame_id       = "base_link";
    transform_.frame_id_       = world_frame_;
    transform_.child_frame_id_ = "base_link";

    last_time_ = ros::Time::now();
}


void CarControl::cmd_vel_callback(const geometry_msgs::Twist& msg)
{
    // Only take orientation commands
//    twist_.angular.z = msg.angular.z;
    twist_ = msg;
}

void CarControl::compute_movement()
{
    ros::Time current_time  = ros::Time::now();
    odom_.header.stamp = current_time;
    transform_.stamp_  = current_time;

    // Compute change of movement
    double dt = (current_time - last_time_).toSec();
    double dx = twist_.linear.x * std::cos(th_) * dt;
    double dy = twist_.linear.x * std::sin(th_) * dt;
    double dh = twist_.angular.z * dt;

    // Apply Odometry
    odom_.pose.pose.position.x += dx;
    odom_.pose.pose.position.y += dy;
    th_ += dh;
    odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_);
    odom_.twist.twist           = twist_;

    // Apply TF
    transform_.setOrigin(tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, th_);
    transform_.setRotation(q);

    // Publish Odometry and TF
    odom_pub_.publish(odom_);
    tf_broadcaster_.sendTransform(transform_);
    last_time_ = current_time;
}

CarControl::~CarControl() = default;

} // namespace pomdp_car_control
