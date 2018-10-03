#include <nav_msgs/Odometry.h>
#include <pomdp_car_control/pomdp_car_control.h>

namespace pomdp_car_control
{
CarControl::CarControl(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh(nh), p_nh(private_nh)
{



    odom_pub_    = nh.advertise<nav_msgs::Odometry>("/odom", 100);

    current_velocity_ = 0.0;
    twist_.linear.x = current_velocity_;
    change_velocity_ = 0.2;
    max_velocity_ = 3.0;
    min_velocity_ = 0.0;

    nh.getParam("car_start_x", odom_.pose.pose.position.x);   
    nh.getParam("car_start_y", odom_.pose.pose.position.y);
    nh.getParam("car_start_orientation", th_);
    nh.getParam("world_frame", world_frame_);

    std::cout << "world frame: " << world_frame_ << std::endl;

    nh.getParam("base_frame", base_frame_);
    std::cout << "base_frame:  " <<  base_frame_ << std::endl;

    std::cout << "car_start_x:  " <<  odom_.pose.pose.position.x << std::endl;
    std::cout << "car_start_y:  " <<  odom_.pose.pose.position.y << std::endl;
    std::cout << "car_start_orientation:  " <<  th_ << std::endl;



    nh.getParam("vel_topic", vel_topic_);
    cmd_vel_sub_ = nh.subscribe(vel_topic_, 10, &CarControl::cmd_vel_callback, this);
//    pomdp_cmd_vel_sub_ = nh.subscribe("pomdp_vel", 10, &CarControl::pomdp_vel_callback, this);

    velocity_service_ = p_nh.advertiseService("pomdp_velocity", &CarControl::pomdp_vel_callback, this);
//    position_service_ = p_nh.advertiseService("pomdp_position", &CarControl::pomdp_pos_callback, this);
}

void CarControl::setup()
{
    // Setup frames from the world frame (map) to the car (base_link)
    odom_.header.frame_id      = world_frame_;
    odom_.child_frame_id       = base_frame_;
    transform_.frame_id_       = world_frame_;
    transform_.child_frame_id_ = base_frame_;


    // NOT SURE Setup frames from the BASE frame (map) to the car (base_link)
   //  odom_.header.frame_id      = world_frame_;
   //  odom_.child_frame_id       = "base_link";
   //  transform_.frame_id_       = world_frame_;
   //  transform_.child_frame_id_ = "base_link";



    last_time_ = ros::Time::now();
}


bool CarControl::pomdp_vel_callback( pomdp_car_msgs::ActionObservation:: Request &req, pomdp_car_msgs::ActionObservation:: Response &res)
{

    std::cout << "reqest action " <<  req.action << std::endl;

// accelerate

    if (req.action == 1)
    {
        std::cout << "Accelerate: " <<  req.action << std::endl;
        std::cout << "Velocity before change" << current_velocity_ << std::endl;

        // ternary operation (conditon) ? true statement : false statement
        // if(velocity > max){} else{}
        current_velocity_ = current_velocity_ + change_velocity_ >= max_velocity_ ? max_velocity_ : current_velocity_ + change_velocity_ ;
        current_velocity_ = current_velocity_ + change_velocity_ <= min_velocity_ ? min_velocity_ : current_velocity_ + change_velocity_ ;
        std::cout << "Velocity after change" << current_velocity_ << std::endl;
        std::cout << "\n" << std::endl;
    }

// decelerate

    if (req.action == 2)
    {
        std::cout << "Decelerate: " <<  req.action << std::endl;
        std::cout << "Velocity before change" << current_velocity_ << std::endl;
        //current_velocity_ = current_velocity_ - change_velocity_ ;
        current_velocity_ = current_velocity_ - change_velocity_ >= max_velocity_ ? max_velocity_ : current_velocity_ - change_velocity_ ;
        current_velocity_ = current_velocity_ - change_velocity_ <= min_velocity_ ? min_velocity_ : current_velocity_ - change_velocity_ ;
        std::cout << "Velocity after change" << current_velocity_ << std::endl;
        std::cout << "\n" << std::endl;
    }

// hold
    if (req.action == 3)
    {
        std::cout << "Hold: " <<  req.action << std::endl;
        std::cout << "Velocity before change" << current_velocity_ << std::endl;
        //current_velocity_ = current_velocity_;
        current_velocity_ = current_velocity_ >= max_velocity_ ? max_velocity_ : current_velocity_ ;
        current_velocity_ = current_velocity_ <= min_velocity_ ? min_velocity_ : current_velocity_ ;
        std::cout << "Velocity after change" << current_velocity_ << std::endl;
        std::cout << "\n" << std::endl;

    }

// stop
    if (req.action == 4)
    {
        current_velocity_ = 0;
    }

    twist_.linear.x = current_velocity_;

    res.current_velocity = current_velocity_;
    //twist_ = msg;
}


void CarControl::cmd_vel_callback(const geometry_msgs::Twist& msg)
{
    // Only take orientation commands
    twist_.angular.z = msg.angular.z;
    //twist_ = msg;
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

//      std::cout <<  "linear z " << twist_.linear.x<< std::endl;
//      std::cout <<  "angular  " << twist_.angular.z<< std::endl;

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
