#include <ros/ros.h>
#include <pomdp_car_control/pomdp_car_control.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pomdp_car_control");
    ROS_DEBUG("Starting Car Controller Node!");
    ros::NodeHandle nh("");
    ros::NodeHandle p_nh("~");
    ros::Rate rate(100);

    pomdp_car_control::CarControl car_control(nh, p_nh);
    car_control.setup();

    while (ros::ok()) {
        ros::spinOnce();
        car_control.compute_movement();
        rate.sleep();
    }
    exit(0);
}
