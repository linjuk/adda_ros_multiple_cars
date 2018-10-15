#include <ros/ros.h>
#include <car_teleop/car_teleop.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_teleop_node");
    ROS_DEBUG("Starting car_teleop Node");
    ros::NodeHandle nh("");
    ros::NodeHandle p_nh("~");
    car_teleop::ClassName whatever(nh,p_nh);
    ros::spin();
    exit(0);
}
