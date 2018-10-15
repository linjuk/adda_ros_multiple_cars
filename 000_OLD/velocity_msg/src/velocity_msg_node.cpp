#include <ros/ros.h>
#include <velocity_msg/velocity_msg.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_msg_node");
    ROS_DEBUG("Starting velocity_msg Node");
    ros::NodeHandle nh("");
    ros::NodeHandle p_nh("~");
    velocity_msg::CarControl whatever(nh,p_nh);
    ros::spin();
    exit(0);
}
