#include <ros/ros.h>
#include <observations/observations.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "observations_node");
    ROS_DEBUG("Starting observations Node");
    ros::NodeHandle nh("");
    ros::NodeHandle p_nh("~");
    observations::ClassName whatever(nh,p_nh);
    ros::spin();
    exit(0);
}
