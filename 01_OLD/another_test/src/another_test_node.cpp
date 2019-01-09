#include <ros/ros.h>
#include <another_test/another_test.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "another_test_node");
    ROS_DEBUG("Starting another_test Node");
    ros::NodeHandle nh("");
    ros::NodeHandle p_nh("~");
    another_test::ClassName whatever(nh,p_nh);
    ros::spin();
    exit(0);
}
