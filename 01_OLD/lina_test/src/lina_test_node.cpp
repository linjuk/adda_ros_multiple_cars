#include <ros/ros.h>
#include <lina_test/lina_test.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lina_test_node");
    ROS_DEBUG("Starting lina_test Node");
    ros::NodeHandle nh("");
    ros::NodeHandle p_nh("~");
    lina_test::ClassName whatever(nh,p_nh);
    ros::spin();
    exit(0);
}
