#include <ros/ros.h>
#include <simple_navigation_goals/simple_navigation_goals.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_navigation_goals_node");
    ROS_DEBUG("Starting simple_navigation_goals Node");
    ros::NodeHandle nh("");
    ros::NodeHandle p_nh("~");
    simple_navigation_goals::ClassName whatever(nh,p_nh);
    ros::spin();
    exit(0);
}
