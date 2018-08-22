#include <simple_navigation_goals/simple_navigation_goals.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
namespace simple_navigation_goals{

ClassName::ClassName(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : nh(nh)
    , p_nh(private_nh)

{
// int main(int argc, char** argv){
//    ros::init(argc, argv, "simple_navigation_goals");

//    //pub_=p_nh.advertise<blub_msgs::Blub>("topic_name", 1, false);
//    //sub_=nh.subscribe("/topic_name",10,&ClassName::blaCallback,this);

    simple_action_client = new  MoveBaseClient("move_base", true); // uncommented

//    //wait for the action server to come up
    while(!simple_action_client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 15.0;
    goal.target_pose.pose.position.y = 10.0;
 //   goal.target_pose.pose.position.z = 10.0;

//    goal.target_pose.pose.orientation.w = -1.0;
//    goal.target_pose.pose.orientation.x = 1.0; // doesnt work for now
//    goal.target_pose.pose.orientation.y = 1.0; // doesnt work for now
    goal.target_pose.pose.orientation.z = -1.0;

    ROS_INFO("Sending goal");
    simple_action_client->sendGoal(goal);

    simple_action_client->waitForResult();

    if(simple_action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Yay, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");



}


    ClassName::~ClassName()
{}

//void ClassName::blaCallback(const bla_msgs::Bla &bla_msg){
//}

} // end of namespace simple_navigation_goals
