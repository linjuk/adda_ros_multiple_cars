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

 // CAR 1 //

    simple_action_client_car1 = new  MoveBaseClient("/car1/move_base", true); // uncommented

//    //wait for the action server to come up
    while(!simple_action_client_car1->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "/car1/map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 45.0;
    goal.target_pose.pose.position.y = 17.0;
 //   goal.target_pose.pose.position.z = 10.0;

//    goal.target_pose.pose.orientation.w = -1.0;
//    goal.target_pose.pose.orientation.x = 1.0; // doesnt work for now
//    goal.target_pose.pose.orientation.y = 1.0; // doesnt work for now
    goal.target_pose.pose.orientation.z = 3.141592654;

    ROS_INFO("Sending goal");
    simple_action_client_car1->sendGoal(goal);



//  CAR 2 //

   simple_action_client_car2 = new  MoveBaseClient("/car2/move_base", true); // added for car2

    //wait for the action server to come up
       while(!simple_action_client_car2->waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
       }

       move_base_msgs::MoveBaseGoal goal2;
       //we'll send a goal to the robot to move 1 meter forward
       goal.target_pose.header.frame_id = "/car2/map";
       goal.target_pose.header.stamp = ros::Time::now();

       goal.target_pose.pose.position.x = 33.0;
       goal.target_pose.pose.position.y = 45.0;
       goal.target_pose.pose.orientation.z = 29.841592654;

       ROS_INFO("Sending goal");
       simple_action_client_car2->sendGoal(goal);



      // Wait for results

       simple_action_client_car2->waitForResult();

       if(simple_action_client_car2->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
           ROS_INFO("Car2 arrived to the goal point");
       else
           ROS_INFO("Car2 failed to reach the goal for some reason");


       simple_action_client_car1->waitForResult();

       if(simple_action_client_car1->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
           ROS_INFO("Car1 arrived to the goal point");
       else
           ROS_INFO("Car1 failed to reach the goal for some reason");



}


    ClassName::~ClassName()
{}

//void ClassName::blaCallback(const bla_msgs::Bla &bla_msg){
//}

} // end of namespace simple_navigation_goals
