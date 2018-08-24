#ifndef simple_navigation_goals_H
#define simple_navigation_goals_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


namespace simple_navigation_goals{

//typedef actionlib::SimpleActionClient<bla_msgs::GoToAction> BlaActionClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ClassName {


public:
	ClassName(ros::NodeHandle& nh, ros::NodeHandle& private_nh );
    virtual ~ClassName();

protected:

	//void blaCallback(const bla_msgs::Bla &bla_msg);

 	//void goToCmdActiveCB();
    	//void goToCmdFeedbackCB(const ias_robcom_msgs::GoToFeedbackConstPtr& feedback);
    	//void goToCmdDoneCb(const actionlib::SimpleClientGoalState& state,
                          //const ias_robcom_msgs::GoToResultConstPtr& result);

	ros::NodeHandle& nh;
	ros::NodeHandle& p_nh;
	
private:
	//ros::Publisher pub_;
        //ros::Subscriber sub_;

    MoveBaseClient* simple_action_client_car1;

    //tell the action client that we want to spin a thread by default
    //MoveBaseClient ac("move_base", true);

};

} // end of namespace simple_navigation_goals

#endif // simple_navigation_goals_H
