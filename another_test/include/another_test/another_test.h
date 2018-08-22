#ifndef another_test_H
#define another_test_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
//include <actionlib/client/simple_action_client.h>

namespace another_test{
//typedef actionlib::SimpleActionClient<bla_msgs::GoToAction> BlaActionClient;
class ClassName {

public:
	ClassName(ros::NodeHandle& nh, ros::NodeHandle& private_nh );
    virtual ~ClassName();

protected:

    void boolCallback(const std_msgs::Bool &Bool_msg);

 	//void goToCmdActiveCB();
    	//void goToCmdFeedbackCB(const ias_robcom_msgs::GoToFeedbackConstPtr& feedback);
    	//void goToCmdDoneCb(const actionlib::SimpleClientGoalState& state,
                          //const ias_robcom_msgs::GoToResultConstPtr& result);

	ros::NodeHandle& nh;
	ros::NodeHandle& p_nh;
	
private:
    ros::Publisher pub_; // uncommented this for TRYING 3
    ros::Subscriber sub_;

	//BlaActionClient* bla_action_client;

};




} // end of namespace another_test

#endif // another_test_H
