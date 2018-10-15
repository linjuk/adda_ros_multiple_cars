#ifndef lina_test_H
#define lina_test_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
//include <actionlib/client/simple_action_client.h>

namespace lina_test{
//typedef actionlib::SimpleActionClient<bla_msgs::GoToAction> BlaActionClient;
class ClassName {

public:
	ClassName(ros::NodeHandle& nh, ros::NodeHandle& private_nh );
    virtual ~ClassName();

protected:

    void receivedCallback(const std_msgs::Bool &Bool_msg); // uncommented this for TRYING 3

 	//void goToCmdActiveCB();
    	//void goToCmdFeedbackCB(const ias_robcom_msgs::GoToFeedbackConstPtr& feedback);
    	//void goToCmdDoneCb(const actionlib::SimpleClientGoalState& state,
                          //const ias_robcom_msgs::GoToResultConstPtr& result);

	ros::NodeHandle& nh;
	ros::NodeHandle& p_nh;
	
private:
    ros::Publisher pub_;
    ros::Subscriber sub_; // uncommented this for TRYING 3

	//BlaActionClient* bla_action_client;

};

} // end of namespace lina_test

#endif // lina_test_H
