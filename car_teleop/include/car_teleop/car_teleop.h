#ifndef car_teleop_H
#define car_teleop_H

#include <ros/ros.h>
//include <actionlib/client/simple_action_client.h>

namespace car_teleop{
//typedef actionlib::SimpleActionClient<bla_msgs::GoToAction> BlaActionClient;
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
	//BlaActionClient* bla_action_client;

};

} // end of namespace car_teleop

#endif // car_teleop_H
