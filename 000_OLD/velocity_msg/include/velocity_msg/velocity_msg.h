#ifndef velocity_msg_H
#define velocity_msg_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> // add for odometry

//include <actionlib/client/simple_action_client.h>

namespace velocity_msg{
//typedef actionlib::SimpleActionClient<bla_msgs::GoToAction> BlaActionClient;
class CarControl {

public:
    CarControl(ros::NodeHandle& nh, ros::NodeHandle& private_nh );
    virtual ~CarControl();

protected:

    void odometry_callback(nav_msgs::Odometry &odom_msg); // uncommented for odometry

 	//void goToCmdActiveCB();
    	//void goToCmdFeedbackCB(const ias_robcom_msgs::GoToFeedbackConstPtr& feedback);
    	//void goToCmdDoneCb(const actionlib::SimpleClientGoalState& state,
                          //const ias_robcom_msgs::GoToResultConstPtr& result);

	ros::NodeHandle& nh;
	ros::NodeHandle& p_nh;
	
private:
    ros::Publisher pub_;
    ros::Subscriber sub_; // uncommented for odometry
	//BlaActionClient* bla_action_client;

};

} // end of namespace velocity_msg

#endif // velocity_msg_H
