#ifndef car_teleop_H
#define car_teleop_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//include <actionlib/client/simple_action_client.h>

namespace car_teleop{
//typedef actionlib::SimpleActionClient<bla_msgs::GoToAction> BlaActionClient;
class TeleopCar {

public:
    TeleopCar(ros::NodeHandle& nh, ros::NodeHandle& private_nh );
    virtual ~TeleopCar();

protected:

	//void blaCallback(const bla_msgs::Bla &bla_msg);

 	//void goToCmdActiveCB();
    	//void goToCmdFeedbackCB(const ias_robcom_msgs::GoToFeedbackConstPtr& feedback);
    	//void goToCmdDoneCb(const actionlib::SimpleClientGoalState& state,
                          //const ias_robcom_msgs::GoToResultConstPtr& result);

	ros::NodeHandle& nh;
	ros::NodeHandle& p_nh;
	
private:

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle nh_;
    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

    //ros::Publisher pub_;
        //ros::Subscriber sub_;
	//BlaActionClient* bla_action_client;

};

} // end of namespace car_teleop

#endif // car_teleop_H
