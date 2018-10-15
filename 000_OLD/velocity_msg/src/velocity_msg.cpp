#include <velocity_msg/velocity_msg.h>
#include <nav_msgs/Odometry.h>
namespace velocity_msg{

CarControl::CarControl(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	: nh(nh)
	, p_nh(private_nh)
{
    std::cout << "Starting to publish velocity" << std::endl;
    pub_=p_nh.advertise<geometry_msgs::Twist>("topic_velocity", 1, false);

   sub_=nh.subscribe("/odom",10,&CarControl::odometry_callback,this); //uncommented for odometry
    //bla_action_client = new  BlaActionClient("bla/goto", true);

    while (ros::ok())
          {
        // sleep(0.5);

                   geometry_msgs::Twist msg;
                   msg.linear.x = 0.6;
                   msg.linear.y = 0.0;
                   msg.linear.z = 0.0;
                   msg.angular.x = 0.40;
                   msg.angular.y = 0.0;
                   msg.angular.z = 0.0;

                   pub_.publish(msg);

                  // sleep (5);

               //    geometry_msgs::Twist msg;
                //   msg.linear.x = 0.0;


                 //  pub_.publish(msg);
         }
}

CarControl::~CarControl()
{}

// void CarControl::odometry_callback(nav_msgs::Odometry &odom_msg){
//     std::cout << "Odometry message was received." << std::endl;

//     if (odom_msg.pose.pose.position.x > 5)
//     {
//         odom_msg.pose.pose.position.x = 0;
//     }

//     else
//     {
//         odom_msg.pose.pose.position.x = 2.0;
//   }

// }

//void ClassName::blaCallback(const bla_msgs::Bla &bla_msg){
//}

} // end of namespace velocity_msg
