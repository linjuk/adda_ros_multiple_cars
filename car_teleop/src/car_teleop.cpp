#include <car_teleop/car_teleop.h>
namespace car_teleop{

TeleopCar::TeleopCar(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : nh(nh),
      p_nh(private_nh),
      linear_(1),
      angular_(2)

{   //pub_=p_nh.advertise<blub_msgs::Blub>("topic_name", 1, false);
    //sub_=nh.subscribe("/topic_name",10,&ClassName::blaCallback,this);
    //bla_action_client = new  BlaActionClient("bla/goto", true);

    nh_.param("axis_linear", linear_, 1);
    nh_.param("axis_angular", angular_, 3);
    nh_.param("scale_angular", a_scale_, 1.0);
    nh_.param("scale_linear", l_scale_, 1.0);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("car2/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopCar::joyCallback, this);


}

void TeleopCar::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*joy->axes[angular_];
    twist.linear.x = l_scale_*joy->axes[linear_];

    vel_pub_.publish(twist);
}


TeleopCar::~TeleopCar()
{}

//void ClassName::blaCallback(const bla_msgs::Bla &bla_msg){
//}

} // end of namespace car_teleop
