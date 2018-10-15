#include <car_teleop/car_teleop.h>
namespace car_teleop{

ClassName::ClassName(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	: nh(nh)
	, p_nh(private_nh)
{   //pub_=p_nh.advertise<blub_msgs::Blub>("topic_name", 1, false);
    //sub_=nh.subscribe("/topic_name",10,&ClassName::blaCallback,this);
    //bla_action_client = new  BlaActionClient("bla/goto", true);
}


ClassName::~ClassName()
{}

//void ClassName::blaCallback(const bla_msgs::Bla &bla_msg){
//}

} // end of namespace car_teleop
