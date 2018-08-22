#include <another_test/another_test.h>
namespace another_test{


ClassName::ClassName(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	: nh(nh)
	, p_nh(private_nh)

{
    std::cout << "Ready to receive message" << std::endl;
    pub_=p_nh.advertise<std_msgs::Bool>("topic_received", 1, false);
    sub_=nh.subscribe("/lina_test_node/topic_lina",10,&ClassName::boolCallback,this);
    //bla_action_client = new  BlaActionClient("bla/goto", true);
}


ClassName::~ClassName()
{}

void ClassName::boolCallback(const std_msgs::Bool &bool_msg){
     std::cout << "Bool message was received." << std::endl;
     if (bool_msg.data){
         std::cout << "Message is TRUE" << std::endl;
     }

     else
     {
         std::cout << "Message is FALSE" << std::endl;
     }


     std::cout << "Sending confirmation message" << std::endl;
     std_msgs::Bool msg;
     msg.data=true;
     pub_.publish(msg);



}



} // end of namespace another_test
