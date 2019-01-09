

#include <lina_test/lina_test.h>
namespace lina_test{

ClassName::ClassName(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	: nh(nh)
	, p_nh(private_nh)

{
    std::cout << "Starting to publish message" << std::endl;
    pub_=p_nh.advertise<std_msgs::Bool>("topic_lina", 1, false);
    sub_=nh.subscribe("/another_test_node/topic_received",10,&ClassName::receivedCallback,this);
    //bla_action_client = new  BlaActionClient("bla/goto", true);

    sleep(1);
    std_msgs::Bool msg;
    msg.data=true;
    pub_.publish(msg);


// Trying #1: loop
        while (ros::ok())
        {
               sleep(2);
               std_msgs::Bool msg;
               msg.data=true;
               msg.data=false;
               pub_.publish(msg);        }
// End of Trying #1.

}


ClassName::~ClassName()
{}


void ClassName::receivedCallback(const std_msgs::Bool &bool_msg){
     std::cout << "Bool message was received." << std::endl;
}


//void ClassName::blaCallback(const bla_msgs::Bla &bla_msg){
//}


// Trying #3:



// End of Trying #3.

} // end of namespace lina_test
