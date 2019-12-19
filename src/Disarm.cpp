
#include "Disarm.hpp"

Disarm::Disarm(ros::NodeHandle& t_main_handler) : FlightElement(t_main_handler){

    _disarm_client = t_main_handler.serviceClient<positioning_system::Arm>("arm");

}
Disarm::~Disarm(){
    
}

void Disarm::perform(){
    
    // bool _arm_message;
    // _arm_message = false;
    // this->emit_message((DataMessage*)&_arm_message);

    positioning_system::Arm srv;
    srv.request.armed = false;
    _disarm_client.call(srv);

    if (_disarm_client.call(srv))
    {
        ROS_INFO("DISARMED");
    }
    else
    {
        ROS_ERROR("Failed to call service /arm");
    }

}
void Disarm::receive_msg_data(DataMessage* t_msg){

}


