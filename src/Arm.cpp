#include "Arm.hpp"

Arm::Arm(ros::NodeHandle& t_main_handler) : FlightElement(t_main_handler) {

    _arm_client = t_main_handler.serviceClient<positioning_system::Arm>("arm");

}

Arm::~Arm() {

}

void Arm::perform()
{
    // ArmDataMessage _arm_message;
    // _arm_message.isArmed = 1;
    // this->emit_message((DataMessage*)&_arm_message);
    positioning_system::Arm srv;
    srv.request.armed = true;
    _arm_client.call(srv);

    if (_arm_client.call(srv))
    {
        ROS_INFO("ARMED");
    }
    else
    {
        ROS_ERROR("Failed to call service /arm");
    }

}

void Arm::receive_msg_data(DataMessage* t_msg){

}