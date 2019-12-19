#pragma once
#include "FlightElement.hpp"
#include "UpdateReference.hpp"

// UpdateReference::UpdateReference(ros::NodeHandle& t_main_handler) : FlightElement(t_main_handler){

//     _setpoint_position = t_main_handler.advertise<positioning_system::Waypoint>("/setpoint_position/local", 10);

// }

UpdateReference::UpdateReference(){}

UpdateReference::~UpdateReference(){}

void UpdateReference::perform(){
    ReferenceMessage _reference_update_msg;
    ((MessageToBlock*)&_reference_update_msg)->target_block=this->target_block;
    this->emit_message((DataMessage*)&_reference_update_msg);

    // positioning_system::Waypoint msg;
    // msg.x = 0.f;
    // msg.y = 0.f;
    // msg.z = 1.f;
    // msg.yaw = 0.f;
    // _setpoint_position.publish(msg);

}

void UpdateReference::receive_msg_data(DataMessage* t_msg){}



