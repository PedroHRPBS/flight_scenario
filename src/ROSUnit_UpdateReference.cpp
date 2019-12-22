#include "ROSUnit_UpdateReference.hpp"

ROSUnit_UpdateReference::ROSUnit_UpdateReference(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _setpoint_position = t_main_handler.advertise<positioning_system::Waypoint>("/setpoint_position/local", 10);

}

ROSUnit_UpdateReference::~ROSUnit_UpdateReference() {

}

void ROSUnit_UpdateReference::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::reference){

        ReferenceMessage* ref_msg = (ReferenceMessage*)t_msg;

        positioning_system::Waypoint msg;
        msg.x = 0.f;
        msg.y = 0.f;
        msg.z = 1.f;
        msg.yaw = 0.f;
        _setpoint_position.publish(msg);

    }
    

}
