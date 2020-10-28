#pragma once
#include "FlightElement.hpp"
#include "ArmDataMessage.hpp"
#include "flight_controller/Arm.h"

class Disarm : public FlightElement{

private:
    Port* _output_port_0;
    ros::ServiceClient _disarm_client;

public:
    enum ports_id {OP_0};
    void process(DataMessage* t_msg, Port* t_port) {};

    void perform();
    
    //Disarm(ros::NodeHandle&);
    Disarm();
    ~Disarm();
};