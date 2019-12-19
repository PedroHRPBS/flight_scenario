#pragma once
#include "FlightElement.hpp"
#include "ArmDataMessage.hpp"
#include "positioning_system/Arm.h"

class Arm : public FlightElement{

private:
    ros::ServiceClient arm_client;

public:

    void perform();
    void receive_msg_data(DataMessage* t_msg);
    
    Arm(ros::NodeHandle&);
    ~Arm();
};