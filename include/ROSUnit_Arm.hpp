#pragma once
#include "common_srv/ROSUnit.hpp"
#include "flight_controller/Arm.h"
#include "ArmDataMessage.hpp"

class ROSUnit_Arm : public ROSUnit{

private:
    Port* _input_port_0;
    ros::ServiceClient _arm_client;

public:
    enum ports_id {IP_0};
    void process(DataMessage* t_msg, Port* t_port);

    ROSUnit_Arm(ros::NodeHandle&);
    ~ROSUnit_Arm();
};