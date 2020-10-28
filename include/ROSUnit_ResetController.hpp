#pragma once
#include <flight_controller/Reset_Controller.h>
#include "common_srv/ROSUnit.hpp"
#include "MessageToBlock.hpp"
#include "ResetControllerMessage.hpp"

class ROSUnit_ResetController : public ROSUnit {

private:
    Port* _input_port_0;
    ros::ServiceClient _reset_controller_client;

public:
    enum ports_id {IP_0};
    void process(DataMessage* t_msg, Port* t_port);
    ROSUnit_ResetController(ros::NodeHandle&);
    ~ROSUnit_ResetController();
};