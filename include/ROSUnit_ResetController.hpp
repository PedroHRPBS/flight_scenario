#pragma once
#include <positioning_system/Reset_Controller.h>
#include "ROSUnit.hpp"
#include "PIDDataMessage.hpp"
#include "MessageToBlock.hpp"
#include "ResetControllerMessage.hpp"

class ROSUnit_ResetController : public ROSUnit {

private:
    ros::ServiceClient _reset_controller_client;

public:
    void receive_msg_data(DataMessage* t_msg);
    ROSUnit_ResetController(ros::NodeHandle&);
    ~ROSUnit_ResetController();
};