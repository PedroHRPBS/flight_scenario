#pragma once
#include <flight_controller/Reset_Controller.h>
#include "common_srv/ROSUnit.hpp"
#include "MessageToBlock.hpp"
#include "ResetControllerMessage.hpp"

class ROSUnit_ResetController : public ROSUnit {

private:
    ros::ServiceClient _reset_controller_client;

public:
    void receiveMsgData(DataMessage* t_msg);
    ROSUnit_ResetController(ros::NodeHandle&);
    ~ROSUnit_ResetController();
};