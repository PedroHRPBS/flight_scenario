#pragma once
#include <positioning_system/Update_Controller.h>
#include "ROSUnit.hpp"
#include "ControllerMessage.hpp"
#include <positioning_system/Controller_param.h>

class ROSUnit_UpdateController : public ROSUnit {

private:
    ros::ServiceClient _update_controller_client;

public:
    void receive_msg_data(DataMessage* t_msg);
    ROSUnit_UpdateController(ros::NodeHandle&);
    ~ROSUnit_UpdateController();
};