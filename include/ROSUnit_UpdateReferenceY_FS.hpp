#pragma once
#include "ROSUnit.hpp"
#include "positioning_system/Waypoint.h"
#include <positioning_system/Update_Y_Reference.h>
#include "UpdatePoseMessage_FS.hpp"
#include "MessageToBlock.hpp"
#include "ErrorMsg.hpp"

class ROSUnit_UpdateReferenceY_FS : public ROSUnit{

private:
    ros::ServiceClient _setpoint_position_client;
    ErrorMsg _error_msg;
public:
    void receive_msg_data(DataMessage* t_msg);
    

    ROSUnit_UpdateReferenceY_FS(ros::NodeHandle&);
    ~ROSUnit_UpdateReferenceY_FS();
};