#pragma once
#include "ROSUnit.hpp"
#include "positioning_system/Waypoint.h"
#include <positioning_system/Update_Pose_Reference.h>
#include "UpdatePoseMessage_FS.hpp"
#include "MessageToBlock.hpp"

class ROSUnit_UpdatePoseReference : public ROSUnit{

private:
    ros::ServiceClient _setpoint_position_client;

public:
    void receive_msg_data(DataMessage* t_msg);
    

    ROSUnit_UpdatePoseReference(ros::NodeHandle&);
    ~ROSUnit_UpdatePoseReference();
};