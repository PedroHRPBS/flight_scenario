#pragma once
#include "ROSUnit.hpp"
#include "positioning_system/Waypoint.h"
#include "ReferenceMessage.hpp"
#include "MessageToBlock.hpp"

class ROSUnit_UpdateReference : public ROSUnit{

private:
    ros::Publisher _setpoint_position;

public:
    void receive_msg_data(DataMessage* t_msg);
    

    ROSUnit_UpdateReference(ros::NodeHandle&);
    ~ROSUnit_UpdateReference();
};