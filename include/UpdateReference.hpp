#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "ReferenceMessage.hpp"
#include "ControlSystemMessage.hpp"
#include <positioning_system/Waypoint.h>

class UpdateReference : public FlightElement{

private:
    ros::Publisher _setpoint_position;
    

public:

	block_id target_block;
    void perform();
    void receive_msg_data(DataMessage* t_msg);
    
    UpdateReference(ros::NodeHandle&);
    ~UpdateReference();
};
