#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "UpdatePoseMessage_FS.hpp"
#include "MessageToBlock.hpp"
#include "PositionMsg.hpp"
#include "Vector3DMessage.hpp"

class SetReference_Y : public FlightElement{

private:
    UpdatePoseMessage_FS _pose_reference;

public:
	block_id target_block;
    float setpoint_y;
    
    void perform();
    void receive_msg_data(DataMessage* t_msg);
    
    SetReference_Y();
    ~SetReference_Y();
};
