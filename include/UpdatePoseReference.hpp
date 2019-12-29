#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "UpdatePoseMessage_FS.hpp"
#include "MessageToBlock.hpp"
class UpdatePoseReference : public FlightElement{
private:
	
public:
	block_id target_block;
    UpdatePoseMessage_FS pose_reference;

    void perform();

    void receive_msg_data(DataMessage* t_msg);
    
    UpdatePoseReference();
    ~UpdatePoseReference();
};
